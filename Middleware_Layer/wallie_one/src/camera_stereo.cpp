#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/CameraInfo.h>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <wallie_one/CameraStereoConfig.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_listener.h>
#include <boost/thread/mutex.hpp>
#include <string>
#include <opencv2/core/hal/interface.h>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <math.h>
#include <vector>
#include <malloc.h>
#include <memory.h>
#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>

using namespace cv;
using namespace std;

VideoCapture c;

std::vector<float> genCoeffs(int cols, int rows, std::vector<float> args)
{
	std::vector<float> coeff(10,0);
	int number_values = 0;
	int number_args = args.size();
	int num_coeffs, cp_size,cp_x,cp_y,cp_vals;
	if(number_values==0)
	{
		number_values = 2;
		cp_vals = 0;
		cp_x = 2;
		cp_y = 3;
	}
	else
	{
		cp_vals = 2;
		cp_x = 0;
		cp_y = 1;
	}
	cp_size = number_values+2;
	num_coeffs = 10;
	float rscale = 2.0f/(cols<rows ? cols:rows);
	coeff[0] = args[0];
	coeff[1] = args[1];
	coeff[2] = args[2];
	if(number_args == 3 || number_args == 5){
		coeff[3] = 1.0f-coeff[0]-coeff[1]-coeff[2];
	}
	else{
		coeff[3] = args[3];
	}
	coeff[0] *= rscale*rscale*rscale;
	coeff[1] *= rscale*rscale;
	coeff[2] *= rscale;
	coeff[4] = coeff[0];
	coeff[5] = coeff[1];
	coeff[6] = coeff[2];
	coeff[7] = coeff[3];
	coeff[8] = cols/2.0f;
	coeff[9] = rows/2.0f;
	return coeff;
}


using namespace cv;
using namespace std;
class CameraDriver
{
public:

	typedef wallie_one::CameraStereoConfig Config;
    typedef camera_info_manager::CameraInfoManager CameraInfoManager;

    enum
    {
        LEFT        = 0,
        RIGHT       = 1,
        NUM_CAMERAS
    };

    static const int DEFAULT_CAMERA_INDEX[NUM_CAMERAS];
    static const double DEFAULT_FPS;
    static const char* DEFAULT_CAMERA_NAME;
    int h = 480;
    int w = 640;
    Mat map_x, map_y;
    static const std::string CameraString[NUM_CAMERAS];

	CameraDriver()
	:
		nh( "~" ),
        camera_nh( "stereo" ),
        it( nh ),
        server( nh ),
		reconfiguring( false )
		{
				
			map_x.create(w, h, CV_32FC1);
			map_y.create(w, h, CV_32FC1);
			update_map(coeff);
			ROS_INFO_STREAM("Created Mapping.");
			int backups[NUM_CAMERAS] = {0,1};
			
			ROS_INFO_STREAM("Starting Subscribers.");
			
			imgL_sub = nh.subscribe("/zed_node/left/image_rect_color", 10, &CameraDriver::imgL_cb, this);
			
			imgR_sub = nh.subscribe("/zed_node/right/image_rect_color", 10, &CameraDriver::imgL_cb, this);
			nh.param<std::string>( "camera_name", camera_name, DEFAULT_CAMERA_NAME );
			nh.param<double>( "fps", fps, DEFAULT_FPS );

			for( size_t i = 0; i < NUM_CAMERAS; ++i )
			{
				single_camera_nh[i] = ros::NodeHandle( camera_nh, CameraString[i] );

				camera_info_manager[i] = boost::make_shared< CameraInfoManager >( single_camera_nh[i] );
				setCameraName( *camera_info_manager[i], camera_name + "_" + CameraString[i] );

				frame[i] = boost::make_shared<cv_bridge::CvImage>();
					
				frame[i]->encoding  = sensor_msgs::image_encodings::BGR8;
				
				camera_pub[i] = it.advertiseCamera( CameraString[i] + "/image_raw", 1 );
			}
			
			stereo_frame = boost::make_shared< cv_bridge::CvImage >();
			stereo_frame->encoding  = sensor_msgs::image_encodings::BGR8;
			stereo_pub = it.advertiseCamera("/stereo/image_raw",1);
			camera_info = boost::make_shared< sensor_msgs::CameraInfo >();
			
			server.setCallback( boost::bind( &CameraDriver::reconfig, this, _1, _2 ) );

		}

	~CameraDriver()
	{
	}
	
	void update_map(std::vector<float> coeff)
	{

	  double dx, dy, r, ux, uy;
		for (int y = 0; y < h; y++)
		{
			dy = y-coeff[9]+0.5;
			for (int x = 0; x < w; x++)
			{         
				dx = x-coeff[8]+0.5;
				//printf("dx: %4.2f, dy = %4.2f\n", dx,dy);
				r = sqrt(dx*dx+dy*dy);
				ux = dx*(r*r*r*coeff[0]+r*r*coeff[1]+r*coeff[2]+coeff[3]);
				uy = dy*(r*r*r*coeff[4]+r*r*coeff[5]+r*coeff[6]+coeff[7]);
				ux += coeff[8];
				uy += coeff[9];
				//printf("ux: %4.2f, uy = %4.2f\n", ux,uy);
				map_x.at<float>(y, x) = ux;
				map_y.at<float>(y, x) = uy;
				//proj(y,x) = Vec2f(float(ux), float(uy));
				//printf("ux: %4.2f, uy = %4.2f\n", xMap.at<float>(y, x),yMap.at<float>(y, x));
				
			}
		}
	}

	void reconfig( Config& newconfig, uint32_t level )
	{
		reconfiguring = true;
		boost::mutex::scoped_lock lock( mutex );

		// Resolve frame ID using tf_prefix parameter:
		if( newconfig.frame_id == "" )
		{
		    newconfig.frame_id = "camera";
		}
		std::string tf_prefix = tf::getPrefixParam( nh );
		ROS_DEBUG_STREAM( "tf_prefix = " << tf_prefix );
		newconfig.frame_id = tf::resolve( tf_prefix, newconfig.frame_id );

		setCameraInfo( *camera_info_manager[LEFT] , config.camera_info_url_left , newconfig.camera_info_url_left  );
		setCameraInfo( *camera_info_manager[RIGHT], config.camera_info_url_right, newconfig.camera_info_url_right );

		for( size_t i = 0; i < NUM_CAMERAS; ++i )
		{
		    
		    //newconfig.fps          = setProperty( camera[i], CV_CAP_PROP_FPS         , newconfig.fps          );
		    newconfig.brightness   = setProperty( camera[i], CAP_PROP_BRIGHTNESS  , newconfig.brightness   );
		    newconfig.contrast     = setProperty( camera[i], CAP_PROP_CONTRAST    , newconfig.contrast     );
		    newconfig.saturation   = setProperty( camera[i], CAP_PROP_SATURATION  , newconfig.saturation   );
		    newconfig.hue          = setProperty( camera[i], CAP_PROP_HUE         , newconfig.hue          );
		    newconfig.gain         = setProperty( camera[i], CAP_PROP_GAIN        , newconfig.gain         );
		    newconfig.exposure     = setProperty( camera[i], CAP_PROP_EXPOSURE    , newconfig.exposure     );

		    //setFOURCC( camera[i], newconfig.fourcc );

		    frame[i]->header.frame_id = newconfig.frame_id;
		}
		if( fps != newconfig.fps )
		{
		    fps = newconfig.fps;
		    timer.setPeriod( ros::Duration( 1. / fps ) );
		}
		if(w != newconfig.frame_width || h != newconfig.frame_height || args[0] != newconfig.A || args[1] != newconfig.B || args[2] != newconfig.C || args[3] != newconfig.D)
		{	

			for( size_t i = 0; i < NUM_CAMERAS; ++i )
			{
				//newconfig.frame_width  = setProperty( camera[i], CAP_PROP_FRAME_WIDTH , newconfig.frame_width  );
				//newconfig.frame_height = setProperty( camera[i], CAP_PROP_FRAME_HEIGHT, newconfig.frame_height );
			}
			args[0] = newconfig.A;
			args[1] = newconfig.B;
			args[2] = newconfig.C;
			args[3] = newconfig.D;
			w = 640;//newconfig.frame_width;
			h = 480;//.frame_height;
			coeff = genCoeffs(w,h,args);
			map_x.create(w, h, CV_32FC1);
			map_y.create(w, h, CV_32FC1);
			printf("%d\n",map_x.empty());
			printf("%d\n",map_y.empty());
			update_map(coeff);
		}
		config = newconfig;
		reconfiguring = false;
	}

	void capture( const ros::TimerEvent& te )
	{
		if( not reconfiguring )
		{
			boost::mutex::scoped_lock lock( mutex );
			
		    for( size_t i = 0; i < NUM_CAMERAS; ++i )
		    {
				    camera[i] >> frame[i]->image;
				if( frame[i]->image.empty() ) return;
		    }

		    ros::Time now = ros::Time::now();

		    for( size_t i = 0; i < NUM_CAMERAS; ++i )
		    {
					src = frame[i]->image;
					dst.create( w,h, src.type() );
					remap( src, dst, map_x, map_y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0) );
					dst =  dst(Range(int(h/5),int(h/5)+int(h/2)+int(h/5)),Range(int(h/3),h));	
					resize(dst,sized,Size(int(w/2),h),INTER_LINEAR);
					frame[i]->image = sized;
					frame[i]->header.stamp = now;
					*camera_info = camera_info_manager[i]->getCameraInfo();
					camera_info->header = frame[i]->header;
					
					camera_pub[i].publish( frame[i]->toImageMsg(), camera_info );
					
		    }
/*



*/
		    if(frame[0]->image.empty() || frame[1]->image.empty())
		    {
			return;
		    }
		    else
		    {
			int resultImgW = frame[0]->image.cols+frame[1]->image.cols;
			int resultImgH = frame[0]->image.rows;
			/*for(size_t i = 0; i < frame[0]->image.cols;i++)
			{
				result.col(i) = frame[0]->image.col(i);
				
			}
			for(size_t i = 0; i < frame[0]->image.cols;i++)
			{
				result.col(i+frame[0]->image.cols) = frame[1]->image.col(i);
				
			}*/
			
			printf("Source1: w = %d,h = %d\n",frame[0]->image.cols, frame[0]->image.rows);
			    // Create a black image
			res.create(resultImgH,resultImgW, frame[0]->image.type());
			printf("Dest: w = %d,h = %d\n",res.cols,res.rows);
			    // Copy images in correct position
			frame[0]->image.copyTo(res(Rect(0, 0, frame[0]->image.cols, resultImgH)));
			frame[1]->image.copyTo(res(Rect(frame[0]->image.cols, 0, frame[0]->image.cols, resultImgH)));
			
			stereo_frame->image = res;
			stereo_frame->header.stamp = now;
			printf("%d\n",stereo_frame->image.empty());
			*camera_info = camera_info_manager[0]->getCameraInfo();
			camera_info->header = frame[0]->header;
		        stereo_pub.publish(stereo_frame->toImageMsg(),camera_info);
		    }
		}
		
		
		
	
	}

private:
	
    void setCameraName( CameraInfoManager& camera_info_manager, const std::string& camera_name )
    {
        if( not camera_info_manager.setCameraName( camera_name ) )
        {
            ROS_ERROR_STREAM( "Invalid camera name '" << camera_name << "'" );
            ros::shutdown();
        }
    }

    void setCameraInfo( CameraInfoManager& camera_info_manager, const std::string& camera_info_url, std::string& camera_info_url_new )
    {
	    if( camera_info_url != camera_info_url_new )
    	{
	        if( camera_info_manager.validateURL( camera_info_url_new ) )
		    {
    			camera_info_manager.loadCameraInfo( camera_info_url_new );
	    	}
		    else
    		{
	    		camera_info_url_new = camera_info_url;
		    }
    	}
    }
    
    double setProperty( cv::VideoCapture& camera, int property, double value )
    {
        if( camera.set( property, value ) )
        {
            double current_value = camera.get( property );
            ROS_WARN_STREAM(
                    "Failed to set property #" << property << " to " << value <<
                    " (current value = " << current_value << ")"
            );
            return current_value;
        }

        return value;
    }

    std::string setFOURCC( cv::VideoCapture& camera, std::string& value )
    {
        ROS_ASSERT_MSG( value.size() == 4, "Invalid FOURCC codec" );

        int property = CAP_PROP_FOURCC;
        int fourcc = cv::VideoWriter::fourcc( value[0], value[1], value[2], value[3] );
        if( camera.set( property, fourcc ) )
        {
            fourcc = camera.get( property );
            std::string current_value = fourccToString( fourcc );
            ROS_WARN_STREAM(
                "Failed to set FOURCC codec to '" << value <<
                "' (current value = '" << current_value << "' = " <<  fourcc << ")"
            );
            return current_value;
        }

        return value;
    }

    std::string fourccToString( int fourcc )
    {
    
        std::string str( 4, ' ' );
        
        for( size_t i = 0; i < 4; ++i )
        {
            str[i] = fourcc & 255;
            fourcc >>= 8;
        }
        
        return str;
    }

private:

	ros::NodeHandle nh, camera_nh, single_camera_nh[NUM_CAMERAS];
	image_transport::ImageTransport it;
	image_transport::CameraPublisher camera_pub[NUM_CAMERAS];
	image_transport::CameraPublisher stereo_pub;
	sensor_msgs::CameraInfoPtr camera_info;
	boost::shared_ptr< CameraInfoManager > camera_info_manager[NUM_CAMERAS];
	std::string camera_name;
	
	Config config;
	dynamic_reconfigure::Server< Config > server;
	bool reconfiguring;
	boost::mutex mutex;
	Mat src, dst, sized, joined, res;
	
	const char* remap_window = "Remap demo";
	// Make out subcribers to ZED feed
	ros::Subscriber imgL_sub;
	ros::Subscriber imgR_sub;
	cv_bridge::CvImagePtr frame[NUM_CAMERAS];
	cv_bridge::CvImagePtr stereo_frame;
	std::vector<float> args = {0.3,0.0,0.1,1.0};
	std::vector<float> coeff = genCoeffs(w,h,args);

	long totalTime = 0;
	long captures = 0;
	int camera_index[NUM_CAMERAS] = {2,4};
	double fps;
};

const int CameraDriver::DEFAULT_CAMERA_INDEX[NUM_CAMERAS] = {2, 4};
const double CameraDriver::DEFAULT_FPS = 30.;
const char* CameraDriver::DEFAULT_CAMERA_NAME = "logitech_c120";

const std::string CameraDriver::CameraString[NUM_CAMERAS] = {"left", "right"};

using namespace std;
int main( int argc, char* argv[] )
{
	ros::init( argc, argv, "camera_stereo" );
	cout << "OpenCV version : " << CV_VERSION << endl;
	cout << "Major version : " << CV_MAJOR_VERSION << endl;
	cout << "Minor version : " << CV_MINOR_VERSION << endl;
	cout << "Subminor version : " << CV_SUBMINOR_VERSION << endl;
	CameraDriver camera_driver;

	while( ros::ok() )
	{
		ros::spin();
	}

	return EXIT_SUCCESS;
}

