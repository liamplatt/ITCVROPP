#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <wallie_one/ZedPubStereoConfig.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


#include <image_geometry/pinhole_camera_model.h>


#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/hal/interface.h>

#include <boost/thread/mutex.hpp>
#include <string>
#include <math.h>
#include <vector>
#include <malloc.h>
#include <memory.h>
#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <ctime>
#include <cmath>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace cv;

std::vector<float> genCoeffs(int cols, int rows, std::vector<float> args)
{
	printf("Regenerating Coeffs...\n");
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
	
	printf("Regenerated Coeffs: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6], coeff[7], coeff[8], coeff[9]);
	return coeff;
}



const double DEFAULT_FPS = 15;
class ZPS
{

public:
	typedef wallie_one::ZedPubStereoConfig Config;
	typedef camera_info_manager::CameraInfoManager CameraInfoManager;
	int h = 720;
	int w = 1280;
	double hScale; 
	double wScale;
	bool R = 0;
	bool L = 0;
	
	
	//pcl::PointCloud<pcl::PointXYZRGB> ros_cloudR, ros_cloudL;
	sensor_msgs::CameraInfo cam_info_L, cam_info_R, cam_info;
	double resolution = 1.0f;
	ros::NodeHandle n, camera_nh;
	ros::Timer timer;
	ros::Subscriber objects;
	//image_geometry::PinholeCameraModel cam_model;
	// Create seperate mapping for left and right images
	Mat map_x_r, map_y_r, map_x_l, map_y_l, res, srcR, dstR, sizedR, srcL, dstL, sizedL;
	cv_bridge::CvImagePtr stereo_frame;
	cv_bridge::CvImagePtr left, right;
	image_transport::CameraPublisher stereo_pub;
	sensor_msgs::CameraInfoPtr cameraInfo;
	image_transport::ImageTransport it;
	
	boost::shared_ptr<camera_info_manager::CameraInfoManager> c_info;
	
	image_transport::SubscriberFilter subRightRectified;
	image_transport::SubscriberFilter subLeftRectified;
	int pc_data_radius = 2;
	//message_filters::Subscriber<Image> subRightRectified;
	//message_filters::Subscriber<Image> subLeftRectified;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
	
	message_filters::Synchronizer< MySyncPolicy > sync;
	int px[4];
	int py[4];
	bool PCLAvailable;// = false;
	tf::TransformListener listener;
	bool _op_control = false;
	//tf::StampedTransform transformL, transformR;
	//ros::Subscriber subPCL;
	//ros::Publisher pcl_pubL, pcl_pubR;
	//sensor_msgs::PointCloud2 cloudL, cloudR;
	std::string _stereo_namespace = "/zed/zed_node";
	std::string left_frame_op = "/pcl_reader/left_image_PC";
	std::string right_frame_op = "/pcl_reader/right_image_PC";
	std::string left_frame_nop = _stereo_namespace + "/left/image_rect_color";
	std::string right_frame_nop = _stereo_namespace + "/right/image_rect_color";
	std::string targetR;// = "zed2_right_camera_optical_frame";
	std::string targetL;// = "zed2_left_camera_optical_frame";
	std::string camera_name;
	std::vector<float> args = {0.3,0.0,0.1,1.0};
	std::vector<float> coeff = genCoeffs(w,h,args);
	double fps;
	Config config;
	int zed_res = 3; //3 VGA or 2 HD720P
	dynamic_reconfigure::Server<Config> server;
	bool reconfiguring;
	boost::mutex mutex;
	ros::Timer timer1;
	dynamic_reconfigure::Server<Config>::CallbackType cb;
	
public:
	

	ZPS()
	:
	n("~"),
	camera_nh( "stereo" ),
	it(n),
	camera_name("camera"),
	reconfiguring(false),
	PCLAvailable(false),
	server(n),
	c_info(new camera_info_manager::CameraInfoManager(camera_nh)),
	subRightRectified(it, "/pcl_reader_no_imagew/right_image_PC", 1),
	subLeftRectified(it, "/pcl_reader_no_imagew/left_image_PC", 1),
	sync(MySyncPolicy(10), subLeftRectified, subRightRectified),
	targetR("zed_right_camera_optical_frame"),
	targetL("zed_left_camera_optical_frame")
	{

		ROS_INFO("Registering CB Stereo.....\n");
		std::string image_topic = "image_rect_color";
		n.getParam("stereo_namespace", _stereo_namespace);
		n.getParam("image_topic", image_topic);
	        std::cout << _stereo_namespace << std::endl;
		
		left_frame_nop = _stereo_namespace + "/left/" + image_topic;
		right_frame_nop = _stereo_namespace + "/right/" + image_topic;
		sync.registerCallback(boost::bind(&ZPS::imageStereoRectifiedCallback, this, _1, _2));
		ROS_INFO("Setting up c_info_manager...\n");
		if(!c_info->setCameraName(camera_name))
		{
			ROS_INFO("failed to set camera name\n");
		}
		ROS_INFO("Registered CB Stereo.\n");
		stereo_frame = boost::make_shared< cv_bridge::CvImage >();
		stereo_frame->encoding  = sensor_msgs::image_encodings::BGR8;
		args = {0.05,0.06,0.0,0.86};
		coeff = genCoeffs(w,h,args);
		map_x_l.create(w, h, CV_32FC1);
		map_y_l.create(w, h, CV_32FC1);
		map_x_r.create(w, h, CV_32FC1);
		map_y_r.create(w, h, CV_32FC1);
		update_map(coeff);
		timer1 = n.createTimer(ros::Duration(0.05), &ZPS::timerCallback, this);
		ROS_INFO("Created mapping.\nRegisted Timer CB.");
		stereo_pub = it.advertiseCamera("/stereo/image_raw",1);
		//objects = n.subscribe("/zed2/zed_node/obj_det/objects", 10, &ZED_PCL::objectCB, this);
		//subPCL = n.subscribe("/rtabmap/scan_map", 10, &ZPS::PointCloudCB, this);
		//subStereo = n.subscribe("stereo/image_raw", 10, &ZED_PCL::stereoCallback, this);
		//subDepth = n.subscribe("/zed2/zed_node/depth/depth_registered", 10, &ZED_PCL::depthCallback, this);
		//pcl_pubL = n.advertise<sensor_msgs::PointCloud2>("pcl_left",1);
		//pcl_pubR = n.advertise<sensor_msgs::PointCloud2>("pcl_right",1);
		ROS_INFO("Subcribed to ZED2.\n");
		cb = boost::bind( &ZPS::reconfig, this, _1, _2);
		server.setCallback(cb);
		// objects = n.subscribe("/zed2/zed_node/obj_det/objects", 10, &ZPS::objectCB, this);
		ROS_INFO("Server callback successfully set.\n");
		
	}
	
	
	void timerCallback(const ros::TimerEvent&)
	{
		bool op_control = _op_control;
		n.getParam("/zed_stereo_node/op_control", op_control);

		if(_op_control != op_control)
		{
			_op_control = op_control;
			
			ROS_INFO("OP control Change!\n");
			subRightRectified.unsubscribe();
			subLeftRectified.unsubscribe();
			
			if(op_control)
			{
				ROS_INFO("Subscribing to pcl_reader\n");
				subLeftRectified.subscribe(it, "/pcl_reader_no_imagew/left_image_PC", 1);
			
				subRightRectified.subscribe(it, "/pcl_reader_no_imagew/right_image_PC", 1);
			}
			else
			{
				ROS_INFO("Subscribing to zed_node\n");
				subLeftRectified.subscribe(it, left_frame_nop , 1);
			
				subRightRectified.subscribe(it, right_frame_nop , 1);
			}
		}

		

	}
	void reconfig( Config& newconfig, uint32_t level )
	{
		ROS_INFO("Reconfigure Request: %f %f %f %f %f", 
            config.A, config.B, 
            config.C, 
            config.D, 
            config.fps, config.resolution);
		reconfiguring = true;
		boost::mutex::scoped_lock lock( mutex );
		if( newconfig.frame_id == "" )
		{
		    newconfig.frame_id = "camera";
		}
		std::string tf_prefix = tf::getPrefixParam( n );
		newconfig.frame_id = tf::resolve( tf_prefix, newconfig.frame_id );
		ROS_INFO("Bootset\n");
 
		if( fps != newconfig.fps )
		{
			ROS_INFO("FPS is now: \n");
		    fps = newconfig.fps;
			printf("%f\n",fps);
			
		    timer.setPeriod( ros::Duration( 1. / fps ) );

		}

		
		if(args[0] != newconfig.A || args[1] != newconfig.B || args[2] != newconfig.C ||args[3] != newconfig.D || resolution != newconfig.resolution)
		{	
			ROS_INFO("Param Change!\n");
			zed_res = newconfig.resolution;
			if(zed_res == 2)
			{
				w = 1280;
				h = 720;
				
			}
			else
			{
				// Assume resolution 3, resolution is fixed in parameter configuration!
				w = 640;
				h = 480;
				
			}
			args[0] = newconfig.A;
			args[1] = newconfig.B;
			args[2] = newconfig.C;
			args[3] = newconfig.D;
			coeff = genCoeffs(w, h, args);
			map_x_l.create(h, w, CV_32FC1);
			map_y_l.create(h, w, CV_32FC1);
			
			map_x_r.create(h, w, CV_32FC1);
			map_y_r.create(h, w, CV_32FC1);
			update_map(coeff);
		}
		
		config = newconfig;
		reconfiguring = false;

	}


	void update_map(std::vector<float> coeff)
	{
		printf("Updating mapping...W=%d and H=%d\n", w, h);
		double dx, dy, r, ux, uy;
		
		// Width = 640
		// Height = 480
		// Loop over each y coordinate 0:1079
		for (int y = 0; y < h; y++)
		{
			// Now -539.5:539.5
			dy = y - coeff[9] + 0.5;
			// Loop over each x coordinate 0:1919
			for (int x = 0; x < w; x++)
			{         
				// Now -958.5:959.5
				dx = x - coeff[8] + 0.5;
				//printf("dx: %4.2f, dy = %4.2f\n", dx,dy);
				
				
				// Distance between origin and current point
				// Range between 0:399.3
				r = sqrt(dx*dx + dy*dy);
				// Third order polynomial
				//printf("%4.2f,",r);
				// for 0, ux = 
				// for 200 ux
				ux = dx * (r*r*r*coeff[0] + r*r*coeff[1] + r*coeff[2] + coeff[3]);
				uy = dy * (r*r*r*coeff[4] + r*r*coeff[5] + r*coeff[6] + coeff[7]);
				ux += coeff[8];
				uy += coeff[9];
				//printf("ux: %4.2f, uy = %4.2f\n", ux,uy);
				map_x_r.at<float>(y, x) = ux;
				map_y_r.at<float>(y, x) = uy;
				
				
				// TODO create mapping for left hand side
				ux = dx*(r*r*r*coeff[0] + r*r*coeff[1] + r*coeff[2] + coeff[3]);
				uy = dy*(r*r*r*coeff[4] + r*r*coeff[5] + r*coeff[6] + coeff[7]);
				ux += coeff[8];
				uy += coeff[9];
				map_x_l.at<float>(y, x) = ux;
				map_y_l.at<float>(y, x) = uy;
				
				
				//proj(y,x) = Vec2f(float(ux), float(uy));
				//printf("ux: %4.2f, uy = %4.2f\n", xMap.at<float>(y, x),yMap.at<float>(y, x));
				
			}
		}
	}
	
	void imageStereoRectifiedCallback(const sensor_msgs::Image::ConstPtr&  msgL, const sensor_msgs::Image::ConstPtr&  msgR)
	{
		//ROS_INFO("in imageStereoRectifiedCallback!\n");
		//ROS_INFO("w = %d, h = %d\n", w, h);
		double start = ros::Time::now().toSec();
		try
	    	{
			right = cv_bridge::toCvCopy(msgR, sensor_msgs::image_encodings::BGR8);
			//std::printf("W: %d, H: %d\n",right->image.cols, right->image.rows);
			cv::resize(right->image, right->image, Size(w, h));
			srcR = right->image;
			dstR.create(h, w, srcR.type());
			//std::printf("Created right dist empty image.\n");
			remap(srcR, dstR, map_x_r, map_y_r, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
			//std::printf("Successfully remapped right image!\n");
			//dstR =  dstR(Range(int(h/5), int(h/5) + int(h/2) + int(h/5)), Range(int(h/3), h));	
			resize(dstR, sizedR, Size(int(w), h), INTER_LINEAR);
			right->image = sizedR;
			R = 1;		
	    }
	    catch (cv_bridge::Exception& e)
	    {	
			ROS_ERROR("cv_bridge exception: %s", e.what());
			R = 0;
			return;
	    }
		
		// TODO We will need to find a way to "flip" how the distortion is applied on the left side.
		// It should be facing the other direction
	    try
	    {
			//ROS_INFO("Right Rectified image received from ZED - Size: %dx%d",msg->width, msg->height);
			left = cv_bridge::toCvCopy(msgL, sensor_msgs::image_encodings::BGR8);
			cv::resize(left->image, left->image, Size(w, h));
			srcL = left->image;
			dstL.create(h, w, srcL.type());
			// Before remap, ad
			//std::printf("Created left dist empty image.\n");
			remap(srcL, dstL, map_x_l, map_y_l, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0));
			//std::printf("Successfully remapped left image!\n");
			//dstL =  dstL(Range(int(h/5),int(h/5)+int(h/2)+int(h/5)),Range(int(h/3),h));	
			resize(dstL, sizedL, Size(int(w),h), INTER_LINEAR);
			left->image = sizedL;
			L = 1;
	    }
	    catch (cv_bridge::Exception& e)
	    {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			L = 0;
			return;
	    }
		
		double end = ros::Time::now().toSec();
		//ROS_INFO("L and R image -> vr space processed in: %f sec.\n", end-start);
		if(L && R && ~reconfiguring)
		{
			double start1 = ros::Time::now().toSec();
			//std::printf("Received and remapped left and right image.\n");
		    if(left->image.empty() || right->image.empty())
		    {
				std::printf("But both images are empty...\n");
				L = 0;
				R = 0;
				return;
		    }
		    else
		    {
				sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(c_info->getCameraInfo()));
				
				//std::printf("Creating stereo image...\n");
				int resultImgW = left->image.cols + right->image.cols;
				int resultImgH = left->image.rows;
				// std:printf("Resulting image_size: %d, %d\n", resultImgW, resultImgH);
				res.create(resultImgH,resultImgW, left->image.type());
				
				//std::printf("Created empty stereo image...\n");
				
				//std::printf("Copying left image to empty stereo image...\n");
				left->image.copyTo(res(Rect(0, 0, left->image.cols, resultImgH)));
				
				//std::printf("Copied left image!\n");
				//std::printf("Copying right image to empty stereo image...\n");
				right->image.copyTo(res(Rect(left->image.cols, 0, left->image.cols, resultImgH)));
				
				//std::printf("Copied right image!\n");
				ros::Time now = ros::Time::now();
				
				//std::printf("Copying result into stereo frame...\n");
				stereo_frame->image = res;
				stereo_frame->header.stamp = now;
				ci->height = stereo_frame->image.rows;
				ci->width = stereo_frame->image.cols;
				//std::printf("Stereo frame size is- W: %d, H: %d\n", stereo_frame->image.cols, stereo_frame->image.rows);
				//std::printf("Copied result into stereo frame!\n");
				//for(int j = 0; j < 4; j++)
				//{
				//	cv::circle(stereo_frame->image, cv::Point(std::round(px[j]/2),std::round(py[j])),10,Scalar(255, 0, 0));
				//}
				ci->header.stamp = stereo_frame->header.stamp;
				//std::printf("Publishing stereo image...\n");
				stereo_pub.publish(stereo_frame->toImageMsg(), ci);
				//std::printf("Published stereo image!\n");
		    }
			// If data is available project each point in data to left and right images
			
			L = 0;
			R = 0;
			
			double end1 = ros::Time::now().toSec();
			//ROS_INFO("L and R image published in: %f sec.\n", end1-start1);
		}
		
	}
	/*
	void objectCB(const zed_interfaces::ObjectsStamped::ConstPtr& msg)
	{
		
		//ROS_INFO("***** New object list *****");
		for (int i = 0; i < msg->objects.size(); i++)
		{
			if (msg->objects[i].label_id == -1)
			  continue;

			/*ROS_INFO_STREAM(msg->objects[i].label << " [" << msg->objects[i].label_id << "] - Pos. ["
												  << msg->objects[i].position[0] << "," << msg->objects[i].position[1] << ","
												  << msg->objects[i].position[2] << "] [m]"
												  << "- Conf. " << msg->objects[i].confidence
										<< " - Tracking state: " << static_cast<int>(msg->objects[i].tracking_state));
			cv::circle(left->image, msg->objects[i].position[0],msg->objects[i].position[1]), 100, 2.5*msg->objects[i].confidence);
			
			float pnts[3];
			for(int j = 0; j < 4; j++)
			{
				std::printf("Object: %d\n", i);
				std::printf("	Corner: %d\n", j);
				std::printf("		x: %d\n", msg->objects[i].bounding_box_2d.corners[j].kp[0]);
				std::printf("		y: %d\n", msg->objects[i].bounding_box_2d.corners[j].kp[1]);
				px[j] = std::round((msg->objects[i].bounding_box_2d.corners[j].kp[0])/2);
				py[j] = std::round((msg->objects[i].bounding_box_2d.corners[j].kp[1])/2);
				cv::circle(stereo_frame->image, cv::Point(std::round(msg->objects[i].bounding_box_2d.corners[j].kp[0]/2),std::round(msg->objects[i].bounding_box_2d.corners[j].kp[1]/2)),10,255);
			}
		}
	pnts_def = true;	
	}
	*/
private:

    void setCameraInfo( CameraInfoManager& camera_info_manager, const std::string& camera_info_url, std::string& camera_info_url_new )
    {
		ROS_INFO("in setCamInfo!\n");
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
		ROS_INFO("Propset\n");
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

        return value;//http://wiki.ros.org/dynamic_reconfigure/Tutorials/SettingUpDynamicReconfigureForANode%28cpp%29
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
	
	
};


int main(int argc, char** argv)
{
  	ros::init(argc, argv, "zed_sub_stereo");
	ZPS Z;

	while( ros::ok() )
	{
		ros::spin();
	}
  return 0;
}
