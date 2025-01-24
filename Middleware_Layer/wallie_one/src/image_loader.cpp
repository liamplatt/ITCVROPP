#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include <wallie_one/ZedPubStereoConfig.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/UInt64.h>
#include <geometry_msgs/Point.h>

#include <image_geometry/pinhole_camera_model.h>

#include <camera_info_manager/camera_info_manager.h>

#include <image_transport/subscriber_filter.h>
#include <image_transport/image_transport.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <laser_assembler/AssembleScans.h>
#include <laser_assembler/AssembleScans2.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/hal/interface.h>


#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>

#include <boost/thread/mutex.hpp>
#include <glob.h>
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
#include <fstream>
#include <stdio.h>

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
using namespace cv;
class ImageLoader
{

public:
  ros::NodeHandle nh_;
	ros::Subscriber subTime;
	tf::TransformListener listener;
  	image_transport::ImageTransport it_;
		ros::Timer timer;
	ros::Publisher pubCloud;
 	std::string topic_name;
  	std::string topic_name_right = "/zed/zed_nodelet/right/image_rect_color_throttled";
 	std::string topic_name_left = "/zed/zed_nodelet/left/image_rect_color_throttled";
	int time_stamp = 0;
 	cv::Mat image_l, image_r, image_depth;
	Eigen::Matrix4f camera_pose;
	image_transport::Publisher image_pub_r, image_pub_l, image_pub_depth;
  	cv_bridge::CvImagePtr frame_l, frame_r, frame_depth;
public:

	  ImageLoader()
	    :
	 nh_("~"), 
	it_(nh_),
	frame_l(new cv_bridge::CvImage),
	frame_r(new cv_bridge::CvImage),
	frame_depth(new cv_bridge::CvImage)
  {
		subTime = nh_.subscribe<std_msgs::UInt64>("/roi_time", 1, &ImageLoader::timeCB, this);
		image_pub_l = it_.advertise("/image_roi_left", 1);
		image_pub_r = it_.advertise("/image_roi_right", 1);
		image_pub_depth = it_.advertise("/image_roi_depth", 1);
		timer = nh_.createTimer(ros::Duration(1.0f/10.0f), &ImageLoader::callback, this); // 80 Hz pub rate
		pubCloud = nh_.advertise<sensor_msgs::PointCloud2>("/pc_cloud_registered", 1);
		
  }
		
	void timeCB(const std_msgs::UInt64::ConstPtr& msg)
	{
		time_stamp = msg->data;
	}	
	void callback(const ros::TimerEvent&)
	{
		bool op_control_ = false;
		nh_.getParam("/zed_stereo_node/op_control", op_control_);
		if (op_control_)
		{	
			
			char image_path[100];
			int frame_cap_time = time_stamp;

			sprintf(image_path, "%d_right.png", frame_cap_time);
			frame_r->encoding = "bgr8";
		
			 frame_r->image = imread(image_path, CV_LOAD_IMAGE_COLOR);
			
			if(frame_r)
			{	
				 if(frame_r->image.empty())
				 {
				 	std::cout << "Could not read the image: " << image_path << std::endl;
				 }
				else
				{

					
					image_pub_r.publish(frame_r->toImageMsg());
				}
			}
			else
			{
			 	std::cout << "Could not read the image: " << image_path << std::endl;
			}

		}
	}
};


int main(int argc, char** argv)
{
  ros::init( argc, argv, "image_buffer" );

  ImageLoader ImageLoader;

  while( ros::ok() )
  {
    ros::spin();
  }

}
