#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include <wallie_one/ZedPubStereoConfig.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/Point.h>

#include <image_geometry/pinhole_camera_model.h>

#include <camera_info_manager/camera_info_manager.h>

#include <image_transport/subscriber_filter.h>
#include <image_transport/image_transport.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/hal/interface.h>

 #include <rosbag/bag.h>
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

class ImageBuffer
{

public:
  ros::NodeHandle nh_;
	ros::Subscriber subPC, subDepth; 
	 rosbag::Bag bag;
	tf::TransformListener listener;
  	image_transport::ImageTransport it_;
 	std::string topic_name;
  	std::string topic_name_right = "/zed/zed_nodelet/right/image_rect_color_throttled";
 	std::string topic_name_left = "/zed/zed_nodelet/left/image_rect_color_throttled";
	
	image_transport::SubscriberFilter subRightRectified;
	image_transport::SubscriberFilter subLeftRectified;
		std::string registered_point_cloud_topic = "/zed/zed_nodelet/point_cloud/cloud_registered";
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
 	cv::Mat image_l, image_r, image_depth;
	Eigen::Matrix4f camera_pose;
  	cv_bridge::CvImagePtr frame_l, frame_r, frame_depth;
	message_filters::Synchronizer< MySyncPolicy > sync;
			pcl::PointCloud<pcl::PointXYZRGB> point_cloud;

public:

  ImageBuffer()
    :
 nh_("~"), 
it_(nh_),
subRightRectified(it_, "/zed/zed_nodelet/right/image_rect_color_throttled", 1),
subLeftRectified(it_, "/zed/zed_nodelet/left/image_rect_color_throttled", 1),
sync(MySyncPolicy(10), subLeftRectified, subRightRectified)
  {
	//sub_ = nh_.subscribe(topic_name_default, 1, &ImageBuffer::callback, this);
		nh_.getParam("registered_point_cloud_topic", registered_point_cloud_topic);
		std::string stereo_namespace = "/zed/zed_nodelet";
		nh_.getParam("stereo_namespace", stereo_namespace);
		//subDepth = nh_.subscribe("/zed/zed_nodelet/depth/depth_registered_throttled", 1, &ImageBuffer::callback, this);
		//subPC = nh_.subscribe(registered_point_cloud_topic, 10, &ImageBuffer::PointCloudCBReg, this);
		ROS_INFO("Registering CB Stereo.....\n");
		sync.registerCallback(boost::bind(&ImageBuffer::imageStereoRectifiedCallback, this, _1, _2));
		subRightRectified.unsubscribe();
		subLeftRectified.unsubscribe();
	
		subLeftRectified.subscribe(it_, stereo_namespace + "/left/image_rect_color_throttled", 1);
	
		subRightRectified.subscribe(it_, stereo_namespace + "/right/image_rect_color_throttled", 1);

  }



~ImageBuffer()
{
bag.close();
}
	void imageStereoRectifiedCallback(const sensor_msgs::Image::ConstPtr&  msgL, const sensor_msgs::Image::ConstPtr&  msgR)
	{
	try
	{
		frame_r = cv_bridge::toCvCopy(msgR, sensor_msgs::image_encodings::RGB8);
		image_r = frame_r->image;
		if(image_r.empty())
		{
		  std::cerr << "Image buffer is empty" << std::endl;
		}
		else
		{
			// Save the frame into a file
			char aa[100];
			int frame_cap_time = int(std::round(msgL->header.stamp.toSec()));

			//bag.write("right/image_rect_color", msgR->header.stamp, *msgR);
			sprintf(aa, "%d_right.png", frame_cap_time);
			imwrite(aa, image_r); // A JPG FILE IS BEING SAVED
			  std::cout << "Image buffer saved to " << aa << std::endl;
		}
		
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR_STREAM("Failed to take first image: " << e.what());
		return;
	}
	/*if(point_cloud.width*point_cloud.height){
	int frame_cap_time = int(std::round(msgL->header.stamp.toSec()));
	char aa[100];
	sprintf(aa, "%d.pcd", frame_cap_time);
	pcl::io::savePCDFileASCII (aa, point_cloud);
}*/
}

	void PointCloudCBReg(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
			
	}


};


int main(int argc, char** argv)
{
  ros::init( argc, argv, "image_buffer" );

  ImageBuffer ImageBuffer;

  while( ros::ok() )
  {
    ros::spin();
  }

}
