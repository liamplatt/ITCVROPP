
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <zed_interfaces/ObjectsStamped.h>
#include <zed_interfaces/BoundingBox2Di.h>
#include <image_transport/image_transport.h>
//#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/hal/interface.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>

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
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

using namespace cv;
class ZED_PCL
{
	
	public:
		int h = 720;
		int w = 1280;
		int pc_data_radius = 5;
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
	    cv_bridge::CvImagePtr left, right;
		
		//Subscribe to left and right camera images from zed_node
		Eigen::Matrix4f Tm;
	    image_transport::Publisher image_pubL, image_pubR;
		ros::Subscriber subL, subR;
		//image_geometry::PinholeCameraModel modelL, modelR;
		//message_filters::Subscriber<sensor_msgs::Image> image_sub_L, image_sub_R;
		geometry_msgs::TransformStamped tf_stamped;
		//message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_L, info_sub_R;
		
		//message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> sync_L, sync_R;
		geometry_msgs::Quaternion Quat;
		tf::TransformListener listener;
		ros::Subscriber subPC, subQ, subCamL, subCamR;
		//ros::Publisher pcl_pubL, pcl_pubR;
		sensor_msgs::PointCloud2 cloudL, cloudR, cloudLtemp, cloudRtemp;
		//For data projection
		pcl::PointCloud<pcl::PointXYZRGB> ros_cloudR, ros_cloudL;
		std::string targetR = "zed2_right_camera_optical_frame";
		std::string targetL = "zed2_left_camera_optical_frame";
		std::string image_topic_L = "/blank_image_l";
		std::string image_topic_R = "/blank_image_r";
		
		std::string info_topic_L = "/zed_node/left/camera_info";
		std::string info_topic_R = "/zed_node/right/camera_info";
		std::string target_hmd = "hmd_imu_frame";
		bool LAvailable = false;
		bool RAvailable = false;
		double Tx_l, Tx_r, cx_l, cx_r, fx_l, fx_r, fy_l, fy_r, Ty_l, Ty_r, cy_l, cy_r;
		
	public:
	ZED_PCL()
	:
	it_(nh_)
	{
		
		ROS_INFO("Advertising left_image_PC and right_image_PC\n");
		image_pubL = it_.advertise("pcl_reader/left_image_PC", 1);
		image_pubR = it_.advertise("pcl_reader/right_image_PC", 1);
		ROS_INFO("Subscribing to /rtabmap/cloud_map\n");
		subPC = nh_.subscribe("/zed_node/point_cloud/cloud_registered", 10, &ZED_PCL::PointCloudCB, this);
		subL = nh_.subscribe(image_topic_L, 1, &ZED_PCL::imageCbL, this);
		subR = nh_.subscribe(image_topic_R, 1, &ZED_PCL::imageCbR, this);
		subCamL = nh_.subscribe(info_topic_L, 1, &ZED_PCL::infoCbL, this);
		subCamR = nh_.subscribe(info_topic_R, 1, &ZED_PCL::infoCbR, this);
	}
	void PointCloudCB(const sensor_msgs::PointCloud2 &in)
	{
		ROS_INFO("POintcloud callback\n");
		// listen for tf for right and left camera frames
		
		pcl_ros::transformPointCloud(targetL, in, cloudLtemp, listener);
		pcl_ros::transformPointCloud(targetR, in, cloudRtemp, listener);
		
		pcl_ros::transformPointCloud(target_hmd, cloudLtemp, cloudL, listener);
		pcl_ros::transformPointCloud(target_hmd, cloudLtemp, cloudR, listener);
		pcl::fromROSMsg(cloudR, ros_cloudR);
		pcl::fromROSMsg(cloudL, ros_cloudL);
		
		if(LAvailable)
		{	
			for(size_t i = 0; i < ros_cloudL.points.size(); i++)
			{
				// Create 3d point from current iter point
				Point3d point3 = Point3d(ros_cloudL.points[i].x, ros_cloudL.points[i].y, ros_cloudL.points[i].z);
				int r = ros_cloudL.points[i].r; int g = ros_cloudL.points[i].g; int b = ros_cloudL.points[i].b;
				Point2d point2;
				point2.x = (fx_l*point3.x + Tx_l) / point3.z + cx_l;
				point2.y = (fy_l*point3.y + Ty_l) / point3.z + cy_l;
				
				circle(left->image  , cv::Point(point2.x, point2.y), pc_data_radius, CV_RGB(r,g,b));
			}
			
		image_pubL.publish(right->toImageMsg());
		}
		if(RAvailable)
		{	
			for(size_t i = 0; i < ros_cloudR.points.size(); i++)
			{
				Point3d point3 = cv::Point3d(ros_cloudR.points[i].x, ros_cloudR.points[i].y, ros_cloudR.points[i].z);
				int r = ros_cloudR.points[i].r; int g = ros_cloudR.points[i].g; int b = ros_cloudR.points[i].b;
				Point2d point2;
				//Project to image
				point2.x = (fx_r*point3.x + Tx_r) / point3.z + cx_r;
				point2.y = (fy_r*point3.y + Ty_r) / point3.z + cy_r;
				circle(right->image , Point(point2.x, point2.y), pc_data_radius, CV_RGB(r,g,b));
			}
			image_pubR.publish(left->toImageMsg());
		}
		
		
	}
	void infoCbL(const sensor_msgs::CameraInfoConstPtr& info_msg)
	{
		fx_l = info_msg->K[0];
		fy_l = info_msg->K[4];
		
		Tx_l = info_msg->P[3];
		Ty_l = info_msg->P[7];
		
		cx_l = info_msg->K[2];
		cy_l = info_msg->K[5];
	}
	void infoCbR(const sensor_msgs::CameraInfoConstPtr& info_msg)
	{
		fx_r = info_msg->K[0];
		fy_r = info_msg->K[4];
		
		Tx_r = info_msg->P[3];
		Ty_r = info_msg->P[7];
		
		cx_r = info_msg->K[2];
		cy_r = info_msg->K[5];
	}
	
	//Project to left image
	void imageCbL(const sensor_msgs::CompressedImageConstPtr& image_msg)
	{
		ROS_INFO("Image L callback\n");
		//modelL.fromCameraInfo(info_msg);
		try
		{
			Mat l = imdecode(cv::Mat(image_msg->data),1);
			left = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
			std::printf("W: %d, H: %d\n",left->image.cols, left->image.rows);
		}
		
		catch (cv_bridge::Exception& e)
	    {	
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
	    }
		LAvailable = true;
	}
	//Project to right image
	void imageCbR(const sensor_msgs::CompressedImageConstPtr& image_msg)
	{
		ROS_INFO("Image R callback\n");
		//modelR.fromCameraInfo(info_msg);
		try
		{
			Mat r = imdecode(cv::Mat(image_msg->data),1);
			right = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
			std::printf("W: %d, H: %d\n",right->image.cols, right->image.rows);
		}
		catch (cv_bridge::Exception& e)
	    {	
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
	    }
		
		RAvailable = true;
	}
	
	
};

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "pcl_reader");
	ZED_PCL ZPC;

	while( ros::ok() )
	{
		ros::spin();
	}
  return 0;
}
