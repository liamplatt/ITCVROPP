

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
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
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

#include <tf/transform_listener.h>
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
#include <tf/transform_listener.h>
#include "Eigen/Geometry"




using namespace pcl;
using namespace std;



using namespace cv;
class pc_buffer
{
	// Point cloud buffer object to simply hold point cloud data sent at a freq. of 0.4Hz and save to a pcd file.
	// PCD data file is loaded into memory in op control mode and published at rate of 80Hz
	
	public:
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		ros::Subscriber subPC1, subPC2;
		//ros::Publisher pcl_pubL, pcl_pubR;
		sensor_msgs::PointCloud2 out_cloud;
		//For data projection
		ros::Timer timer;
		ros::Publisher pub;
		tf::TransformListener listener;
		double voxel_size_ = 0.05;
		double map_res_= 0.1;
		bool op_control_ = false;
		bool file_loaded_ = false;
		double old_voxel_size_ = 0.05;
		int point_cloud_fps = 60;
		ros::Time time_stamp;
		int point_cloud_fps_ = 60;
		int thresh_ = 5;
		std::vector<ros::Time> time_vector;
		std::string point_cloud_topic = "/cloud_in";
		std::string registered_point_cloud_topic = "/zed/zed_nodelet/point_cloud/cloud_registered";
		std::string load_custom_pcd = "~/temp_cloud.pcd";
	public:
	pc_buffer()
	:
	nh_("~"),
	it_(nh_)
	{
		
		nh_.getParam("point_cloud_topic", point_cloud_topic);
		nh_.getParam("registered_point_cloud_topic", registered_point_cloud_topic);
		subPC1 = nh_.subscribe(point_cloud_topic, 10, &pc_buffer::PointCloudCB, this);
		subPC2 = nh_.subscribe(registered_point_cloud_topic, 10, &pc_buffer::PointCloudCBReg, this);
		timer = nh_.createTimer(ros::Duration(1.0f/point_cloud_fps), &pc_buffer::pub_callback, this); // 80 Hz pub rate
		pub = nh_.advertise<sensor_msgs::PointCloud2>("/pc_buffer", 1);
		
		
		
	}
	void pub_callback(const ros::TimerEvent&)
	{
		
		nh_.getParam("/zed_stereo_node/op_control", op_control_);
		nh_.getParam("/zed_stereo_node/load_custom_pcd", load_custom_pcd);
		
		if(!file_loaded_ && op_control_)
		{
			ROS_INFO("Loading pcd data file...\n");
			
			// Read in latest cloud if in op control mode
			pcl::io::loadPCDFile (load_custom_pcd, out_cloud);
			sensor_msgs::PointCloud2 cloud;	
			file_loaded_ = true;
		}
		// Convert to ROS data type
		if(op_control_)
		{
			out_cloud.header.frame_id = "map";
			out_cloud.header.stamp = time_stamp;
			pub.publish(out_cloud);

		}
		else
		{
			file_loaded_ = false;
		}
		nh_.getParam("/zed_stereo_node/pointCloudFPS", point_cloud_fps_);
		if(point_cloud_fps_ != point_cloud_fps)
		{
			ROS_INFO("New point cloud FPS: %d.\n", point_cloud_fps_);
			point_cloud_fps = point_cloud_fps_;
			timer.stop();
			timer.setPeriod(ros::Duration(1.0f/point_cloud_fps), true);
			timer.start();
		}
	}
	
	
	
	void PointCloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
	
		nh_.getParam("/zed_stereo_node/load_custom_pcd", load_custom_pcd);
		nh_.getParam("/zed_stereo_node/voxel_size", voxel_size_);

		nh_.getParam("/zed_stereo_node/op_control", op_control_);
		
		// Container for original & filtered data
		//out_cloud = *input; //Old versio: simple passthrough
		if(!op_control_ || voxel_size_ != old_voxel_size_)
		{
			try
			{
				pcl::PCLPointCloud2 carrier_cloud;
				sensor_msgs::PointCloud2 carrier;

				pcl::PCLPointCloud2* pass_cloud = new pcl::PCLPointCloud2;
				pcl::PCLPointCloud2ConstPtr cloudPtr2(pass_cloud);
				
				pcl_conversions::toPCL(*cloud_msg, *pass_cloud); // Convert to PCL data type
				

				pcl::VoxelGrid<pcl::PCLPointCloud2> v_filter;
				 // Perform the actual filtering
				v_filter.setInputCloud(cloudPtr2); // Pass raw_cloud to the filter
				v_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_); // Set leaf size
				v_filter.filter(carrier_cloud); // Store output data in first_cloud
			   
				pcl::PointCloud<pcl::PointXYZRGB> point_cloud;;
				pcl::fromPCLPointCloud2( carrier_cloud, point_cloud);
				pcl::io::savePCDFileASCII (load_custom_pcd, point_cloud);
			}
			catch (...)
			{
			ROS_INFO("Point cloud is empty!");
			}
			
		}
		old_voxel_size_ = voxel_size_;
		// listen for tf for right and left camera frames
		//pcl_ros::transformPointCloud(targetL, in, cloudLtemp, listener);
		//pcl_ros::transformPointCloud(targetR, in, cloudRtemp, listener);
		
		//pcl_ros::transformPointCloud(target_hmd, cloudLtemp, cloudL, listener);
		//pcl_ros::transformPointCloud(target_hmd, cloudLtemp, cloudR, listener);
		//pcl::fromROSMsg(cloudR, ros_cloudR);
		//pcl::fromROSMsg(cloudL, ros_cloudL);
		//PCLAvailable = true;

	}
	
	void PointCloudCBReg(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
		nh_.getParam("/zed_stereo_node/op_control", op_control_);
		time_stamp = cloud_msg->header.stamp;
	}
	
};

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "pcl_reader");
	pc_buffer buffer;

	while( ros::ok() )
	{
		ros::spin();
	}
  return 0;
}
