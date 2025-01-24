
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/UInt32.h>

#include <std_msgs/UInt64.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/common_headers.h>
#include <pcl/common/angles.h> // deg2rad, rad2deg, normAngle (normalize angle to (-pi, pi))

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
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
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

using namespace cv;
class ZED_PCL
{
	
	public:
		typedef pcl::PointXYZRGBNormal PointXYZRGBNormal;
		typedef pcl::PointCloud<PointXYZRGBNormal> PointXYZRGBNormalCloud;
		typedef pcl::Normal PointNormal;
		typedef pcl::PointCloud<PointNormal> PointNormalCloud;
		typedef pcl::PointXYZRGB PointXYZRGB;
		typedef pcl::PointCloud<PointXYZRGB> PointXYZRGBCloud;
		typedef pcl::PointXYZRGBL PointXYZRGBL;
		typedef pcl::PointCloud<PointXYZRGBL> PointXYZRGBLCloud;
		typedef pcl::KdTreeFLANN<PointXYZRGB> KdTree;
		
		ros::NodeHandle nh_;
		PointXYZRGB click_point_pos;
		ros::Subscriber subPC, sub_point_click;
		bool PCLAvailable = false;
  		std::string cloud_topic = "/zed/zed_nodelet/mapping/fused_cloud";
		ros::Publisher pub;
KdTree::Ptr m_kdtree;
		float radius = 2.0f;
		float mean_k = 100.0f;
		float std_thresh = 1.0f;
		float dist_thresh = 2.0f;
	public:
	ZED_PCL()
	{
		nh_.getParam("cloud_topic", cloud_topic);
		nh_.getParam("radius", radius);
		subPC = nh_.subscribe(cloud_topic, 10, &ZED_PCL::PointCloudCB, this);
		pub = nh_.advertise<sensor_msgs::PointCloud2>("/model_cloud", 1);
		sub_point_click = nh_.subscribe("clicked_point", 1000, &ZED_PCL::point_click, this);
		m_kdtree = KdTree::Ptr(new KdTree);
		
	}

	void point_click(const geometry_msgs::PointStamped::ConstPtr& pose)
	{
		
		nh_.getParam("radius", radius);
		click_point_pos.x = pose->point.x;
		click_point_pos.y = pose->point.y;
		click_point_pos.z = pose->point.z;
		ROS_INFO("click point = %.2f, %.2f, %.2f\n", click_point_pos.x, click_point_pos.y, click_point_pos.z);
		
	}
	void PointCloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
		
		sensor_msgs::PointCloud2 cloud_in;
		cloud_in = *cloud_msg;

		nh_.getParam("dist_thresh", dist_thresh);
		nh_.getParam("radius", radius);
		nh_.getParam("mean_k", mean_k);
		nh_.getParam("std_thresh", std_thresh);
		pcl::PCLPointCloud2 pass_cloud;
		pcl_conversions::toPCL(*cloud_msg, pass_cloud);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);


		pcl::fromPCLPointCloud2(pass_cloud,*temp_cloud);
		pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);
		std::vector<int> idxRadius;
		std::vector<float> dstsRadius;
		m_kdtree->setInputCloud(temp_cloud);
		int b = m_kdtree->radiusSearch(click_point_pos, radius, idxRadius, dstsRadius);
		if (b >= 0)
		{
		
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
			inliers2->indices = idxRadius;
			extract.setInputCloud(temp_cloud);
			extract.setIndices(inliers2);
			extract.setNegative(false);
			extract.filter(*temp_cloud);
		}
	  	std::cerr << "Cloud before filtering: " << std::endl;

	  	std::cerr << *temp_cloud << std::endl;
  		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		 sor.setInputCloud (temp_cloud);

		  sor.setMeanK (mean_k);

		  sor.setStddevMulThresh (std_thresh);

		  sor.filter (*temp_cloud);


		  std::cerr << "Cloud after filtering: " << std::endl;

		  std::cerr << *temp_cloud << std::endl;
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		  // Create the segmentation object
		  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		  // Optional
		  seg.setOptimizeCoefficients (true);
  		seg.setOptimizeCoefficients (true);
		  // Mandatory
		  seg.setModelType (pcl::SACMODEL_PLANE);
		  seg.setMethodType (pcl::SAC_RANSAC);
		  seg.setDistanceThreshold (dist_thresh);
		
		  seg.setInputCloud (temp_cloud);
		  seg.segment (*inliers, *coefficients);
		pcl::ExtractIndices<pcl::PointXYZRGB> extract_sac;
   		 extract_sac.setInputCloud (temp_cloud);
           	 extract_sac.setIndices (inliers);
            	extract_sac.setNegative (false); // extract the inliers in consensus model (the part to be removed from point cloud)
            	extract_sac.filter (*temp_cloud);
		  std::cerr << "Cloud after planer segment: " << std::endl;

		  std::cerr << *temp_cloud << std::endl;
			sor.setInputCloud (temp_cloud);

		  sor.setMeanK (mean_k);

		  sor.setStddevMulThresh (std_thresh);

		  sor.filter (*temp_cloud);


		  std::cerr << "Cloud after filtering: " << std::endl;

		  std::cerr << *temp_cloud << std::endl;
		
		  //pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;

		  //sor.setInputCloud (temp_cloud);

		  //sor.setMeanK(mean_k);

		  //sor.setStddevMulThresh(std_thresh);

		  //sor.filter(*temp_cloud);
		pub.publish(temp_cloud);
		ROS_INFO("Publishing the model cloud!\n");
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
