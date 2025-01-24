
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include "std_msgs/Float32.h"

class pc_detector
{
	
	public:
		ros::NodeHandle nh_;
		ros::Subscriber sub;
				
		ros::Publisher pub;
		ros::Publisher vis_pub;
		ros::Publisher lidar_pub;
		std_msgs::Float32 lidar_msg;
		double dist_threshold = 10.0;
		int min_cluster_size = 500;
		int region_color_threshold = 6;
		int point_color_threshold = 5;
	public:
	pc_detector()
	{
		  ROS_INFO("Starting detector....\n"); 
		  // Create a ROS subscriber for the input point cloud
		  // Set up to switch between full data and filtered 
		  sub = nh_.subscribe("filtered_cloud", 1, &pc_detector::cloud_cb, this);

		  // Create a ROS publisher for the output point cloud
		  pub = nh_.advertise<sensor_msgs::PointCloud2>("pc_object", 1);

		  // Create a ROS publisher for the marker output
		  vis_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);

		  // Create a ROS publisher for the sign distance
		  lidar_pub = nh_.advertise<std_msgs::Float32>("sd_distance", 1000);

		
	}
	


	void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
	{
		
		nh_.getParam("/zed_stereo_node/dist_threshold", dist_threshold);
		nh_.getParam("/zed_stereo_node/region_color_threshold", region_color_threshold);
		nh_.getParam("/zed_stereo_node/min_cluster_size", min_cluster_size);
		nh_.getParam("/zed_stereo_node/point_color_threshold", point_color_threshold);
	  // Container for original & filtered data
	  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2; // pointer to empty pointcloud2 struct cloud
	  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	  pcl::PCLPointCloud2 cloud_filtered; // defines cloud_filtered as type pointcloud2
	  //pcl::PCLPointCloud2 cloud_projected;
	  // Convert to PCL data type
	  pcl_conversions::toPCL(*cloud_msg, *cloud);

		std::cout<<"pc_roi received...\n";
	  pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_pcl (new pcl::PointCloud <pcl::PointXYZRGB>);
	   pcl::SACSegmentation<pcl::PointXYZI> seg;
	  pcl::ProjectInliers<pcl::PointXYZI> proj;
	  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzPtr(new pcl::PointCloud<pcl::PointXYZI>);
	  pcl::PointCloud<pcl::PointXYZI>::Ptr segPtr(new pcl::PointCloud<pcl::PointXYZI>);
	  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	  pcl::fromPCLPointCloud2(*cloud, *cloud_pcl);
	  pcl::fromPCLPointCloud2(*cloud, *cloud_xyzPtr);
	  pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

	  //------------------------------ PLANE FIITING ---------------------------------------//
	  //------------------------------               ---------------------------------------//
	  if (cloud->width > 0)
	  {
			
		seg.setOptimizeCoefficients(true);
		// Mandatory
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.11);

		seg.setInputCloud(cloud_xyzPtr);
		seg.segment(*inliers, *coefficients);

		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setIndices(inliers);
		proj.setInputCloud(cloud_xyzPtr);
		proj.setModelCoefficients(coefficients);
		proj.filter(*segPtr);
		
	  std::cout<<"indices\n";
		pcl::IndicesPtr indices (new std::vector <int>);
		pcl::removeNaNFromPointCloud (*cloud_pcl, *indices);
		pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
		// Mandatory
		if(cloud_pcl->points.size() > 0)
		{
			reg.setInputCloud(cloud_pcl);
			
			reg.setIndices(indices);
			reg.setSearchMethod(tree);
			reg.setDistanceThreshold(dist_threshold);
			reg.setPointColorThreshold(point_color_threshold);
			reg.setRegionColorThreshold (region_color_threshold);
			reg.setMinClusterSize (min_cluster_size);
			std::vector <pcl::PointIndices> clusters;
			reg.extract (clusters);
			pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
		//------------------------------               ---------------------------------------//
		//------------------------------------------------------------------------------------//
		pcl::toPCLPointCloud2(*colored_cloud, cloud_filtered);
		}
	  
	  if (segPtr->points.size() > 10)
	  {

		// Compute average intensity
		float avg = 0.0;
		for (size_t i = 0; i < segPtr->points.size(); ++i)
		{
		  avg = avg + segPtr->points[i].intensity;
		}
		avg = avg / segPtr->points.size();

		// Final Check for Sign
		if (avg > 80 && coefficients->values[0] > 0.90)
		{ //IF intensity average and Normal vector meet specifications, convert segPtr to cloud_filtered and output

		  //Convert Data and publish
		  pcl::toPCLPointCloud2(*segPtr, cloud_filtered);
		  ROS_INFO("SIGN DETECTED, X: %f", segPtr->points[0].x); // Printout X location 
		  lidar_msg.data = segPtr->points[0].x;
			
		  // Visualization marker stuff
		  visualization_msgs::Marker marker;
		  marker.header.frame_id = "map";
		  marker.header.stamp = ros::Time();
		  marker.ns = "my_namespace";
		  marker.id = 0;
		  marker.type = visualization_msgs::Marker::SPHERE;
		  marker.action = visualization_msgs::Marker::ADD;
		  marker.pose.position.x = segPtr->points[0].x;
		  marker.pose.position.y = segPtr->points[0].y;
		  marker.pose.position.z = segPtr->points[0].z;
		  marker.pose.orientation.x = 0.0;
		  marker.pose.orientation.y = 0.0;
		  marker.pose.orientation.z = 0.0;
		  marker.pose.orientation.w = 1.0;
		  marker.scale.x = 0.5;
		  marker.scale.y = 2.5;
		  marker.scale.z = 2.5;
		  marker.color.a = 1.0; // Don't forget to set the alpha!
		  marker.color.r = 1.0;
		  marker.color.g = 0.0;
		  marker.color.b = 0.0;
		  //only if using a MESH_RESOURCE marker type:
		  vis_pub.publish(marker);
		  lidar_pub.publish(lidar_msg);
		}
	  }

	  // Convert to ROS data type & Publish
	  sensor_msgs::PointCloud2 output;
	  pcl_conversions::fromPCL(cloud_filtered, output);
	  output.header.frame_id = "map";
	  pub.publish(output);
	  }
	}
};

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "sign_detection");
	pc_detector detector;

	while( ros::ok() )
	{
		ros::spin();
	}
  return 0;
}
