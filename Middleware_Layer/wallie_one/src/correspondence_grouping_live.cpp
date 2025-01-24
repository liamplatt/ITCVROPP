#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
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

#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

typedef pcl::PointXYZRGBA PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;


//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool use_cloud_resolution_ (false);
bool use_hough_ (true);
float model_ss_ (0.01f);
float scene_ss_ (0.03f);
float rf_rad_ (0.015f);
float descr_rad_ (0.02f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);

double

class corr_grouper
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
				
		std::string model_filename_;
		std::string scene_filename_;
		ros::NodeHandle nh_;
		PointXYZRGB click_point_pos;
		ros::Subscriber subPC, sub_point_click;
		bool PCLAvailable = false;
  		std::string cloud_topic = "/zed/zed_nodelet/mapping/fused_cloud";
  		std::string model_filename = "~/catkin_ws/model.pcd";
		ros::Publisher pub;
		KdTree::Ptr m_kdtree;
		float radius = 2.0f;
		float mean_k = 100.0f;
		float std_thresh = 2.0f;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr model;
	public:
	corr_grouper()
:
model(new pcl::PointCloud<pcl::PointXYZRGB>)
	{
		nh_.getParam("cloud_topic", cloud_topic);
		nh_.getParam("model_filename", model_filename_);
		 if (pcl::io::loadPCDFile (model_filename_, *model) < 0)
		  {
		    std::cout << "Error loading model cloud." << std::endl;
		  }
		subPC = nh_.subscribe(cloud_topic, 10, &corr_grouper::PointCloudCB, this);
		m_kdtree = KdTree::Ptr(new KdTree);
		
	}

	void PointCloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
		 pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
		 pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
		 pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
		 pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
		 pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
		 pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
		 pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

		pcl::PCLPointCloud2 pass_cloud;
		pcl_conversions::toPCL(*cloud_msg, pass_cloud);
		pcl::fromPCLPointCloud2(pass_cloud, *model);

		float resolution = static_cast<float> (computeCloudResolution());
		if (resolution != 0.0f)
		{
			model_ss_   *= resolution;
			scene_ss_   *= resolution;
			rf_rad_     *= resolution;
			descr_rad_  *= resolution;
			cg_size_    *= resolution;
		}

		std::cout << "Model resolution:       " << resolution << std::endl;
		std::cout << "Model sampling size:    " << model_ss_ << std::endl;
		std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
		std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
		std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
		std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;


		ROS_INFO("Publishing the model cloud!\n");
	}

float computeCloudResolution()
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<PointType>::Ptr tree;
  tree.setInputCloud(model);

  for (std::size_t i = 0; i < model->size(); ++i)
  {
    if (! std::isfinite ((*model)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}
	
};

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "pcl_reader");
	corr_grouper ZPC;

	while( ros::ok() )
	{
		ros::spin();
	}
  return 0;
}

