
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


#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

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
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud;
		ros::NodeHandle nh_;
		ros::Subscriber subPC;
		bool PCLAvailable = false;
  		std::string cloud_topic = "/zed/zed_nodelet/mapping/fused_cloud";
		ros::Publisher pub;
KdTree::Ptr m_kdtree;
		float radius_ = 5.0f;
		float tf_eps_ = 1e-12;
		float fit_eps_ = 0.1f;
		int max_iters_ = 100;
		std::string model_filename_ = "model_roomba.pcd";
	public:
	ZED_PCL()
:
model_cloud (new pcl::PointCloud<pcl::PointXYZRGB>)
	{
		nh_.getParam("model_filename", model_filename_);
		nh_.getParam("cloud_topic", cloud_topic);
		subPC = nh_.subscribe(cloud_topic, 10, &ZED_PCL::PointCloudCB, this);
		pub = nh_.advertise<sensor_msgs::PointCloud2>("/model_cloud", 1);
		//pub2 = nh_.advertise<sensor_msgs::PointCloud2>("/scene_cloud", 1);
		if (pcl::io::loadPCDFile (model_filename_, *model_cloud) < 0)
		  {
		    std::cout << "Error loading model cloud." << std::endl;
		  }
		
	}

	void PointCloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
		nh_.getParam("radius", radius_);
		nh_.getParam("max_iters", max_iters_);
		nh_.getParam("tf_eps", tf_eps_);
		nh_.getParam("fit_eps", fit_eps_);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB>);
        	sensor_msgs::PointCloud2 cloud_out;
		pcl::PCLPointCloud2 pass_cloud;
		pcl_conversions::toPCL(*cloud_msg, pass_cloud);
		pcl::fromPCLPointCloud2(pass_cloud, *scene_cloud);
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
		icp.setInputSource(model_cloud);
		icp.setInputTarget(scene_cloud);

		icp.setMaxCorrespondenceDistance(radius_);
		icp.setMaximumIterations(max_iters_);
		icp.setTransformationEpsilon (tf_eps_);
		icp.setEuclideanFitnessEpsilon(fit_eps_);
		icp.align(*cloud_aligned);
		//pcl::toPCLPointCloud2(cloud_aligned, pass_cloud);
		pcl::toROSMsg(*cloud_aligned, cloud_out);
		cloud_out.header.frame_id = "map";
		pub.publish(cloud_out);

	}
	
};

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "pcl_icp");
	ZED_PCL ZPC;

	while( ros::ok() )
	{
		ros::spin();
	}
  return 0;
}
