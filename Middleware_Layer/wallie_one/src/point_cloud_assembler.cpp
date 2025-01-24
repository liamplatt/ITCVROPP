
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
class PointCloud2Assembler
{
	
	public:
		
	  //! \brief Stores history of scans
	  std::deque<pcl::PointXYZRGB> scan_hist_;
	  boost::mutex scan_hist_mutex_;

	  //! \brief The number points currently in the scan history
	  unsigned int total_pts_;

	  //! \brief The max number of scans to store in the scan history
	  unsigned int max_scans_;

	  //! \brief The frame to transform data into upon receipt
	  std::string fixed_frame_;

	  //! \brief The frame to transform data into upon receipt
	  std::string cloud_topic_ = "/zed/zed_nodelet/point_cloud/cloud_registered";

	  //! \brief Specify how much to downsample the data. A value of 1 preserves all the data. 3 would keep 1/3 of the data.
	  unsigned int downsample_factor_;

	PointCloud2Assembler()
	:
	{
		nh_.getParam("fixed_frame", fixed_frame_);
		nh_.getParam("downsample_factor", downsample_factor_);
		nh_.getParam("max_scans", max_scans_);
		nh_.getParam("cloud_topic", cloud_topic_);
		subPC = nh_.subscribe(cloud_topic_, 10, &PointCloud2Assembler::PointCloudCB, this);
		
	}

	void PointCloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr curr_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl_ros::transformPointCloud(fixed_frame_, *cloud_msg, curr_cloud, listener); 
		scan_hist_mutex_.lock() ;
	  	if (scan_hist_.size() == max_scans_)                           // Is our deque full?
		{
		    total_pts_ -= scan_hist_.front()->points.size () ;            // We're removing an elem, so this reduces our total point count
		    scan_hist_.pop_front() ;                                     // The front of the deque has the oldest elem, so we can get rid of it
		}
		scan_hist_.push_back(curr_cloud) ;                              // Add the newest scan to the back of the deque
		total_pts_ += curr_cloud->points.size() ;                       // Add the new scan to the running total of points

		std::printf("Scans: %4u  Points: %10u\n", scan_hist_.size(), total_pts_);

  		scan_hist_mutex_.unlock() ;
	}
	void findPointClouds()
	{
	 scan_hist_mutex_.lock() ;
	  // Determine where in our history we actually are
	  unsigned int i = 0 ;

	  // Find the beginning of the request. Probably should be a search
	  while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
		  scan_hist_[i].header.stamp < req.begin )                                    // Keep stepping until we've exceeded the start time
	  {
	    i++ ;
	  }
	  unsigned int start_index = i ;

	  unsigned int req_pts = 0 ;                                                          // Keep a total of the points in the current request
	  // Find the end of the request
	  while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
		  scan_hist_[i].header.stamp < req.end )                                      // Don't go past the end-time of the request
	  {
	    req_pts += (scan_hist_[i].points.size ()+downsample_factor_-1)/downsample_factor_ ;
	    i += downsample_factor_ ;
	  }
	  unsigned int past_end_index = i ;

	  if (start_index == past_end_index)
	  {
	    resp.cloud.header.frame_id = fixed_frame_;
	    resp.cloud.header.stamp = req.end;
	    resp.cloud.points.resize(0);
	    resp.cloud.channels.resize(0);
	  }
	  else
	  {
	    //resp.cloud.header.stamp = req.end ;
	    resp.cloud.header.frame_id = fixed_frame_ ;
	    unsigned int cloud_count = 0 ;
	    for (i=start_index; i<past_end_index; i+=downsample_factor_)
	    {

	      // Sanity check: Each channel should be the same length as the points vector
	      for (unsigned int chan_ind = 0; chan_ind < scan_hist_[i].channels.size(); chan_ind++)
	      {
		if (scan_hist_[i].points.size () != scan_hist_[i].channels[chan_ind].values.size())
		  ROS_ERROR("Trying to add a malformed point cloud. Cloud has %u points, but channel %u has %u elems", (int)scan_hist_[i].points.size (), chan_ind, (int)scan_hist_[i].channels[chan_ind].values.size ());
	      }

	      for(unsigned int j=0; j<scan_hist_[i].points.size (); j+=downsample_factor_)
	      {
		resp.cloud.points[cloud_count].x = scan_hist_[i].points[j].x ;
		resp.cloud.points[cloud_count].y = scan_hist_[i].points[j].y ;
		resp.cloud.points[cloud_count].z = scan_hist_[i].points[j].z ;

		for (unsigned int k=0; k<num_channels; k++)
		  resp.cloud.channels[k].values[cloud_count] = scan_hist_[i].channels[k].values[j] ;

		cloud_count++ ;
	      }
	      resp.cloud.header.stamp = scan_hist_[i].header.stamp;
	    }
	  }
	  scan_hist_mutex_.unlock() ;
}
	
};

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "pcl_icp");
	PointCloud2Assembler PointCloud2Assembler;

	while( ros::ok() )
	{
		ros::spin();
	}
  return 0;
}
