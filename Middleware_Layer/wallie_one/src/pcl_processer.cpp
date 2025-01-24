

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

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <darknet_ros_msgs/ObjectCount.h>

#include <image_transport/image_transport.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

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
#include <pcl/filters/frustum_culling.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/hal/interface.h>

#include <std_msgs/UInt64.h>
#include <std_msgs/Time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>

#include <pcl/filters/crop_box.h>
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
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace cv;
using namespace pcl;
class pc_buffer
{
	// Point cloud buffer object to simply hold point cloud data sent at a freq. of 0.4Hz and save to a pcd file.
	// PCD data file is loaded into memory in op control mode and published at rate of 80Hz
	
	public:
		typedef pcl::PointXYZRGB PointXYZRGB;
		tf::TransformBroadcaster br;
		typedef pcl::KdTreeFLANN<PointXYZRGB> KdTree;
		sensor_msgs::Image image;
		ros::NodeHandle nh_;
		tf::TransformListener listener;
		image_geometry::PinholeCameraModel cam_model_;
		image_transport::ImageTransport it_;
		ros::Subscriber subPCRoi, subBounding, subROI, subCam, subPCFused,subPoint,subIdx, subTime;
		ros::Publisher pubSegment_radius, pubSegment_color,pubSegment, pubGoal;
		sensor_msgs::PointCloud2 out_cloud;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cloud_reg; 
		PointXYZRGB click_point_pos;
		double thresh_;
		tf::StampedTransform transform_last;
		uint64 click_point_idx = 0;
		bool op_control_ = true;
		KdTree::Ptr m_kdtree;
		bool cloud_av = false;
		int time_stamp = 0;
		std::vector<cv::Rect> pixel;
		std::vector<cv::Rect> world_min, world_max;
		std::vector<std::string> classes;
		sensor_msgs::PointCloud2 cloud_in;
					int min_cluster_size = 500;
					int point_color_thresh = 6;
					int region_color_thresh = 5;
		double zmax = 10;
		tf::Quaternion quat_tf;
		tf::Matrix3x3 Rotation;
		Eigen::Matrix4f camera_pose;
		double zmin = 0.2;
		double expand_ratio = 1.0;
		double Tx_l, Tx_r, cx_l, cx_r, fx_l, fx_r, fy_l, fy_r, Ty_l, Ty_r, cy_l, cy_r = 1;
		
		 float vertical_fov = 60;
		geometry_msgs::PoseStamped goal_point;
		 float horizontal_fov = 90;
		 float near_plane_distance = .1;
		 float far_plane_distance = 25;
		bool transform_loaded = false;
		int last_transform_time_stamp = -1;

	pc_buffer()
	:
	nh_("~"),
	it_(nh_),
	listener(ros::Duration(10000)),
	cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
	{
		std::string info_topic;

		m_kdtree = KdTree::Ptr(new KdTree);
		nh_.getParam("info_topic", info_topic);
		info_topic = info_topic + "/left/camera_info";
		pubSegment = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_cloud_object_1", 1);
		subCam = nh_.subscribe(info_topic, 1, &pc_buffer::infoCb, this);
		//subPoint = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &pc_buffer::point_click, this);
		subIdx = nh_.subscribe<std_msgs::UInt64>("/point_clicked", 1, &pc_buffer::idxCB, this);
		subBounding = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, &pc_buffer::boundingBoxCB, this);
		subTime = nh_.subscribe<std_msgs::UInt64>("/roi_time", 1, &pc_buffer::timeCB, this);
		//subPCRoi = nh_.subscribe<sensor_msgs::PointCloud2>("/pc_cloud_registered", 1, &pc_buffer::PointCloudCB, this);
		subPCFused = nh_.subscribe<sensor_msgs::PointCloud2>("/pc_buffer", 1, &pc_buffer::PointCloudCBFused, this);
		pubSegment_color = nh_.advertise<sensor_msgs::PointCloud2>("/color_cloud", 1);
		pubSegment_radius = nh_.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 1);
		pubGoal = nh_.advertise<geometry_msgs::PoseStamped>("/debug_pose", 1);
		//timer = nh_.createTimer(ros::Duration(1.0f/10), &pc_buffer::rgbVis, this); // 80 Hz pub rate
		click_point_pos.x = 0;
		click_point_pos.y = 0;
		camera_pose = Eigen::Matrix4f::Identity();
		click_point_pos.z = 0;
		
		
	}
	void timeCB(const std_msgs::UInt64::ConstPtr& msg)
	{
		
		time_stamp = msg->data;
	}
	void idxCB(const std_msgs::UInt64::ConstPtr& msg)
	{
		click_point_idx = msg->data;
	}
		
	void PointCloudCBFused(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{

		nh_.getParam("/zed_stereo_node/op_control", op_control_);
		nh_.getParam("/zed_stereo_node/thresh", thresh_);
		if(op_control_ )
		{
				
			pcl::PCLPointCloud2 pass_cloud;
			pcl_conversions::toPCL(*cloud_msg, pass_cloud);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::fromPCLPointCloud2(pass_cloud,*temp_cloud);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_segment(new pcl::PointCloud<pcl::PointXYZRGB>);
			click_point_pos.x = temp_cloud->points[click_point_idx].x;
			click_point_pos.y = temp_cloud->points[click_point_idx].y;
			click_point_pos.z = temp_cloud->points[click_point_idx].z;
			cloud_in = *cloud_msg;
			pcl::PCLPointCloud2 temp;
			pcl_conversions::toPCL(cloud_in, temp);
			pcl::fromPCLPointCloud2(temp,*cloud);
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
			std::vector<int> idxRadius;
			std::vector<float> dstsRadius;
			m_kdtree->setInputCloud(cloud);
			int b = m_kdtree->radiusSearch(click_point_pos, thresh_, idxRadius, dstsRadius);
			if (b >= 0)
			{
			
				pcl::ExtractIndices<pcl::PointXYZRGB> extract;
				inliers->indices = idxRadius;
				extract.setInputCloud(cloud);
				extract.setIndices(inliers);
				extract.setNegative(false);
				extract.filter(*cloud);
			}
			else
		geometry_msgs::PoseStamped goal_point;
			{

			}
		
			/*
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
			std::vector<int> idxRadius;
			std::vector<float> dstsRadius;
			m_kdtree->setInputCloud(temp_cloud);
			try
			{
				int b = m_kdtree->radiusSearch(click_point_pos, thresh_, idxRadius, dstsRadius);
				if (b >= 0)
				{
				
					pcl::ExtractIndices<pcl::PointXYZRGB> extract;
					inliers->indices = idxRadius;
					extract.setInputCloud(temp_cloud);
					extract.setIndices(inliers);
					extract.setNegative(false);
					extract.filter(*temp_cloud);
				}
				else
				{

				}
				pubSegment_radius.publish(temp_cloud);
				
				pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
				int min_cluster_size = 500;
				int point_color_thresh =6;
				int region_color_thresh =5;
				nh_.getParam("/zed_stereo_node/min_cluster_size", min_cluster_size);
				nh_.getParam("/zed_stereo_node/point_color_threshold", point_color_thresh);
				nh_.getParam("/zed_stereo_node/region_color_threshold", region_color_thresh);
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);
				  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
				  reg.setInputCloud (temp_cloud);
				  reg.setSearchMethod (tree);
				  reg.setDistanceThreshold (10);
				  reg.setPointColorThreshold (6);
				  reg.setRegionColorThreshold (5);
				  reg.setMinClusterSize (min_cluster_size);
				  std::vector <pcl::PointIndices> clusters;
				  reg.extract(clusters);
				if(!clusters.empty())
				{
				  	cloud_color = reg.getColoredCloud();
				
					sensor_msgs::PointCloud2 cloud_out;
				
					pcl::toROSMsg(*cloud_color, cloud_out);
					cloud_out.header.frame_id = "map";
					pubSegment_color.publish(cloud_out);
				}
				  CropBox<PointXYZRGB> cropBoxFilter;
				  cropBoxFilter.setInputCloud (cloud_segment);
				  // Cropbox slighlty bigger then bounding box of points
					
			}
			catch (...)
			{
			}
		*/
		}
	}
	void PointCloudCB(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
		geometry_msgs::PoseStamped goal_point;

		pcl::PCLPointCloud2 cloud_temp;
		
		pcl_conversions::toPCL(*cloud_msg, cloud_temp);
		
		pcl::fromPCLPointCloud2(cloud_temp,*cloud_reg);

	}
	void infoCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
	{
		fx_l = info_msg->K[0];
		fy_l = info_msg->K[4];
		
		Tx_l = info_msg->P[3];
		Ty_l = info_msg->P[7];
		
		cx_l = info_msg->K[2];
		cy_l = info_msg->K[5];
	  	cam_model_.fromCameraInfo(info_msg);
	}

	


	void boundingBoxCB(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
	{
		nh_.getParam("/zed_stereo_node/op_control", op_control_);
		if(op_control_)
		{
			pixel.clear();
			classes.clear();
			world_min.clear();
			world_max.clear();
			for(size_t i = 0; i < msg->bounding_boxes.size(); i++)
			{
				int xmin = msg->bounding_boxes[i].xmin;
				int ymin = msg->bounding_boxes[i].ymin;
				int xmax = msg->bounding_boxes[i].xmax;
				int ymax = msg->bounding_boxes[i].ymax;
	    			cv::Point2d pixel_point_min(xmin, ymin);
	    			cv::Point2d pixel_point_max(xmax, ymax);
				cv::Point3d xyz_min = cam_model_.projectPixelTo3dRay(pixel_point_min);
				cv::Point3d xyz_max = cam_model_.projectPixelTo3dRay(pixel_point_max);
				if (msg->bounding_boxes[i].probability >= .7)
				{
				cv::Rect roi(xmin, ymin, xmax, ymax);
				pixel.push_back(roi);
				classes.push_back(msg->bounding_boxes[i].Class);
				}

			}

			if((cloud->width * cloud->height))
			{
			
				nh_.getParam("/zed_stereo_node/vertical_fov", vertical_fov);
				nh_.getParam("/zed_stereo_node/horizontal_fov", horizontal_fov);
				nh_.getParam("/zed_stereo_node/near_plane_distance", near_plane_distance);
				nh_.getParam("/zed_stereo_node/far_plane_distane", far_plane_distance);
				pcl::FrustumCulling<pcl::PointXYZRGB> fc;
				
				tf::StampedTransform transform;
				try
				{ros::Time now = ros::Time::now();
						ROS_INFO("Looking up transform from timestamp: %d\nCurrent timestamp: %d", time_stamp, int(now.toSec()));
					 	//listener.lookupTransform("hmd_imu_frame_r", "map", ros::Time(0), transform);
						
						//pcl_ros::transformAsMatrix(transform, camera_pose);
						//last_transform_time_stamp = time_stamp;
						//quat_tf = transform.getRotation();
					if (last_transform_time_stamp != time_stamp)
					{
						ros::Time now = ros::Time::now();
						ROS_INFO("Looking up transform from timestamp: %d\nCurrent timestamp: %d", time_stamp, int(now.toSec()));
					 	listener.lookupTransform("map", "zed_right_camera_frame", ros::Time(time_stamp), transform);
						
						pcl_ros::transformAsMatrix(transform, camera_pose);
						last_transform_time_stamp = time_stamp;
						quat_tf = transform.getRotation();
					}
			 	}
				catch (tf::TransformException ex)
				{
				      ROS_ERROR("%s",ex.what());
				      ros::Duration(1.0).sleep();
				}
						std::cout<<camera_pose;
				
				//pcl::transformPointCloud (*cloud, *cloud, camera_pose);
				Eigen::Matrix4f camera_pose_old = camera_pose;
				goal_point.pose.position.x = camera_pose(0,3);
				goal_point.pose.position.y = camera_pose(1,3);
				goal_point.pose.position.z = camera_pose(2,3);
				geometry_msgs::Quaternion quat;
				tf::quaternionTFToMsg(quat_tf, quat);
				goal_point.pose.orientation = quat;
				goal_point.header.frame_id = "map";
		    		goal_point.header.stamp =  ros::Time(time_stamp);
				pubGoal.publish(goal_point);
				for(size_t i = 0; i < classes.size(); i++)
				{
						
						
					ros::Time start_frustum = ros::Time::now();
					ROS_INFO("u = %d, v = %d", pixel[i].x+(pixel[i].width/2), pixel[i].y+(pixel[i].height/2));
					cv::Point2d point2d(pixel[i].x+(pixel[i].width/2), pixel[i].y+(pixel[i].height/2));
					cv::Point3d point = cam_model_.projectPixelTo3dRay(point2d);
					//cv::Point3d point = backProjectPixelToWorld(pixel[i].x+(pixel[i].width/2)*1, pixel[i].y+(pixel[i].height/2), 1);
					ROS_INFO("x = %f, y = %f",point.x, point.y);
					camera_pose(0,3) = camera_pose(0,3) - point.x;
					
					camera_pose(1,3) = camera_pose(1,3) - point.y;
					camera_pose(2,3) = camera_pose(2,3);
					ROS_INFO("x = %f, y = %f, z = %f", camera_pose(0,3), camera_pose(1,3),camera_pose(2,3));
						
					fc.setCameraPose (camera_pose);
					fc.setInputCloud(cloud);
					fc.setVerticalFOV(vertical_fov);
					fc.setHorizontalFOV(horizontal_fov);
					fc.setNearPlaneDistance(near_plane_distance);
					fc.setFarPlaneDistance(far_plane_distance);

					//std::vector<cv::Point3d> frustum = boundingBox2D_to_frustum(pixel[i]);
					fc.setCameraPose (camera_pose);
					//cv::Rect temp = pixel[i];
					//pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundingbox_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  
					//for(int j = 0; j < frustum.size(); j++)
					//{
					//	ROS_INFO("vertex %d: x = %f, y = %f, z = %f", j, frustum[j].x, frustum[j].y, frustum[j].z);
					//	boundingbox_ptr->push_back(pcl::PointXYZRGB(frustum[j].x, frustum[j].y, frustum[j].z));
					//}
					// pcl::ConvexHull<pcl::PointXYZRGB> hull;
					  //hull.setInputCloud(boundingbox_ptr);
					 // hull.setDimension(3);
					 // std::vector<pcl::Vertices> polygons;

					  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
					 // hull.reconstruct(*surface_hull, polygons);
					//pcl::CropHull<pcl::PointXYZRGB> bb_filter;

  					//pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects (new pcl::PointCloud<pcl::PointXYZRGB>);
					 // bb_filter.setDim(2);
					 // bb_filter.setInputCloud(cloud);
					  //bb_filter.setHullIndices(polygons);
					  //bb_filter.setHullCloud(surface_hull);
			         	pcl::PointCloud<pcl::PointXYZRGB> object_cloud;	
					  //bb_filter.filter(object_cloud);
					  //std::cout << objects->size() << std::endl;

					//


					fc.filter(object_cloud);
					
					ros::Time end_frustum = ros::Time::now();
					float execution_time = (end_frustum - start_frustum).toNSec() * 1e-6;
					ROS_INFO("time to process frustum: %f nsec", execution_time);
					ros::Time start_color = ros::Time::now();
					pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
					nh_.getParam("/zed_stereo_node/min_cluster_size", min_cluster_size);
					nh_.getParam("/zed_stereo_node/point_color_threshold", point_color_thresh);
					nh_.getParam("/zed_stereo_node/region_color_threshold", region_color_thresh);
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);
					
					  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
					  reg.setInputCloud (object_cloud.makeShared());
					  reg.setSearchMethod (tree);
					  reg.setDistanceThreshold (10);
					  reg.setPointColorThreshold (point_color_thresh);
					  reg.setRegionColorThreshold (region_color_thresh);
					  reg.setMinClusterSize (min_cluster_size);
					  std::vector <pcl::PointIndices> clusters;
					  reg.extract(clusters);
					sensor_msgs::PointCloud2 cloud_out_color;
					
					if(!clusters.empty())
					{
					  	cloud_color = reg.getColoredCloud();
							
						ros::Time end_color = ros::Time::now();
						execution_time = (end_color - start_color).toNSec() * 1e-6;
						ROS_INFO("time to process color region: %f nsec", execution_time);
						sensor_msgs::PointCloud2 cloud_out_color;
					
						//pcl::transformPointCloud (*cloud_color, *cloud_color, camera_pose);
						pcl::toROSMsg(*cloud_color, cloud_out_color);
						cloud_out_color.header.frame_id = "map";
						pubSegment_color.publish(cloud_out_color);
					}
					
					
		
					//pcl::transformPointCloud (*cloud_color, *cloud_color, camera_pose);
					//for(int row = 0; row < cloud_reg->height; row++){
					//	for(int col = 0; col < cloud_reg->width; col++){
					//			ROS_INFO("row = %f, col = %f", row,col);
					//		if(row >= pixel[i].y && row < pixel[i].y+pixel[i].height && col > pixel[i].x && col < pixel[i].x+pixel[i].width)
					//		{
					//				
					//			cloud_color->push_back(cloud_reg->at(row,col));
					//		}
					//	}
					//}
						
					
		    			//fc.setRegionOfInterest(pixel[i].x, pixel[i].y, pixel[i].width, pixel[i].height);
					ROS_INFO("Cloud point %d", cloud->width);
					sensor_msgs::PointCloud2 cloud_out;
					//ros_cloudR = *cloud_out_r;
					cloud_out.header.frame_id = "hmd_imu_frame_r";
					//pcl_ros::transformPointCloud("odom", object_cloud, object_cloud, listener);
					//pcl_ros::transformPointCloud("zed_right_camera_optical_frame", object_cloud, object_cloud, listener);
					//pcl_ros::transformPointCloud("hmd_imu_frame_r", object_cloud, object_cloud, listener);
					pcl::toROSMsg(object_cloud, cloud_out);
					ROS_INFO("Cloud point %d", object_cloud.points.size());
					pubSegment.publish(cloud_out);
					camera_pose = camera_pose_old;
				}
			}
		
		}

	}

	std::vector<cv::Point3d> boundingBox2D_to_frustum(cv::Rect bounding_box)
	{
		std::vector<cv::Point3d> frustum;
		int x1 = bounding_box.x;
		int y1 = bounding_box.y;
		int x2 = bounding_box.x + bounding_box.width;
		int y2 = bounding_box.y + bounding_box.height;
		x1 = x1 - (x2 - x1) * expand_ratio / 2;
		y1 = y1 - (y2 - y1) * expand_ratio / 2;
		x2 = x2 + (x2 - x1) * expand_ratio / 2;
		y2 = y2 + (y2 - y1) * expand_ratio / 2;

		cv::Point2d point1(x1, y1);
		cv::Point3d point = cam_model_.projectPixelTo3dRay(point1);
		point.z = zmin;
		frustum.push_back(point);

		cv::Point2d point2(x2, y1);
		point = cam_model_.projectPixelTo3dRay(point2);
		point.z = zmin;
		frustum.push_back(point);

		cv::Point2d point3(x2, y2);
		point = cam_model_.projectPixelTo3dRay(point3);
		point.z = zmin;
		frustum.push_back(point);

		cv::Point2d point4(x1, y2);
		point = cam_model_.projectPixelTo3dRay(point4);
		point.z = zmin;
		frustum.push_back(point);

		cv::Point2d point5(x1, y1);
		point = cam_model_.projectPixelTo3dRay(point5);
		point.z = zmax;
		frustum.push_back(point);

		cv::Point2d point6(x2, y1);
		point = cam_model_.projectPixelTo3dRay(point6);
		point.z = zmax;
		frustum.push_back(point);

		cv::Point2d point7(x2, y2);
		point = cam_model_.projectPixelTo3dRay(point7);
		point.z = zmax;
		frustum.push_back(point);

		cv::Point2d point8(x1, y2);
		point = cam_model_.projectPixelTo3dRay(point8);
		point.z = zmax;
		frustum.push_back(point);
		return frustum;
		
	}
	cv::Point3d backProjectPixelToWorld(int x, int y, double depth)
	{
		cv::Point3d point;

		point.x = (x - cx_r) / fx_r;
		point.y = (y - cy_r) / fx_r;
		point.z = depth;
		
		return point;
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
