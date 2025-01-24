/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * Author : Andy McEvoy, Dave Coleman
 * Desc   : Allows manual control of a TF through the keyboard
 */
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <wallie_one/ZedPubStereoConfig.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


#include <image_geometry/pinhole_camera_model.h>


#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/hal/interface.h>

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

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <fstream>
#include <string>

#include <ros/package.h>

class ManualTFAlignment
{
public:

  /*
   * \brief
   */
  ManualTFAlignment()
	: nh_("~")
{
// set defaults
  mode_ = 1;
  delta_ = 0.010;

  // initial camera transform
  double x, y, z, roll, pitch, yaw;

  nh_.getParam("initial_x", x);
  nh_.getParam("initial_y", y);
  nh_.getParam("initial_z", z);
  nh_.getParam("initial_roll", roll);
  nh_.getParam("initial_pitch", pitch);
  nh_.getParam("initial_yaw", yaw);
  nh_.getParam("file_package", file_package_);
  nh_.getParam("file_name", file_name_);
  nh_.getParam("topic_name", topic_name_);
  nh_.getParam("from", from_);
  nh_.getParam("to", to_);

  setPose(Eigen::Vector3d(x, y, z), Eigen::Vector3d(roll, pitch, yaw));

  // default, save in tf_keyboard_cal/data
  std::string package_path = ros::package::getPath(file_package_);
  save_path_ = package_path + file_name_;

  // listen to keyboard topic
  keyboard_sub_ = nh_.subscribe(topic_name_, 100,
                                &ManualTFAlignment::keyboardCallback, this);

}

  /*
   * \brief
   */
  void keyboardCallback(const keyboard::Key::ConstPtr& msg)
{
  int entry = msg->code;
  const double fine = 0.001;
  const double coarse = 0.01;
  const double very_coarse = 0.1;

  //std::cout << "key: " << entry << std::endl;

  switch(entry)
  {
    case 112: //
      writeTFToFile();
      break;
    case 117: // (very coarse delta)
      std::cout << "Delta = very coarse (0.1)" << std::endl;
      delta_ = very_coarse;
      break;
    case 105: // (coarse delta)
      std::cout << "Delta = coarse (0.01)" << std::endl;
      delta_ = coarse;
      break;
    case 111: // (fine delta)
      std::cout << "Delta = fine (0.001)" << std::endl;
      delta_ = fine;
      break;

    // X axis
    case 113: // up
      updateTF(1, delta_);
      break;
    case 97: // down
      updateTF(1, -delta_);
      break;

    // y axis
    case 119: // up
      updateTF(2, delta_);
      break;
    case 115: // down
      updateTF(2, -delta_);
      break;

    // z axis
    case 101: // up
      updateTF(3, delta_);
      break;
    case 100: // down
      updateTF(3, -delta_);
      break;

    // roll
    case 114: // up
      updateTF(4, delta_);
      break;
    case 102: // down
      updateTF(4, -delta_);
      break;

    // pitch
    case 116: // up
      updateTF(5, delta_);
      break;
    case 103: // down
      updateTF(5, -delta_);
      break;

    // yaw
    case 121: // up
      updateTF(6, delta_);
      break;
    case 104: // down
      updateTF(6, -delta_);
      break;

    default:
      // don't do anything
      break;
  }
}

  /*
   * \brief
   */
  void printMenu()
{
std::cout << "Manual alignment of camera to world CS:" << std::endl;
  std::cout << "=======================================" << std::endl;
  std::cout << "MOVE: X  Y  Z  R  P  YAW " << std::endl;
  std::cout << "------------------------" << std::endl;
  std::cout << "up    q  w  e  r  t  y " << std::endl;
  std::cout << "down  a  s  d  f  g  h " << std::endl;
  std::cout << std::endl;
  std::cout << "Fast: u " << std::endl;
  std::cout << "Med:  i " << std::endl;
  std::cout << "Slow: o " << std::endl;
  std::cout << "Save: p " << std::endl;
}

  /*
   * \brief
   */
  void publishTF()
{
static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  // set camera pose translation
  transform.setOrigin( tf::Vector3( translation_[0],
                                    translation_[1],
                                    translation_[2]) );

  // set camera pose rotation
  q.setRPY(rotation_[0], rotation_[1], rotation_[2]);
  transform.setRotation(q);

  // publish
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), from_ , to_));
}

  /*
   * \brief
   */
  void setPose(Eigen::Vector3d translation, Eigen::Vector3d rotation)
{
  translation_ = translation;
  rotation_ = rotation;
}

  /*
   * \brief
   */
  void updateTF(int mode, double delta)
{
  switch(mode)
  {
    case 1:
      translation_ += Eigen::Vector3d(delta, 0, 0);
      break;
    case 2:
      translation_ += Eigen::Vector3d(0, delta, 0);
      break;
    case 3:
      translation_ += Eigen::Vector3d(0, 0, delta);
      break;
    case 4:
      rotation_ += Eigen::Vector3d(delta, 0, 0);
      break;
    case 5:
      rotation_ += Eigen::Vector3d(0, delta, 0);
      break;
    case 6:
      rotation_ += Eigen::Vector3d(0, 0, delta);
      break;
    default:
      // don't do anything
      break;
}

  ros::NodeHandle nh_;

  // Name of class
  std::string name_ = "manipulation_data";

  Eigen::Vector3d translation_;
  Eigen::Vector3d rotation_;
  std::string save_path_;
  int mode_;
  double delta_;
  std::string from_;
  std::string to_;
  std::string file_package_;
  std::string file_name_;
  std::string topic_name_;
  ros::Subscriber keyboard_sub_;
};





public:
ros::NodeHandle nh_;

  // Name of class
  std::string name_ = "manipulation_data";

  Eigen::Vector3d translation_;
  Eigen::Vector3d rotation_;
  std::string save_path_;
  int mode_;
  double delta_;
  std::string from_;
  std::string to_;
  std::string file_package_;
  std::string file_name_;
  std::string topic_name_;
  ros::Subscriber keyboard_sub_;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_keyboard");
  ROS_INFO_STREAM_NAMED("tf_keyboard", "Starting keyboard control");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  tf_keyboard_cal::ManualTFAlignment tf_align;
  tf_align.printMenu();

  ros::Rate rate(30.0); // hz
  while ( ros::ok() )
  {
    // publish transform to camera
    tf_align.publishTF();

    rate.sleep();
  }

}
