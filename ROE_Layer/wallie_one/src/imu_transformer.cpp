#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"

tf::TransformBroadcaster* tfB_;
tf::StampedTransform transform_;
tf::Quaternion tmp_;

#ifndef TF_MATRIX3x3_H
  typedef btScalar tfScalar;
  namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif

void imuMsgCallback(const sensor_msgs::Imu& imu_msg)
{
  tf::quaternionMsgToTF(imu_msg.orientation, tmp_);

  tfScalar yaw, pitch, roll;
  tf::Matrix3x3(tmp_).getRPY(roll, pitch, yaw);

  tmp_.setRPY(roll, pitch, 0.0);

  transform_.setRotation(tmp_);
	transform_.frame_id_ = imu_msg.header.frame_id;
  transform_.stamp_ = imu_msg.header.stamp;

  tfB_->sendTransform(transform_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  ros::NodeHandle n;
  
  tfB_ = new tf::TransformBroadcaster();
  transform_.getOrigin().setX(0.0);
  transform_.getOrigin().setY(0.0);
  transform_.getOrigin().setZ(0.0);
  transform_.child_frame_id_ = "map";

  ros::Subscriber imu_subscriber = n.subscribe("imu_hmd", 10, imuMsgCallback);

  ros::spin();

  delete tfB_;

  return 0;
}
