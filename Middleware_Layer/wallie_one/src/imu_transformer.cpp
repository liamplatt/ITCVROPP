#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"

tf::TransformBroadcaster* tfB_;
tf::StampedTransform transform_;
tf::Quaternion tmp_;
ros::Timer timer;
#ifndef TF_MATRIX3x3_H
  typedef btScalar tfScalar;
  namespace tf { typedef btMatrix3x3 Matrix3x3; }
#endif

float x = 0;
float y = 0;
float yaw = 0;
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
	// Need to grab axis[0] (y)
	// axis[1] (x)
	//joy_msg->axes[1]
	//tf::quaternionMsgToTF(joy_msg.orientation, tmp_);
	  ROS_INFO("recieved imu tf\n");

	  //tfScalar yaw, pitch, roll;
	  //tf::Matrix3x3(tmp_).getRPY(roll, pitch, yaw);

	  //tmp_.setRPY(roll, pitch, 0.0);
	//Set the transform origin to this
	x = x + joy_msg->axes[1];
	y = y + joy_msg->axes[0];
	yaw = joy_msg->axes[2]*3.14;
	transform_.getOrigin().setX(x);
	transform_.getOrigin().setY(y);
	transform_.getOrigin().setZ(0.0);
	 tf::Quaternion q;
	 
  q.setRPY(yaw, 0, 0);
  q.normalize();
  transform_.setRotation(q);
	//fprintf("X = %d\n", transform_.getOrigin().getX());
	//fprintf("Y = %d\n", transform_.getOrigin().getY());
}
void broadcast(const ros::TimerEvent&)
{
  //tfB_->sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "map", "hmd_imu_frame"));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, ROS_PACKAGE_NAME);

  ros::NodeHandle n;
  ROS_INFO("Running imu tf\n");
  tfB_ = new tf::TransformBroadcaster();
  transform_.getOrigin().setX(0.0);
  transform_.getOrigin().setY(0.0);
  transform_.getOrigin().setZ(0.0);
  transform_.child_frame_id_ = "map";
	
  //ros::Subscriber joy_subscriber = n.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
  

	timer = n.createTimer(ros::Duration(1.0/60.0f), broadcast);

  ros::spin();

  delete tfB_;

  return 0;
}
