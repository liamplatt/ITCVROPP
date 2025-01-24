#!/usr/bin/env python
import numpy as np
import math
import socket
import cv2
import rospy
import tf
import tf2_ros
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from std_msgs.msg import Bool
import roslib
from std_msgs.msg import String, Header
from cStringIO import StringIO


class Broadcaster():
	def __init__(self):
		self.br = tf.TransformBroadcaster()
		self.x = 0
		self.y = 0
		self.z = 0
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.update = 0
		self.cmd_vel = Twist()
		self.click = Bool()
		self.timer = rospy.Timer(rospy.Duration(.001),self.callback);
		self.op = rospy.get_param('/zed_stereo_node/op_control', 0)
		self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size = 1)
		self.click_pub = rospy.Publisher("/pc_click", Bool, queue_size=1)
		self.pose_pub = rospy.Publisher("/hmd_pose", Bool, queue_size=1)
		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
	def callback(self, timer):
		q = tf.transformations.quaternion_from_euler(0, self.yaw, 0)
		self.br.sendTransform((self.x, self.y, self.z), q, rospy.Time.now(), "hmd_imu_frame_l", "zed_left_camera_optical_frame")
		self.br.sendTransform((self.x, self.y, self.z), q, rospy.Time.now(), "hmd_imu_frame_r", "zed_right_camera_optical_frame")
		
	def joy_callback(self, msg):
		self.z = msg.axes[1]
		self.x = -msg.axes[0]
		self.cmd_vel.angular.z = msg.axes[0]*np.pi/4
		self.cmd_vel.linear.x = msg.axes[1]/4
		if msg.buttons[2]:
			self.click.data = 1
			self.click_pub.publish(self.click)
			self.update = 1
		self.op = rospy.get_param('/zed_stereo_node/op_control', 0)
		if msg.buttons[1]:
			self.op = not self.op

		if msg.buttons[3]:
			self.x = 0
			self.y = 0
			self.roll = 0
			self.z = 0
			self.pitch = 0
			self.yaw  = 0
		rospy.set_param('/zed_stereo_node/op_control', self.op)
		self.op = rospy.get_param('/zed_stereo_node/op_control', 0)
		
		if not self.op:
			self.x = 0
			self.y = 0
			self.z = 0
			self.roll = 0
			self.pitch = 0
			self.yaw  = 0
			self.cmd_vel_pub.publish(self.cmd_vel)
		if self.update:
			if msg.buttons[5]:
				self.roll = self.roll + np.pi/16
			if msg.buttons[6]:
				self.roll = self.roll - np.pi/16
				
			if msg.buttons[10]:
				self.pitch = self.pitch + np.pi/16
			if msg.buttons[9]:
				self.pitch = self.pitch - np.pi/16
			self.update = 0
		
		self.yaw = msg.axes[2]*2*np.pi
		

if __name__ == '__main__':
	rospy.init_node("TFBroadcaster")
	print("Subscribing to joy")
	broadcast = Broadcaster()
	print("done")
	rospy.spin()

