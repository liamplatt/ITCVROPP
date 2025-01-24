#!/usr/bin/env python

import numpy as np
import math
import cv2
import rospy
import numpy.linalg as LA
from collections import deque
import random
from nav_msgs.msg import Odometry
from create_msgs.msg import Bumper
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion


	
class Roomba:
	def __init__(self,r,d = 0.235):
		print("Starting wheel encoder subscriber")
		self.subWheelEncoders = rospy.Subscriber("joint_states", JointState, self.JointStateCB)
		self.subZEDOdometry = rospy.Subscriber("zed2/zed_node/pose",PoseStamped,self.ZEDOdometryCB)
		self.subWheelEncoders = rospy.Subscriber("odom", Odometry, self.RoombaOdomCB)
		self.subJoy = rospy.Subscriber("joy", Joy, self.joyCB)
		self.pubsys = rospy.Publisher("syscommand", String, queue_size = 1)
		self.pubCmdVel = rospy.Publisher("cmd_vel", Twist, queue_size = 1)
		#self.SensorFusion = SensorFusion(2)
		self.u = np.zeros((2,1))
		self.xE = np.zeros((3,1))
		self.stateZed = self.xE
		self.stateRoomba = self.xE
		self.xD = self.xE
		self.CMD = Twist()
		self.dt = r
		self.d = d
		self.VaNeg = 1
		self.VfNeg = 1
		self.lastDataAvailable = False
		self.index = 0
		self.vel = Twist()
		self.reset = False
		self.syscommand = String()
	def joyCB(self, msg):
		print("I Heard Joy MEssage!")
		self.vel.angular.z = msg.axes[0]
		self.vel.linear.x = msg.axes[1]
		self.reset = msg.buttons[2]
	def ZEDOdometryCB(self, msg):
		orientation_q = msg.pose.orientation
	        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		self.stateZed = np.array([[msg.pose.position.x],[msg.pose.position.y],[yaw]])
	def RoombaOdomCB(self,msg):
		orientation_q = msg.pose.pose.orientation
	        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		self.stateRoomba = np.array([[msg.pose.pose.position.x],[msg.pose.pose.position.y],[yaw]])
        def JointStateCB(self,msg):
		
		if self.lastDataAvailable:
			#print("Measured wheel velocities: ")
			#print(msg.velocity[0], msg.velocity[1])
			self.wheelDists = np.array([msg.velocity[0], msg.velocity[1]])/1000 #np.array([msg.position[0], msg.position[1]])/100-self.lastWheelDists
			#print("Measured wheel distances(m): ")
			#print(self.wheelDists)
			
			#self.xE = self.SensorFusion.Fuse(np.hstack(self.xE,self.zedxE))
		else:
			self.wheelDists = np.array([msg.velocity[0], msg.velocity[1]])/1000

		self.lastWheelDists = np.array([msg.velocity[0], msg.velocity[1]])/1000
		
		if self.index >= 2:
			self.lastDataAvailable = True
		self.index += 1
		
	def tick(self):
		self.sendCmdVel(self.vel.linear.x,self.vel.angular.z)
		#print("ZED2 State: ")
		#print(self.stateZed)
		#print("Roomba Odom State: ")
		#print(self.stateRoomba)
		if self.reset:
			print("Sending Reset")
			self.syscommand.data = "reset"
			self.pubsys.publish(self.syscommand)
			self.syscommand.data = ""
	def applyMotionModel(self,dr,dl):
		#Dead reckoning from wheel encoders
		self.ds = self.VfNeg * (dr + dl)/2
		#print("Vf {d}".format(d=self.ds))
		self.dtheta = self.normalize_angle(self.VaNeg *(dr - dl)/self.d)
		#print("dTheta {d}".format(d=self.dtheta))
		theta_last = self.xD[2][0]
		theta_last = self.normalize_angle(theta_last)
		#print("Theta {d}".format(d=theta_last))
		self.xD = self.xD + np.array([  [self.ds*self.dt*math.cos(theta_last+self.dt*self.dtheta/2)],
						[self.ds*self.dt*math.sin(theta_last+self.dt*self.dtheta/2)],
						[self.dtheta*self.dt             ]   ])
		self.xD[2][0] = self.normalize_angle(self.xD[2][0])
		return self.xD
	def getxE(self):
		return self.xE
	def sendCmdVel(self,Vf,Va):
		if Va < 0:
			self.VaNeg = -1
		else:
			self.VaNeg = 1
		if Vf < 0:
			self.VfNeg = -1
		else:
			self.VfNeg = 1
		self.CMD.linear.x = Vf
		self.CMD.angular.z = Va
		self.u[0] = Vf
		self.u[1] = Va
		self.pubCmdVel.publish(self.CMD)
	def normalize_angle(self, a):
		"""
		Wrap the angle between 0 and 2 * pi.

		Args:
		angle (float): angle to wrap.

		Returns:
		 The wrapped angle.

		"""
		theta = np.fmod(a + np.pi, 2*np.pi)-np.pi
		if theta < -np.pi:
			theta += 2*np.pi
		return theta
def main():
	
	r = 1.0/10.0
	robot = Roomba(r)
	
	rospy.init_node('Localizer', anonymous=True)
	rate = rospy.Rate(60) # 10hz
	OdomMSG = Odometry()
	print("Set up Odometry msg...")
	while not rospy.is_shutdown():
		robot.tick()
		rate.sleep()
	
if __name__ == "__main__":
	main()
