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
from cv_bridge import CvBridge
from sensor_msgs.msg import JointState, Joy, Image
from geometry_msgs.msg import Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class Node:
	def __init__(self):
	
		self.subDepth = rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, self.depthCB)
		self.pubDepth = rospy.Publisher("image", Image, queue_size = 1)
		self.imgOut = Image()
		self.br = CvBridge()
	def depthCB(self, msg):
		imgIn = self.br.imgmsg_to_cv2(msg)
		self.imgOut = self.br.cv2_to_imgmsg(imgIn)
		self.pubDepth.publish(self.imgOut)

def main():
	rospy.init_node('SlamRepub', anonymous=True)
	n = Node()
	rospy.spin()

	
if __name__ == "__main__":
	main()

