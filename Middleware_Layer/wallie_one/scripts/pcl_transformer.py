#!/usr/bin/env python
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import numpy as np
import rospy

class BlankImagePub(object):
	def __init__(self):
		self.image_l = np.zeros([1028,720,3], dtype = np.uint8)
		self.image_r = np.zeros([1028,720,3], dtype = np.uint8)
		self.br = CvBridge()
		self.loop_rate = rospy.Rate(100) # 30 Hz
		self.image_pub_l = rospy.Publisher("blank_image_l", CompressedImage, queue_size = 10)
		self.image_pub_r = rospy.Publisher("blank_image_r", CompressedImage, queue_size = 10)
	def run(self):
		rospy.loginfo("Running blank image pub.")

		rospy.loginfo("{e}".format(e = CompressedImage.__slots__))
		while not rospy.is_shutdown():

			self.image_pub_l.publish(self.br.cv2_to_compressed_imgmsg(self.image_l))
			self.image_pub_r.publish(self.br.cv2_to_compressed_imgmsg(self.image_r))
			self.loop_rate.sleep()

rospy.init_node('pcl_transformer',anonymous=True)
BIP = BlankImagePub()
BIP.run()
	

