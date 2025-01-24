#!/usr/bin/env python
import numpy as np
import math
import socket
import cv2
import rospy
import roslib
from cv_bridge import CvBridge
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cStringIO import StringIO



class ImageRelay():
	def __init__(self, topic, frate):
		self.image = None
		self.pub = None
		self.frate = frate
		self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.connection = socket.create_connection(("192.168.1.110", 5005))
		self.bridge = CvBridge()
		rospy.loginfo('Starting publisher.')
		self.image_pub = rospy.Publisher(topic, Image, queue_size=1)
		print("Socket image publishing server started...");print(frate)
		self.msg_header = Header();
		self.ros_image = Image()
	def run(self):
		buff = '';rate = rospy.Rate(self.frate)
		self.server.bind(('',5005))
		while not rospy.is_shutdown():
			while True:
				recieving_buffer = self.connection.recv(1024)
				if not recieving_buffer: break
				buff += recieving_buffer
			s = StringIO(buff)
			s.seek(0)
			img = np.load(s)['frame']
			if img.size == 921600:
			    # print("image decode success: ")
			    # print(img.size)
			    img_msg = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
			    self.image_pub.publish(img_msg)
		self.connection.close()

def main():            
    rospy.init_node('ZedImageSub',anonymous=True)
    topic = "zed/right/image_rect_color"
    frate = 120
    if rospy.has_param('~frate'):
    	frate = rospy.get_param('~frate')
    	print(frate)
    if rospy.has_param('~topic'):
    	topic = rospy.get_param('~topic')
    IR = ImageRelay(topic, frate)
    IR.run()
	
    
if __name__ == '__main__':
    main()
