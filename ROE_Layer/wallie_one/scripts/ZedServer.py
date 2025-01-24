#!/usr/bin/env python
import numpy as np
import math
import socket
from cStringIO import StringIO
import cv2
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2




class ImageRelay():
    def __init__(self, topic, port):
	self.image = None
	self.br = CvBridge()
	self.sub = rospy.Subscriber(topic, Image, self.image_cb,queue_size=10)
    	self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	self.server.bind((socket.gethostname(), port))
	self.server.listen(5)
	print("Socket image publishing server started...")
    def image_cb(self, msg):
	print("Socket img cb...")
	(clientsock, client_address) = self.server.accept()
	self.image = self.br.imgmsg_to_cv2(msg)
	print(clientsock)
	# Send the cv Mat image
	f = StringIO()
	np.savez_compressed(f, frame=self.image)
	f.seek(0)
	out = f.read()
	clientsock.sendall(out)

def main():
    rospy.init_node('ZedImageSocketPub',anonymous=True)
    topic = "zed_node/right/image_rect_color"
    if rospy.has_param('~topic'):
        topic = rospy.get_param('~topic')
    if rospy.has_param('~port'):
        port = rospy.get_param("~port")
    else:
        port = 5005
    IR = ImageRelay(topic, port)
    rospy.spin()

if __name__ == '__main__':
    main()
