#!/usr/bin/env python
import numpy as np
import math
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2




class PCViewer():
	def __init__(self):
		pass
	def PointCloudCB(self):
		pass


def display():            
    rospy.init_node('ViewController',anonymous=True)
    subL = rospy.Subscriber('stereo/stereo_image/image_raw', CompressedImage, disp.dispCallback)
    

    rospy.spin()
    
if __name__ == '__main__':
    display()
