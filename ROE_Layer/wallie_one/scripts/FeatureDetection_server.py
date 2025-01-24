#!/usr/bin/env python

from __future__ import print_function

from wallie_one.srv import FeatureDetection, FeatureDetectionResponse
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rospy, cv2, roslib, math, os, yaml, thread, shlex, subprocess

def handle_FeatureDetection(req):
    feat_det = cv2.FastFeatureDetector_create()
    feat_det.setNonmaxSuppression(False)
    image_np = CompImg2cv2(req.img)
    gray_image = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
    ft_pts = feat_det.detect(gray_image, None)
    for ft_pts in ft_pts:
            x,y = ft_pts.pt
            cv2.circle(image_np,(int(x),int(y)), 3, (0,0,255), -1)
    b = CvBridge()
    image_ros = b.cv2_to_compressed_imgmsg(image_np)        
    return image_ros
def CompImg2cv2(img):
    
    np_arr = np.fromstring(img.data, np.uint8)
    return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
def FeatureDetection_server():
    rospy.init_node('FeatureDetection_server')
    s = rospy.Service('FeatureDetection', FeatureDetection, handle_FeatureDetection)
    rospy.spin()

if __name__ == "__main__":
    FeatureDetection_server()