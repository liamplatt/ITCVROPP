#!/usr/bin/env python
import numpy as np
import math
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage
from wallie_one.msg import stereo_image
from wallie_one.srv import FeatureDetection, FeatureDetectionResponse

class TouchScreen:
    def __init__(self, name):
        self.displayWidth = 1024
        self.displayHeight = 600
        self.imgL = None
        self.imgR = None
        self.img = None
        self.sized_img = None
        self.kp = 0
        self.counter = 0
        if name != None:
            self.name = name
        else:
            name = 'cv_vid'
        cv2.namedWindow(self.name, cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(self.name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        rospy.loginfo('Starting display...')
    def dispCallback(self, data):
        #Data is a sensor_msgs/CompressedImage
        #Send service request with same msg type
        img_ft_pts = self.FeatureDetection_client(data)
        #Data received is CompressedImage
        #Convert raw compressed image data to np array
        self.img = self.CompImg2cv2(img_ft_pts)
        #Decode data 
        if self.img is None:
            rospy.loginfo('Could not read video frames,')
        else:
            cv2.imshow(self.name, self.img)
            cv2.waitKey(1)
    def FeatureDetection_client(self, img):
        from beginner_tutorials.srv import FeatureDetection, FeatureDetectionResponse
        FeatureDetection = rospy.ServiceProxy('FeatureDetection', FeatureDetection)
        try:
            resp = FeatureDetection(img)
            return resp.img_feats
        except rospy.ServiceException as err:
            rospy.loginfo("Service did not process request: " + str(err))
            return img
    def HCatImg(self,frameL,frameR):
        return np.hstack((frameL, frameR))
    def CompImg2cv2(self, img):
        np_arr = np.fromstring(img.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    def __del__(self):
        cv2.destroyAllWindows()

def display():            
    rospy.init_node('display',anonymous=True)
    disp = TouchScreen('cv_img')
    subL = rospy.Subscriber('stereo/stereo_image/image_raw', CompressedImage, disp.dispCallback)
    

    rospy.spin()
    
if __name__ == '__main__':
    display()
