#!/usr/bin/env python
import numpy as np
from scipy import ndimage
from wand.image import Image
import wand
from cv_bridge import CvBridge, CvBridgeError
import rospy, cv2, roslib, math, os, yaml, thread, shlex, subprocess
from std_msgs.msg import String, Header
from sensor_msgs.msg import CompressedImage, CameraInfo
from wallie_one.msg import stereo_image

def range_of_vals(x, axis=0):
    return np.max(x, axis=axis), np.min(x, axis=axis),

def print_stuff(x,y):
	rospy.loginfo("x = ")
	rospy.loginfo(range_of_vals(x.ravel()))
	rospy.loginfo("y = ")
	rospy.loginfo(range_of_vals(y.ravel()))
class StereoCamera:
    def __init__(self, cam_id_L, cam_id_R, frate):
        rospy.init_node('stereo', anonymous = True)
        self.bridge = CvBridge()
        rospy.loginfo('Starting publisher.')
        self.stereo_image_pub = rospy.Publisher("stereo/stereo_image/image_raw", CompressedImage, queue_size=1)
        self.stereo_image_info_pub = rospy.Publisher("stereo/stereo_image/camera_info", CameraInfo, queue_size=1)
        self.camera_info = CameraInfo()
        self.msg_header = Header()
        self.ros_image = CompressedImage()
        rospy.loginfo('Reading calibration files.')
        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.left_yaml_file = dir_path+"/../calibration/left.yaml"
        self.right_yaml_file = dir_path+"/../calibration/right.yaml"
        self.frate = frate
        rospy.loginfo('Finding camera indices...')
        available_ports,working_ports,non_working_ports  = self.list_ports()
        rospy.loginfo(available_ports)
        rospy.loginfo(working_ports)
        rospy.loginfo(non_working_ports)

        if len(working_ports) < 2:
            rospy.loginfo('Could not find cameras!')
	elif len(working_ports) == 3:
	    rospy.loginfo('Found three cameras. Using ports 0 and 2.')
	    
            self.camR = cv2.VideoCapture(working_ports[0])
            self.camL = cv2.VideoCapture(working_ports[2])
	else:
            self.camR = cv2.VideoCapture(working_ports[0])
            self.camL = cv2.VideoCapture(working_ports[1])

    def pub_image(self, publisher, image, header):
        try:
            self.ros_image = self.Img2CompImg(image)
            self.ros_image.header = header
            publisher.publish(self.ros_image)
        except CvBridgeError as e:
            rospy.loginfo(e)
    def yaml_to_camera_info(self,yaml_file):
        with open(yaml_file, "r") as f :
            calib_data = yaml.load(f)
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        return camera_info_msg
    def Img2CompImg(self,Img):
        return self.bridge.cv2_to_compressed_imgmsg(Img)
    def barrel(self,img):
	print(type(img))
	print(type(wand.__version__))
	array = np.zeros([480, 620, 3], dtype=np.uint8)
        array[:, :] = [0xff, 0x00, 0x00]
	#['header', 'height', 'width', 'encoding', 'is_bigendian', 'step', 'data'] 
	with Image.from_array(array) as img_wand:
            img_wand.distort('barrel', (0.2,0.0,0.0,1.0))
	    dist_img = np.array(img_wand)
	    return dist_img
    def barrel_distortion_scipy(self, img):
    	k_1 = 0.2
	k_2 = 0.05
	w,h,_ = img.shape
	rospy.loginfo("Width and height of original image are: ")
	rospy.loginfo(w)
	rospy.loginfo(h)
	x,y = np.meshgrid(np.float32(np.arange(w)),np.float32(np.arange(h))) # meshgrid for interpolation mapping

	rospy.loginfo("Range of original mesh grid: ")
	print_stuff(x,y)
	# center and scale the grid for radius calculation (distance from center of image)
	x_c = w/2 
	y_c = h/2 
	rospy.loginfo("Centers of picture are: ")
	rospy.loginfo(x_c)
	rospy.loginfo(y_c)
	x = x - x_c
	y = y - y_c
	rospy.loginfo("Range of centered and scaled mesh grid: ")
	print_stuff(x,y)
	x = x/x_c
	y = y/y_c
	radius = np.sqrt(x**2 + y**2) # distance from the center of image
	m_r = 1 + k_1*radius + k_2*radius**2 # radial distortion model
	rospy.loginfo("Radial Distortion Model: ")
	rospy.loginfo(m_r)
	# apply the model 
	x= x * m_r 
	y = y * m_r
	
	# reset all the shifting
	rospy.loginfo("After model: ")
	print_stuff(x,y)
	x= (x*x_c) +x_c
	y = (y*y_c) + y_c
	rospy.loginfo("After shifting and scaling: ")
	cords = [y.ravel(),x.ravel(),np.zeros(len(x.ravel()))]
	print_stuff(x,y)
	
	distorted = ndimage.map_coordinates(img, cords) 
	distorted.resize(img.shape) 
	cropped_image = distorted[0:w/4, 0:h/4]
    	return distorted
    def run(self):
        rospy.loginfo('Starting video capture.')
	#Need to check paths for these files and generate a yaml for each with distortion coeff matrix
	right_cam_info = CameraInfo()
	right_cam_info.width = 480
	right_cam_info.height = 620
        #right_cam_info = self.yaml_to_camera_info(self.right_yaml_file)
        #left_cam_info = self.yaml_to_camera_info(self.left_yaml_file)
        rospy.loginfo('Processing camera info...')
        rospy.loginfo('Running...')
        rate = rospy.Rate(self.frate)
	cv2.namedWindow("left")
	cv2.namedWindow("right")
	cv2.namedWindow("left_dist")
	cv2.namedWindow("right_dist")
        while not rospy.is_shutdown():
            retL,frameL = self.camL.read()
            retR,frameR = self.camR.read()
	    cv2.imshow("left", frameL)
            cv2.waitKey(1)
	    cv2.imshow("right", frameR)
            cv2.waitKey(1)
	    #Get the original image size
	    size = frameL.shape
	    #apply barrel distortion algo
	    frameL = self.barrel_distortion_scipy(frameL)
	    frameR = self.barrel_distortion_scipy(frameR)
	    #Crop and resize images
	    frameLCropped = frameL[0:133,0:173,:]
	    frameRCropped = frameR[0:133,0:173,:]
            frameLCropped = frameLCropped.resize(size)
	    frameRCropped = frameRCropped.resize(size)
	    rospy.loginfo((frameRCropped.shape))
	    cv2.imshow("left_dist", frameL)
            cv2.waitKey(1)
	    cv2.imshow("right_dist", frameRCropped)
            cv2.waitKey(1)
            if not retL or not retR:
                rospy.loginfo('[ERROR]: frame error')
            left_image = frameL
            right_image = frameR
            self.msg_header.frame_id = 'stereo_image'
            self.msg_header.stamp = rospy.Time.now()
            right_cam_info.header = self.msg_header
            self.stereo_image_info_pub.publish(right_cam_info)
            stereo_image = self.HCatImg(frameL,frameR)
            try:
                thread.start_new_thread( self.pub_image, (self.stereo_image_pub, stereo_image, self.msg_header, ))
            except:
                rospy.loginfo('Failed to start neccessary threads')
            rate.sleep()
    	cv2.destroyAllWindows()
    def HCatImg(self,frameL,frameR):
        return np.hstack((frameL, frameR))
    def list_ports(self):
        """
        Test the ports and returns a tuple with the available ports and the ones that are working.
        """
        non_working_ports = []
        dev_port = 0
        working_ports = []
        available_ports = []
        while len(non_working_ports) < 10: # if there are more than 5 non working ports stop the testing. 
            camera = cv2.VideoCapture(dev_port)
            if not camera.isOpened():
                non_working_ports.append(dev_port)
                print("Port %s is not working." %dev_port)
            else:
                is_reading, img = camera.read()
                w = camera.get(3)
                h = camera.get(4)
                if is_reading:
                    print("Port %s is working and reads images (%s x %s)" %(dev_port,h,w))
                    working_ports.append(dev_port)
                else:
                    print("Port %s for camera ( %s x %s) is present but does not reads." %(dev_port,h,w))
                    available_ports.append(dev_port)
            dev_port +=1
        return available_ports,working_ports,non_working_ports  

if __name__ == '__main__':
 
    node_name = 'StereoCamPub'
    topic = 'video_frames'
    frate= 30 #Hz
    cam_id_L = 6
    cam_id_R = 5
    if rospy.has_param('~frate'):
        frate = rospy.get_param('~frate')
    if rospy.has_param('~node_name'):
        node_name = rospy.get_param('~node_name')
    if rospy.has_param('~topic'):
        topic = rospy.get_param('~topic')
    if rospy.has_param('~cam_id_L'):
        cam_id_L = rospy.get_param('~cam_id_L')
    if rospy.has_param('~cam_id_R'):
        cam_id_R = rospy.get_param('~cam_id_R')
    SC = StereoCamera(cam_id_L, cam_id_R, frate)
    SC.run()


    
