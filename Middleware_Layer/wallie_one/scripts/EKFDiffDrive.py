#!/usr/bin/env python

import numpy as np
import math
import cv2
import rospy
from nav_msgs.msg import Odometry
from create_msgs.msg import Bumper
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler

class DiffDriveEKF:
    def __init__(self, n, init_state = np.asarray([[0],[0],[0]]), d = 0.235):
        """
        Inputs:
        n: dimension of state vector (3)
        d: axle length of diff drive model
        """
        #Axle Length (distance between wheels) meters
        self.d = d
        self.n = n
        self.u = np.zeros((2,1))

        #Prediction state vector [x, y, theta]
        self.xP = np.zeros((n,1))
        #Estimated state vector [x, y, theta]
        self.xE = init_state
        #State transition matrix for position
        self.F = np.eye(n)
        self.Q = np.diag([0.2,0.2,0.05])
        self.Pe = np.eye(n)
        self.R = np.diag([.2**2,.22**2,((.22**2-.2**2)/self.d)/np.pi*180])
        self.H = np.eye(n)
    def f(self):
        """
        Input: u [distanceLeftWheel, distanceRightWheel]
        
        """
        self.ds = (self.u[1] + self.u[0])/2
        self.dtheta = (self.u[1] + self.u[0])/self.d
        theta = self.xE[2]
        self.xP = self.xE + np.matmul(np.array([[np.cos(self.xE[2] + self.dtheta/2), 0],[np.sin(self.xE[2] + self.dtheta/2), 0],[0,dt]]), np.array([[self.ds],[self.dtheta]]))
        
    def Fp(self):
        #Jacobian matrix for position as function of wheel travel
        self.F[0,0] = np.cos(self.xE[2]+self.dtheta/2)/2 - self.ds/2*np.cos(self.xE[2]+self.dtheta/2)
        self.F[0,1] = np.cos(self.xE[2]+self.dtheta/2)/2 + self.ds/2*np.sin(self.xE[2]+self.dtheta/2)
        self.F[1,0] = np.sin(self.xE[2]+self.dtheta/2)/2 + self.ds/2*np.cos(self.xE[2]+self.dtheta/2)
        self.F[1,1] = np.sin(self.xE[2]+self.dtheta/2)/2 - self.ds/2*np.cos(self.xE[2]+self.dtheta/2)
        self.F[2,0] = 1/self.d
        self.F[2,1] = -1/self.d
        self.F = np.matmul(self.F, self.u)
    def tick(self,u):
        self.u = u
        self.predictUpdate()
        return self.xE
    def predictUpdate(self):
        self.f()
        self.Fp()
        self.Pp = np.dot(np.dot(self.F,self.Pe),self.F.T) + self.Q
        self.S = self.Pp + self.R
        self.K = np.dot(self.Pp, np.linalg.inv(self.S))
        #Update state vector estimate using kalman gain
        self.xE = self.xP + np.dot(self.K, self.z-self.xP)
        
        #Update Pp covariance matrix
        self.Pp = np.dot((np.eye(self.n)-np.dot(self.K,self.H)),self.Pp)

class SensorFusion(object):
    def __init__(self, numSensors):
        self.numSensors = numSensors
        self.weights = np.ones((numSensors,1))*(1/numSensors)
    def Fuse(self, sensor_readings):
        for n in range(0,self.numSensors):
            self.fusion += sensor_readings[:,n]
        return self.fusion



class StateController(object):
    def __init__(self):
	print("Starting wheel encoder subscriber")
        self.subWheelEncoders = rospy.Subscriber("joint_states", JointStat, self.JointStateCB)
        self.subZEDOdometry = rospy.Subscriber("zed2/zed_node/odom",self.ZEDOdometryCB)
        self.SensorFusion = SensorFusion(2)
        self.u = np.zeros((2,1))
        self.KF = DiffDriveEKF(3)
        self.xE = np.zeros((3,1))
    def JointStateCB(self,msg):
        self.u[0] = msg.position[0]
        self.u[1] = msg.position[1]
        self.xE = self.KF.tick(self.u)
        #self.xE = self.SensorFusion.Fuse(np.hstack(self.xE,self.zedxE))
    def ZEDOdometryCB(self,msg):
        pass
def main():
    rospy.init_node('EKFDiffDrive', anonymous=True)
    StateControl = StateController()
    rospy.spin()
if __name__ == "__main__":
    main()
