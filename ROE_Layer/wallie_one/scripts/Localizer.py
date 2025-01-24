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

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class EKFDiffDrive:
	def __init__(self, n, dt, init_state = np.zeros((3,1)), d = 0.235, N = 10):
		"""
		Inputs:
		n: dimension of state vector (3)
		d: axle length of diff drive model
		N: dimension of range measurement
		"""
		#Axle Length (distance between wheels) meters
		self.d = d
		self.n = n
		self.u = np.zeros((2,1))
		self.dt = dt
		#Prediction state vector [x, y, theta]
		self.xP = np.zeros((n+2*N,1))
		#Estimated state vector [x, y, theta]
		self.xE = self.xP
		self.N = N
		#System state transition model\
		#Map to higher dim
		#Main state transition predict model
		self.A = np.zeros((3,3))
		#List of currently observed features and list of all feautures
		self.features = dict()
		self.observedFeatures = dict()
		#Input state transition model
		self.B = np.zeros((3,2))
		self.Q = np.zeros((n+2*N,n+2*N))
		self.Q[0][0] = 1e-3
		self.Q[1][1] = 1e-3
		self.Q[2][2] = 7.62e-6
		self.Pe = self.Q
		self.Pp = np.eye(3)
		self.R = np.diagflat([[.2**2,.22**2,((.24**2-.1**2)/self.d)/np.pi*180]])
		self.H = np.eye(n)
		self.inCov = 1e-4
		self.convergance = deque(maxlen = int(np.floor(1/self.dt)*30))
	def f(self):
		"""
		Good

		"""
		self.F = np.zeros((3,2*self.N+self.n))
		self.F[0][0] = 1
		self.F[1][1] = 1
		self.F[2][2] = 1
		self.StateMatrix = np.array([[self.ds*np.cos(self.xE[2][0] + self.dtheta/2)],[self.ds*np.sin(self.xE[2][0] + self.dtheta/2)],[self.dtheta*self.dt]])
		self.xP = self.xE + np.matmul(self.F.T, self.StateMatrix)

	def Fp(self):
		"""


		"""
		#Buils input and system matrices - > Linearized through Jacobian
		theta = self.xE[2][0]
		thetaIn = theta+self.dt*self.dtheta/2
		self.B[0][0] = np.cos(thetaIn)/2 - self.ds/2*np.cos(thetaIn)
		self.B[0][1] = np.cos(thetaIn)/2 + self.ds/2*np.sin(thetaIn)
		self.B[1][0] = np.sin(thetaIn)/2 + self.ds/2*np.cos(thetaIn)
		self.B[1][1] = np.sin(thetaIn)/2 - self.ds/2*np.cos(thetaIn)
		self.B[2][0] = 1/self.d
		self.B[2][1] = -1/self.d
		#Jacobian matrix for system model
		#Only need to update non-zero non identity elements
		self.A[0][2] = -self.ds*np.sin(thetaIn)
		self.A[1][2] = self.ds*np.sin(thetaIn)
		#Transform to higher dimension
		self.G = np.matmul(np.matmul(self.F.T, self.A),self.F)
		self.G = self.G + np.eye(self.G.shape[0]);print(self.G.shape)
	def tick(self, u, wheelReadings, z):
		"""
		Function is called process control input and measurement

		"""
		self.u = u
		self.ds = wheelReadings[0]
		self.dtheta = wheelReadings[1]
		self.N = len(z)
		for feature in z:
			self.addLandmark(feature)

		print("Vf {d}".format(d=self.ds))
		#Iteratively calculate optimum covaraince matrix
		#Trace should be minimized, uncorreletated
		#self.optimize_inputCovariance()
		self.time_update()
		self.meas_update()
		return self.xE
	def time_update(self):
		"""


		"""
		#Apply system model to get prediction
		self.f()
		#Calculate system and input jacobians
		self.Fp()
		system = np.matmul(np.matmul(self.G ,self.Pe), self.G.T)
		#meas = np.matmul(self.inCov, np.matmul(self.B,self.B.T))
		print(self.F.T.shape)
		print(self.R.shape)
		# 9x3 * 3x3 * 3x9
		self.Pp = system + np.matmul(np.matmul(self.F.T ,self.R), self.F)


	def addLandmark(self, landmarks):
		for landmark in landmarks:
			#Unpack data from sensor readings
			#Add to observed features
			self.observedFeatures[landmark.i] = landmark

	def meas_update(self):
		"""


		"""
		self.N = len(self.features.keys())
		# K = P*H'*inv(H*P*H' + R)
		i = 0
		xP = self.xP
		Ptemp = self.Pp
		for key in self.observedFeatures.keys():
			landmark = self.observedFeatures[key]
			if key not in self.features.keys():
				x = self.xP[0][0]
				y = self.xP[1][0]
				theta = self.xP[2][0]
				#Get location of landmark based on measurement
				ujx = x + landmark.r * np.cos(landmark.a + theta)
				ujy = y + landmark.r * np.sin(landmark.a + theta)
				#Compute difference between landmark location and current location
				d = np.array([ujx-x, ujy-y]).T
				dx = d[0][0]
				dy = d[1][0]
				#Add feature to dict for later use
				self.features[key] = np.array([[ujx],[ujy]])
				q = np.matmul(d.T, d)
				#Observation
				zHat = np.array([math.sqrt(q), np.arctan2(dy, dx) - theta]).T
				sqrtQ = np.sqrt(q)
				#Fill in jacobian and map to higher dimensional space
				F = np.hstack(np.hstack(np.vstack(np.eye(3), np.zeros((2,3))), np.zeros((5,2*landmark.i-2))), np.hstack(np.array([[0,0,0,1,0],[0,0,0,0,1]]).T,np.zeros((5,2*self.N-2*landmark.i))))
				H = np.matmul(1/q*np.array([[-sqrtQ*dx, -sqrtQ*dy, 0, sqrtQ*dx, sqrtQ*dy],[dy, -dx, q, -dy, dx]]),F)

				temp = np.matmul(np.matmul(H,self.Pp), H.T) + self.Q.T
				K = np.matmul(np.matmul(self.Pp, H.T), temp)
				innov = landmark.pos - zHat
				self.xP = self.xP + np.matmul(K,innov)
				Ptemp = np.matmul(np.eye(2) - np.matmul(K, H), Ptemp)
				t = LA.cond(K)
				if t > 100:
					print("WARNING: Condition of kalman gain matrix is greater than 100. Cond(K) = {d}.\n".format(d = t))
		self.xE = self.xP
		self.Pe = Ptemp
		"""
		self.S = np.matmul(np.matmul(self.H, self.Pp), self.H.T) + self.R
		self.K = np.matmul(np.matmul(self.Pp, self.H), np.linalg.inv(self.S))
		#Update state vector estimate using kalman gain
		inov = self.z - self.xP
		self.xE = self.xP + np.matmul(self.K, inov)
		#Update Pe covariance matrix
		# P = (I-K*H)*P*(I-K*H)'+K*R*K'
		IK = (np.eye(self.n)-np.matmul(self.K,self.H))
		self.Pe = np.matmul(np.matmul(IK, self.Pp), IK.T) + np.matmul(np.matmul(self.K, self.R), self.K.T)
		"""

		t = LA.det(self.Pe)
		if t <0:
			print("WARNING: Posterior Covariance matrix is not positive-definite. Trace = {n}.\n".format(n = t))
		#Push new posterior covaraiance trace to queue
		#Use to check for convergance
		self.convergance.appendleft(t)

	def optimize_inputCovariance(self):
		c = 0
		maxSweep = 100
		minSweep = -100
		sweepRes = 0.01
		sweepTotal = (minSweep - maxSweep)/sweepRes
		res = np.zeros([1,sweepTotal])
		inCov = []
		for i in range(minSweep, maxSweep, sweepRes):
			self.inCov = i
			self.time_update()
			self.meas_update()
			inCov.append(self.inCov)
			res[c] = np.abs(np.linalg.trace(self.Pe))
			c += 1
		ri = res.argmin()
		self.inCov = inCov[ri]

	def normalize_angle(self, a):
		"""
		Wrap the angle between 0 and 2 * pi.

		Args:
		angle (float): angle to wrap.

		Returns:
		 The wrapped angle.

		"""
		return np.arctan2(np.sin(a),np.cos(a))



	
class Roomba:
	def __init__(self,r,d = 0.235):
		print("Starting wheel encoder subscriber")
		self.subWheelEncoders = rospy.Subscriber("joint_states", JointState, self.JointStateCB)
		self.subZEDOdometry = rospy.Subscriber("zed2/zed_node/pose",PoseStamped,self.ZEDOdometryCB)
		self.subWheelEncoders = rospy.Subscriber("odom", Odometry, self.RoombaOdomCB)
		self.pubCmdVel = rospy.Publisher("cmd_vel", Twist, queue_size = 1)
		#self.SensorFusion = SensorFusion(2)
		self.u = np.zeros((2,1))
		self.KF = EKFDiffDrive(3,r)
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
			print("Measured wheel velocities: ")
			print(msg.velocity[0], msg.velocity[1])
			self.wheelDists = np.array([msg.position[0], msg.position[1]])/100-self.lastWheelDists
			print("Measured wheel distances(m): ")
			print(self.wheelDists)
			self.xD = self.applyMotionModel(self.wheelDists[0],self.wheelDists[1])
			self.xE = self.KF.tick(self.u,self.stateRoomba)
			print("xD: ")		
			print(self.xD)
			print("xE: ")
			print(self.xE)
			print("ZED State: ")
			print(self.stateZed)
			print("Roomba Odom State: ")
			print(self.stateRoomba)
			#self.xE = self.SensorFusion.Fuse(np.hstack(self.xE,self.zedxE))
		else:
			self.wheelDists = np.array([msg.velocity[0], msg.velocity[1]])/100

		self.lastWheelDists = np.array([msg.velocity[0], msg.velocity[1]])/100
		if self.index >= 2:
			self.lastDataAvailable = True
		self.index += 1
		
		
	def applyMotionModel(self,dr,dl):
		#Dead reckoning from wheel encoders
		self.ds = self.VfNeg * (dr + dl)/2
		print("Vf {d}".format(d=self.ds))
		self.dtheta = self.normalize_angle(self.VaNeg *(dr - dl)/self.d)
		print("dTheta {d}".format(d=self.dtheta))
		theta_last = self.xD[2][0]
		theta_last = self.normalize_angle(theta_last)
		print("Theta {d}".format(d=theta_last))
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
		robot.sendCmdVel(0.1,0)
		xE = robot.getxE()
		rate.sleep()
	
if __name__ == "__main__":
	main()
