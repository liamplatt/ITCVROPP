#!/usr/bin/env python
#!/usr/bin/env python
import numpy as np
import math
import socket
import cv2
import rospy
import tf
import tf2_ros
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import roslib
from std_msgs.msg import String, Header
from cStringIO import StringIO


from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


TwistMsg = Twist

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.br = tf.TransformBroadcaster()
		self.x = 0
		self.y = 0
		self.z = 0
		self.roll = 0
		self.pitch = 0
		self.yaw = 0
		self.cmd_vel = Twist()
		self.click = Bool()
		self.timer = rospy.Timer(rospy.Duration(1),self.callback);
		self.op = rospy.get_param('/zed_stereo_node/op_control', 0)
		self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_callback, queue_size = 1)
		self.click_pub = rospy.Publisher("/pc_click", Bool, queue_size=1)
		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None
 
    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist_msg = TwistMsg()

        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg
        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist_msg)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist_msg)
	def joy_callback(self, msg):
		self.x = -msg.axes[0]
		self.y = -msg.axes[1]
		self.cmd_vel.angular.z = msg.axes[0]*np.pi/4
		self.cmd_vel.linear.x = msg.axes[1]/4
		if msg.buttons[2]:
			self.click.data = 1
			self.click_pub.publish(self.click)
		self.op = rospy.get_param('/zed_stereo_node/op_control', 0)
		if msg.buttons[1]:
			self.op = not self.op

		if msg.buttons[3]:
			self.x = 0
			self.y = 0
			self.roll = 0
			self.z = 0
			self.pitch = 0
			self.yaw  = 0
		rospy.set_param('/zed_stereo_node/op_control', self.op)
		self.op = rospy.get_param('/zed_stereo_node/op_control', 0)
		
		if not self.op:
			self.x = 0
			self.y = 0
			self.z = 0
			self.roll = 0
			self.pitch = 0
			self.yaw  = 0
			self.cmd_vel_pub.publish(self.cmd_vel)
		if msg.buttons[5]:
			self.roll = self.roll + np.pi/16
		if msg.buttons[6]:
			self.roll = self.roll - np.pi/16
			
		if msg.buttons[10]:
			self.pitch = self.pitch + np.pi/16
		if msg.buttons[9]:
			self.pitch = self.pitch - np.pi/16
		
		self.yaw = msg.axes[2]*2*np.pi
		self.br.sendTransform((self.x, self.y, self.z), tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw), rospy.Time.now(), "hmd_imu_frame", "zed_left_camera_optical_frame")
		self.br.sendTransform((self.x, self.y, self.z), tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw), rospy.Time.now(), "hmd_imu_frame", "zed_right_camera_optical_frame")

def getKey(settings, timeout):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('tf_broadcat_keyboard')

    if stamped:
        TwistMsg = TwistStamped

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = min(speed_limit, speed * speedBindings[key][0])
                turn = min(turn_limit, turn * speedBindings[key][1])
                if speed == speed_limit:
                    print("Linear speed limit reached!")
                if turn == turn_limit:
                    print("Angular speed limit reached!")
                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)

