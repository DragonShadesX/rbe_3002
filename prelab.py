#!/usr/bin/env python
#imports  --------------------------------------------------------------------------------------------------------
import rospy
import roslib
import math
import tf
import time

from geometry_msgs.msg import map
from geometry_msgs.msg import MapMetaData
from geometry_msgs.msg import OccupancyGrid
from geometry_msgs.msg import pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D
from kobuki_msgs.msg import BumperEvent
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import degrees


#Subscribers and Publishers and Callbacks  -----------------------------------------------------------------------


def initSubscribers():
	rospy.Subscriber("nav_msgs/OccupancyGrid", ????, OccupancyGridCallBack)

	rospy.Subscriber("odom", Odometry, OdomCallBack)

	rospy.Subscriber("nav_msgs/map", map, MapCallBack)

#callback get pos and heading(orientation?) from bot and sets them as global vars
def OdomCallBack(data):

	#pull data from the message and modify to make it easier to use
	px = data.pose.pose.position.x
	py = data.pose.pose.position.y
	quat = data.pose.pose.orientation
	q = [quat.x, quat.y, quat.z, quat.w]
	roll, pitch, yaw = euler_from_quaternion(q)

	#set what is useful to us as global variables to be used everywhere
	global x
	x = px
	global y
	y = py
	global theta
	theta = yaw


#Code  -----------------------------------------------------------------------------------------------------------


#Main  -----------------------------------------------------------------------------------------------------------


if __name__ == '__main__':
	rospy.init_node('lab3.py')

	initSubscribers()
