#!/usr/bin/env python
#imports  --------------------------------------------------------------------------------------------------------
import rospy
import roslib
import math
import tf
import time

from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells

#Subscribers and Publishers and Callbacks  -----------------------------------------------------------------------

#init subscribers and publishers
def initSubPub():
	pubRedCell = rospy.Publisher('grid_red', nav_msgs/GridCells)
	pubOrnageCell = rospy.Publisher('grid_ornage', nav_msgs/GridCells)
	pubYellowCell = rospy.Publisher('grid_yellow', nav_msgs/GridCells)
	pubGreenCell = rospy.Publisher('grid_green', nav_msgs/GridCells)
	pubBlueCell = rospy.Publisher('grid_blue', nav_msgs/GridCells)


#Publish a cell to a color topic
# int int char, chars = [ r, o, y, g, b]
def publsihGreenCell(grid_x, grid_y, color): 
	pub_msg = nav_msgs/GridCells
	pub_msg.frame_id = '/map'
	pub_msg.cell_width = .2
	pub_msg.cell_height = .2
	pub_msg.cells.x = grid_x
	pub_msg.cells.y = grid.y
	pub_msg.cells.z = 0

	if color == 'r':
		pubRedCell.publish(pub_msg)
	if color == 'o':
		pubOrangeCell.publish(pub_msg)
	if color == 'y':
		pubYellowCell.publish(pub_msg)
	if color == 'g':
		pubGreenCell.publish(pub_msg)
	if color == 'b':
		pubBlueCell.publish(pub_msg)
