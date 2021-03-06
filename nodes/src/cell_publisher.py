#!/usr/bin/env python

import rospy
import roslib
import math
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells
from reference_transforms import *

#Subscribers and Publishers and Callbacks  -----------------------------------------------------------------------

#init subscribers and publishers



def initSubPub(mapInfo):
    global MAP_INFO
    MAP_INFO = mapInfo
    global pubRedCell
    pubRedCell = rospy.Publisher('grid_red', GridCells, queue_size=10)
    global pubOrangeCell
    pubOrangeCell = rospy.Publisher('grid_orange', GridCells, queue_size=10)
    global pubYellowCell
    pubYellowCell = rospy.Publisher('grid_yellow', GridCells, queue_size=10)
    global pubGreenCell
    pubGreenCell = rospy.Publisher('grid_green',GridCells, queue_size=10)
    global pubBlueCell
    pubBlueCell = rospy.Publisher('grid_blue', GridCells, queue_size=10)
    global messages
    messages ={'r':GridCells(), 'o':GridCells(), 'y':GridCells(), 'g':GridCells(), 'b':GridCells()}
    for key in messages:
        messages[key].cell_width = mapInfo.resolution
        messages[key].cell_height = mapInfo.resolution
        messages[key].header.frame_id = 'map'


#Publish a cell to a color topic
# int int char, chars = [ r, o, y, g, b]
def addCell(grid_x, grid_y, color):
    global MAP_INFO
    global messages
    pub_msg = messages[color]
    point = Point()
    cell = transform_grid_cells_to_map_meters((grid_x, grid_y), MAP_INFO)
    point.x = cell[0]
    point.y = cell[1]
    point.z = 0
    pub_msg.cells.append(point)

def publish(color):
    global messages
    global pubRedCell
    global pubOrangeCell
    global pubYellowCell
    global pubGreenCell
    global pubBlueCell
    pub_msg = messages[color]
    #print pub_msg
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

def clearCells():
    global messages
    global pubRedCell
    global pubOrangeCell
    global pubYellowCell
    global pubGreenCell
    global pubBlueCell
    for key in messages:
        messages[key].cells = []
    pub_msg = GridCells()
    #These can be arbatray values
    pub_msg.header.frame_id = 'map'
    pub_msg.cell_width = .2
    pub_msg.cell_height = .2
    pub_msg.cells = []

    pubRedCell.publish(pub_msg)
    pubOrangeCell.publish(pub_msg)
    pubYellowCell.publish(pub_msg)
    pubGreenCell.publish(pub_msg)
    pubBlueCell.publish(pub_msg)


## END COLOR HANDLING
