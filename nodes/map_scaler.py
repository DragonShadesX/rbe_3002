#!/usr/bin/env python
import rospy
import roslib
import math
import sys
from rbe_3002.srv import *
from nav_msgs.msg import OccupancyGrid
from src.occupancy_grid_manipulator import *


#Scales the map data from map topic
def compress_map(mapData, division):
    rospy.loginfo("Compressing map by div %d", division)
    division = int(division)
    newGrid = OccupancyGrid()
    width = mapData.info.width
    height = mapData.info.height
    resolution = mapData.info.resolution

    #newGrid.header = mapData.header
    newGrid.info.origin = mapData.info.origin


    newGrid.info.width = int(width/division)
    newGrid.info.height = int(height/division)
    newGrid.info.resolution = resolution * division

    dataChunked = chunk(mapData.data, width, height)

    for x in xrange(int(height/(division))): #All of the height elements
        for y in xrange(int(width/(division))): #All of the width elements
            wallFound = False
            nullSpaceFound = False
            for divx in xrange(division):
                for divy in xrange(division):
                    try:
                        indexx= int(division*x)+divx
                        indexy = int(division*y)+divy
                        value = dataChunked[indexx] [indexy]
                    except:
                        rospy.logfatal('x: %d divx: %d indexx: %d y: %d divy: %d indexy: %d', x, divx, indexx, y, divy, indexy)
                        raise
                    if value == 100:
                        wallFound = True
                    elif value == -1:
                        nullSpaceFound = True
            if wallFound: # If any walls were found then make this a wall
                newGrid.data.append(100)
            elif nullSpaceFound: # If any unknown space is found then make the whole cell unknown
                newGrid.data.append(-1)
            else: # We know what is here
                newGrid.data.append(0)
    #print newGrid
    return newGrid


def map_callback(ret):
    print ret.info

    #Scales the map relative to the width of the robot
    scaleBy = (.23/2)/ret.info.resolution
    scaleBy = int(math.ceil(scaleBy))
    newGrid = compress_map(ret, scaleBy)
    global mapPublisher
    mapPublisher.publish(newGrid)
    print newGrid




if __name__ == '__main__':
    rospy.init_node('map_scaler')
    global mapPublisher
    mapPublisher = rospy.Publisher('map_scaled', OccupancyGrid, queue_size=10)
    mapSub = rospy.Subscriber("map", OccupancyGrid, map_callback)
    rospy.spin()
