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
            nullSpaceFound = True
            for divx in xrange(division):
                for divy in xrange(division):
                    try:
                        indexx= int(division*x)+divx
                        indexy = int(division*y)+divy
                        value = dataChunked[indexx] [indexy]
                    except:
                        print 'x: %d divx: %d indexx: %d y: %d divy: %d indexy: %d' % (x, divx, indexx, y, divy, indexy)
                        raise
                    if value == 100:
                        wallFound = True
                    elif value == 0:
                        nullSpaceFound = False
            if wallFound:
                newGrid.data.append(100)
            elif nullSpaceFound:
                newGrid.data.append(-1)
            else:
                newGrid.data.append(0)
    #print newGrid
    return newGrid


def map_callback(ret):
    print ret.info
    newGrid = compress_map(ret, 6)
    global mapPublisher
    mapPublisher.publish(newGrid)
    print newGrid




if __name__ == '__main__':
    rospy.init_node('map_scaler')
    global mapPublisher
    mapPublisher = rospy.Publisher('map_scaled', OccupancyGrid, queue_size=10)
    mapSub = rospy.Subscriber("map", OccupancyGrid, map_callback)
    rospy.spin()
