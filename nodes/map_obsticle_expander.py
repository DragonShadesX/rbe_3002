#!/usr/bin/env python

import rospy
import roslib
import math
import sys
from rbe_3002.srv import *
from nav_msgs.msg import OccupancyGrid
from src.occupancy_grid_manipulator import *

def expand_arround_cordinate(data, x, y, width, height, expandBy):
    #print "Expanding arround %d %d ExpandBy: %d" % (x, y, expandBy)
    data[x][y] = 99
    if expandBy == 0:
        return None
    if expandBy < 0:
        print expandBy
        print "Error, Expand by below zero"
        exit()
    expandBy  -= 1
    expand_cordinate(data, x-1, y, width, height, expandBy)
    expand_cordinate(data, x+1, y, width, height, expandBy)
    expand_cordinate(data, x, y-1, width, height, expandBy)
    expand_cordinate(data, x, y+1, width, height, expandBy)

def expand_cordinate(data, x, y, width, height, expandBy):
    #print "Expanding %d %d ExpandBy: %d" % (x, y, expandBy)
    if  (x >= 0 and x < height) and (y >= 0 and y < width) and data[x][y] != 100:
        expand_arround_cordinate(data, x, y, width, height, expandBy)


def expand_obsticle_map(mapData, expansionSize):
    newGrid = OccupancyGrid()
    width = mapData.info.width
    height = mapData.info.height
    resolution = mapData.info.resolution

    newGrid.info.origin = mapData.info.origin

    newGrid.info.width = width
    newGrid.info.height = height
    newGrid.info.resolution = resolution

    dataChunked = chunk(mapData.data, width, height)
    #print dataChunked

    num_cubes_expand_by = int(math.ceil(float(expansionSize)/resolution))
    print num_cubes_expand_by

    print "Expanding"
    for x in xrange(height): #All of the height elements
        for y in xrange(width): #All of the width elements
            #print "x: %d, y: %d" % (x, y)
            if dataChunked[x][y] == 100:
                print "Expanding arround %d %d" % (x, y)
                expand_arround_cordinate(dataChunked, x, y, width, height, num_cubes_expand_by)
                #exit()
    print "Expansion Complete"
    for x in xrange(height): #All of the height elements
        for y in xrange(width): #All of the width elements
            cellData = dataChunked[x][y]
            newGrid.data.append(100 if cellData > 0 else cellData)
    print "New Data Generation Complete"

    return newGrid

def map_callback(ret):
    print ret.info
    newGrid = expand_obsticle_map(ret, (.23/2))
    global mapPublisher
    #print newGrid
    mapPublisher.publish(newGrid)


if __name__ == '__main__':
    rospy.init_node('map_obsticle_expander')
    global mapPublisher
    mapPublisher = rospy.Publisher('map_expanded', OccupancyGrid, queue_size=10)
    mapSub = rospy.Subscriber("map_scaled", OccupancyGrid, map_callback)
    rospy.spin()
