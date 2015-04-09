#!/usr/bin/env python
## BEGIN CELL COLOR SETTING
#imports  --------------------------------------------------------------------------------------------------------
import rospy
import roslib
import math
import tf

from src.a_star_logic import *
from src.cell_publisher import *

## BEGIN SERVER HANDLING

from rbe_3002.srv import *
from rbe_3002 import *
import rospy
from nav_msgs.msg import OccupancyGrid

def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

def map_callback(ret):
    #print ret
    width = ret.info.width
    height = ret.info.height
    gridData = GridWithWeights(width, height)
    gridData.walls = []
    #gridData.weights #If we ever choose to use weights here we can
    i = 0 # Element in the list
    for x in xrange(width):
        for y in xrange(height):
            if ret.data[i] == 100:
                gridData.walls.append((x, y))
            i += 1
    #print gridData.walls
    global gridDataGlobal
    gridDataGlobal = gridData

def a_star(req):
    global loaded
    rospy.loginfo("Request for a_star")
    mapSub = rospy.Subscriber("map", OccupancyGrid, map_callback)
    while not loaded and not rospy.is_shutdown():
        rospy.sleep(.01)
        if not 'gridDataGlobal' in globals():
            rospy.sleep(.1)
            break

    loaded = True
    global gridDataGlobal

    print req
    came_from, cost_so_far = a_star_search(gridDataGlobal, req.startPoint, req.targetPoint)
    #draw_grid(gridDataGlobal, width=3, point_to=came_from, start=req.startPoint, goal= req.targetPoint)
    path = reconstruct_path(came_from, req.startPoint, req.targetPoint)
    path = tuple(path)
    pathx, pathy = zip(*path)
    print 'Path:'
    print path
    waypoints = []
    lastx = [None, None]
    lasty = [None, None]
    for (x, y) in path:
        if lastx[1] != None and lasty[1] != None:
            # Calculate the distance between this point and two behind
            dis =  distance([x,y],[lastx[1], lasty[1]])
            #If the distance is 2 then we are on a straight section however if it is less then the last point is a waypoint
            if dis < 1.87:
                waypoints.append((x,y))
                addCell(lastx[0],lasty[0],'b')
        lastx[1] = lastx[0]
        lasty[1] = lasty[0]
        lastx[0] = x
        lasty[0] = y
    publish('b')

    rospy.sleep(3)
    clearCells()
    rospy.sleep(3)
    for(x,y) in path:
        addCell(x,y,'r')
    publish('r')

    #print pathx
    #print pathy
    return AStarResponse(pathx, pathy)

def a_star_server():
    global loaded
    loaded = False
    initSubPub()
    #global mapSub
    rospy.logdebug("Running A* Server")
    #mapSub = rospy.Subscriber("map", OccupancyGrid, map_callback)
    rospy.init_node('a_star_server')
    s = rospy.Service('a_star', AStar, a_star)
    rospy.spin()

if __name__ == '__main__':
    tset = GridWithWeights(1, 1)
    a_star_server()
