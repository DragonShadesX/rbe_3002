#!/usr/bin/env python
## BEGIN CELL COLOR SETTING
#imports  --------------------------------------------------------------------------------------------------------
import rospy
import roslib
import math
import tf

from src.a_star_logic import *
from src.cell_publisher import *
from src.reference_transforms import *

## BEGIN SERVER HANDLING

from rbe_3002.srv import *
from rbe_3002 import *
import rospy
from nav_msgs.msg import OccupancyGrid

MAP_TOPIC = "map_expanded"

def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

def map_callback(ret):
    global map_info
    initSubPub(ret.info)
    map_info = ret.info
    width = ret.info.width
    height = ret.info.height

    gridData = GridWithWeights(width, height)
    gridData.walls = []
    #gridData.weights #If weorigin to use weights here we can
    i = 0 # Element in the list
    for y in xrange(height):
        for x in xrange(width):
            if ret.data[i] == 100:
                #print "100 Found"
                #addCell(x, y, 'o')
                gridData.walls.append((x, y))
            i +=1
    #publish('o')

    #print gridData.wallsorientation
    global gridDataGlobal
    gridDataGlobal = gridData
    global s
    global initialized
    if not initialized:
        initialized = True
        s = rospy.Service('a_star', AStar, a_star)


def a_star(req):
    rospy.loginfo("Request for a_star")
    clearCells()
    rospy.sleep(1)

    global gridDataGlobal
    global map_info

    print req
    startCell = transform_map_meters_to_grid_cells(req.startPoint, map_info)
    targetCell = transform_map_meters_to_grid_cells(req.targetPoint, map_info)
    addCell(startCell[0], startCell[1], 'b')
    publish('b')
    addCell(targetCell[0], targetCell[1], 'r')
    publish('r')

    #TODO: Check for invalid start/end positions. In wall, ect

    came_from, cost_so_far = a_star_search(gridDataGlobal, startCell, targetCell)
    #draw_grid(gridDataGlobal, width=3, point_to=came_from, start=req.startPoint, goal= req.targetPoint)
    path = reconstruct_path(came_from, startCell, targetCell)
    path = tuple(reversed(path))
    newPath = []
    for cell in path:
        newPath.append(transform_grid_cells_to_map_meters(cell, map_info))
    newPath = tuple(newPath)
    pathx, pathy = zip(*newPath)
    print 'Path:'
    print newPath
    # waypoints = []
    # lastx = [None, None]
    # lasty = [None, None]
    # for (x, y) in path:
    #     if lastx[1] != None and lasty[1] != None:
    #         # Calculate the distance between this point and two behind
    #         dis =  distance([x,y],[lastx[1], lasty[1]])
    #         #If the distance is 2 then we are on a straight section however if it is less then the last point is a waypoint
    #         if dis < 1.87:
    #             waypoints.append((x,y))
    #             addCell(lastx[0],lasty[0],'b')
    #     lastx[1] = lastx[0]
    #     lasty[1] = lasty[0]
    #     lastx[0] = x
    #     lasty[0] = y
    # publish('b')
    #
    # rospy.sleep(3)
    # clearCells()
    #rospy.sleep(3)
    # for(x,y) in path:
    #     addCell(x,y,'r')
    # publish('r')
    # addCell(path[0][0], path[0][1],'b');
    # publish('b')

    #print pathx
    #print pathy
    return AStarResponse(pathx, pathy)

def a_star_server():
    global initialized
    initialized = False
    #global mapSub
    rospy.logdebug("Running A* Server")
    rospy.init_node('a_star_server')
    mapSub = rospy.Subscriber(MAP_TOPIC, OccupancyGrid, map_callback)
    rospy.spin()

if __name__ == '__main__':
    #tset = GridWithWeights(1, 1)
    a_star_server()
