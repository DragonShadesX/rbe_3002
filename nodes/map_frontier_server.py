#!/usr/bin/env python

import rospy
import roslib
import math
import sys
from geometry_msgs.msg import Point
from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid
from src.occupancy_grid_manipulator import *
from src.OccupancyMap_Object import *
from src.reference_transforms import transform_grid_cells_to_map_meters
''' Takes in the map input and finds the center of all of the fronteers '''

NAME = "map_frontier"
MAP_TOPIC = "map_expanded"
FRONTIER_PUB_TOPIC = 'grid_frontier'

def map_callback(ret):
    rospy.loginfo("Map Callback")
    #print ret
    map_data = OccupancyMap(ret)
    centroids = map_data.findFrontiers()

    # Generate all grid cells info
    gridCells = GridCells()
    gridCells.cell_width = ret.info.resolution
    gridCells.cell_height = ret.info.resolution
    gridCells.header.frame_id = 'map'
    gridCells.header.stamp = rospy.Time.now()

    #Generate all of the point data
    for centroid in centroids:
        point = Point()
        cell = transform_grid_cells_to_map_meters((centroid[0], centroid[1]), ret.info)
        point.x = cell[0]
        point.y = cell[1]
        point.z = 0
        gridCells.cells.append(point)

    # Publish this data
    global frontierPub
    rospy.loginfo("Publishing frontier")
    frontierPub.publish(gridCells)



def main():
    rospy.init_node(NAME)
    rospy.sleep(3)
    mapSub = rospy.Subscriber(MAP_TOPIC, OccupancyGrid, map_callback)
    global frontierPub
    frontierPub = rospy.Publisher(FRONTIER_PUB_TOPIC, GridCells, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
