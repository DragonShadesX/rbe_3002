
import rospy
import roslib
import math
import sys
from src.occupancy_grid_manipulator import *
''' Takes in the map input and finds the center of all of the fronteers '''

NAME = "map_frontier"
MAP_TOPIC = "map_expanded"


def map_callback(ret):
    print ret
    map_data = Map(ret)



def main():
    rospy.init_node(NAME)
    mapSub = rospy.Subscriber(MAP_TOPIC, OccupancyGrid, map_callback)


if __name__ == '__main__':
    main()
