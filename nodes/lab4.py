#!/usr/bin/env python
import rospy
import roslib
import math
import sys
from rbe_3002.srv import *
from src.path_manipulation import *

def main(delay):
    rospy.sleep(delay)
    rospy.loginfo("Starting Unit Test")
    rospy.wait_for_service('a_star')
    s = rospy.ServiceProxy('a_star', AStar)
    # Cordinates are relative to the origin of the map in meters
    startPoint = (-3, 7)
    goalPoint = (2, 7)

    # When we make a request to this service
    req = AStarRequest(startPoint, goalPoint)
    pathResponse = s(req)
    #Round all of the values to something reasonable
    pathx = [ round(elem, 3) for elem in pathResponse.pathx ]
    pathy = [ round(elem, 3) for elem in pathResponse.pathy ]
    path = zip(pathx, pathy)
    print "Path"
    print path

    waypoints = path_to_waypoints(path)
    print "Waypoints"
    print waypoints

    #Then the first and last values returned by the path should be the start and goal repspectively



if __name__ == '__main__':
    if len(sys.argv) > 1:
        delay = int(sys.argv[1])
        main(delay)
    else:
        main(0)
