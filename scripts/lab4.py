#!/usr/bin/env python
import rospy
import roslib
import math
import sys
from rbe_3002.srv import *

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
    print "WARNING: THESE VALUES ARE RELATIVE TO THE GRID, NOT THE ROBOT, I STILL NEED TO APPLY A REFERENC FRAME TRANSFORM."
    print "I'll do this inside of the a_star_server so that it returns the value in the correct reference frame"
    pathResponse = s(req)
    path = zip(pathResponse.pathx, pathResponse.pathy)
    print path

    #Then the first and last values returned by the path should be the start and goal repspectively



if __name__ == '__main__':
    if len(sys.argv) > 1:
        delay = int(sys.argv[1])
        main(delay)
    else:
        main(0)
