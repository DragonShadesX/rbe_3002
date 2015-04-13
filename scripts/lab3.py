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
    startPoint = (150, 100)
    goalPoint = (190, 140)

    # When we make a request to this service
    req = AStarRequest(startPoint, goalPoint)
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
