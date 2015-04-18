#!/usr/bin/env python
import rospy
import roslib
import math
import sys
from rbe_3002.srv import *
from src.path_manipulation import *
from src.point2point_move_function import *
from src.movement import *
from geometry_msgs.msg import PointStamped


def PoseCallBack(ret):
    global goal
    goal = (ret.point.x, ret.point.y)


def getAStarWaypoints(a_star_service):
    # Cordinates are relative to the origin of the map in meters
    (trans, rot) = getLocation()
    startPoint = (trans[0], trans[1])
    goalPoint = goal

    # When we make a request to this service
    req = AStarRequest(startPoint, goalPoint)
    pathResponse = a_star_service(req)
    #Round all of the values to something reasonable
    pathx = [ round(elem, 3) for elem in pathResponse.pathx ]
    pathy = [ round(elem, 3) for elem in pathResponse.pathy ]
    path = zip(pathx, pathy)
    print "Path"
    print path

    waypoints = path_to_waypoints(path)
    print "Waypoints"
    print waypoints
    return waypoints

def main(delay):
    #move_base_symple
    rospy.init_node('lab4')
    initMovement()
    init_point2pont()
    rospy.sleep(delay)
    rospy.wait_for_service('a_star')
    a_star_service = rospy.ServiceProxy('a_star', AStar)
    print "Waiting for click point"
    click_sub = rospy.Subscriber("clicked_point", PointStamped, PoseCallBack)
    rospy.wait_for_message("clicked_point", PointStamped)
    #rotate(math.pi)
    #rotate(math.pi)

    waypoints = getAStarWaypoints(a_star_service)


    while len(waypoints) != 0:
        point2point(waypoints[1][0], waypoints[1][1])
        waypoints = getAStarWaypoints(a_star_service)


    #Then the first and last values returned by the path should be the start and goal repspectively



if __name__ == '__main__':
    if len(sys.argv) > 1:
        delay = int(sys.argv[1])
        main(delay)
    else:
        main(0)
