#!/usr/bin/env python

from rbe_3002.srv import *
import rospy



def a_star(req):
    print "Passed arguments\n"
    print req
    return req

def a_star_server():
    print "Running A* Server"
    rospy.init_node('a_star_server')
    s = rospy.Service('a_star', AStar, a_star)
    rospy.spin()

if __name__ == '__main__':
    a_star_server()
