#!/usr/bin/env python

import rospy
from numpy import *
from math import *
import tf
from movement import *

driveSpeed = .2

def init_point2pont():
    global listener
    listener = tf.TransformListener()


def getLocation():
    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        try:
            return listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

# move from current position to goal x, y:
# curT must be with respect to the world with zero along the positive x axis
# this assumes rotate() is with repect to the robot
# if world frame replace argument with number in comment following rotate function
def point2point(goalX, goalY):
    global listener
    print "\n\n"
    print "Driving to (%f, %f)" % (goalX, goalY)

    (trans, rot) = getLocation()
    #euler = tf.transformations.euler_from_quaternion(rot)
    #currentAngle = euler[2]

    print "Trans"
    print trans
    print "Rot"
    print rot
    moveX = goalX - trans[0]
    moveY = goalY - trans[1]

    #This block to get rid of divide by zero errors
    targetAngle = math.atan2(moveY, moveX)
    print "targetAngle" + str(targetAngle)

    rotate_to(targetAngle)
    driveStraight(driveSpeed, (moveX**2 + moveY**2)**.5)
    return
