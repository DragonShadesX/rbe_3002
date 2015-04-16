#!/usr/bin/env python

from numpy import *
from math import *

global driveSpeed 
driveSpeed = .2

# move from current position to goal x, y:
# curT must be with respect to the world with zero along the positive x axis
# this assumes rotate() is with repect to the robot
# if world frame replace argument with number in comment following rotate function
def point2point(goalX, goalY):
    moveX = goalX - trans.x
    moveY = goalY - trans.y

    #This block to get rid of divide by zero errors
    if moveX == 0:
        moveX +=.01
    if moveY == 0:
        moveY +=.01

    rotate(-atan(moveY / moveX))
    driveStraight(driveSpeed, (moveX**2 + moveY**2)**.5)
    return



for i in waypoints:
    point2point(i[0], i[1])