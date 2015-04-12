#!/usr/bin/env python


global driveSpeed 
driveSpeed = .2

global curT

# move from to 
# current x, y;  goal x, y;  unit scaling factor
# curT must be with respect to the world with zero along the positive x axis
# this assumes rotate() is with repect to the robot
# if world frame replace argument with number in comment following rotate function
def point2point(curX, curY, goalX, goalY, unit_size):
    moveX = goalX - curX 
    moveY = goalY - curY
    if moveX != 0 and moveY != 0:
        raise NameError('Diagonal Move Attempted, ignoring...')
        break
    if moveX > 0:
        rotate(-curT)  # 0
        driveStraight(driveSpeed, moveX*unit_size)
        curT = 0
    if moveX < 0:
        rotate(3.14 - curT)  # 3.14
        driveStraight(driveSpeed, abs(moveX*unit_size))
        curT = 3.14
    if moveY > 0:
        rotate(-1.57 - curT)  # -1.57
        driveStraight(driveSpeed, moveY*unit_size)
        curT = -1.57
    if moveY < 0:
        rotate(1.57 - curT)  # 1.57
        driveStraight(driveSpeed, abs(moveY*unit_size))
        curT = 1.57


