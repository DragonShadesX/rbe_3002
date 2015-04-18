#!/usr/bin/env python

import rospy, tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D
import math
import time


def getTime():
    return rospy.get_rostime().secs

def mapRadians(value):
    while(value > math.pi):
        value = value - (2 * math.pi)
    while(value < -math.pi):
        value = value + (2* math.pi)

    return value

def publishTwist (u, w):
    global teleop_pub
    twist = Twist()
    twist.linear.x = u; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = w
    teleop_pub.publish(twist)

def calcualteForwardVelocity(u1, u2):
    #calc forward velocity
    u =  (r/2) * (u1 + u2) #math here #calculate the forward velocity
    w =  (r/base) * (u1 - u2) #math here #calculate the angular velocity
    return {'u': u, 'w':w}


#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    global r
    global base

    #calc forward velocity
    velocities = calcualteForwardVelocity(u1, u2);
    u =  velocities['u']
    w =  velocities['w']
    start = getTime() # time at which the robot started moving

    #publish twist until it is time to stop
    while((getTime() - start < time) and (not rospy.is_shutdown())):
        publishTwist(u, w)

    publishTwist(0, 0)

#This function accepts a speed and a distance for the robot to move in a straight line
#speed is given in m/s and distance is given in meters
def driveStraight(speed, distance):
    global pub
    global pose
    print 'driving straight'
    odist = 0

    r = rospy.Rate(50)

    startx = x
    starty = y
    while (x-startx)**2 + (y - starty)**2 < distance**2:
        publishTwist(speed, 0)
        #print "x: %f" % (x)
        #print "y: %f" % (y)
        rospy.sleep(poleRate)
    publishTwist(0, 0)

def rotate_to(angle):
    r = rospy.Rate(50)
    if(angle < 0):
        speed = -.5
    else:
        speed = 0.5
    while abs(theta - angle) > tolAngle:
        publishTwist(0, speed)
        #print "theta: %f" % (theta)
        r.sleep()
    publishTwist(0, 0)


def rotate(angle):
    global pub
    global theta

    r = rospy.Rate(50)

    thetaStart = theta
    print "Rotate: By Angle %f From angle: %f" % (angle, theta)

    startAngle = thetaStart
    targetAngle = thetaStart + angle

    if targetAngle > math.pi:
        targetAngle = targetAngle - 2*math.pi
    elif targetAngle < -math.pi:
        targetAngle = targetAngle + 2*math.pi

    print targetAngle

    if(angle < 0):
        speed = -.5
    else:
        speed = 0.5

    while abs(theta - targetAngle) > tolAngle:
        publishTwist(0, speed)
        #print "theta: %f" % (theta)
        r.sleep()
    publishTwist(0, 0)


#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    global tolAngle
    rospy.sleep(.5)
    l = base/2
    d1 = (radius - l) * theta
    d2 = (radius + l) * theta

    u1 = speed * d1
    u2 = speed * d2

    #calc forward velocity
    velocities = calcualteForwardVelocity(u1, u2);
    u =  velocities['u']
    w =  velocities['w']

    thetaStart = theta
    print "theta: %f" % (theta)
    thetaGoal = thetaStart + angle
    thetaGoal = mapRadians(thetaGoal)
    print "thetaGoal: %f" % (thetaGoal)

    while((theta < thetaGoal - tolAngle or theta > thetaGoal + tolAngle) and (not rospy.is_shutdown())):
        publishTwist(u, w)
        #print "theta: %f thetaGoal: %f" % (theta, thetaGoal)
        rospy.sleep(poleRate)

    publishTwist(0, 0)



#Odometry Callback function.
def read_odometry(data):
    px = data.pose.pose.position.x
    py = data.pose.pose.position.y
    quat = data.pose.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)

    global x
    global y
    global theta

    x = px
    y = py
    theta = yaw


#Bumper Event Callback function
def readBumper(msg):
    global bumper
    print "Bumper: %f" % msg.state
    if (msg.state == 1):
        bumper = 1
    else:
        bumper = 0



# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code:
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    pass # Delete this 'pass' once implemented


def shutdown():
    # stop turtlebot
    rospy.loginfo("Stop TurtleBot")
	# Publish the stop command
    publishTwist(0, 0)
	# sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
    rospy.sleep(1)


def initMovement():
    global r
    global base
    r = .035 #Turtle bot wheel radius = .035M
    base = .23 #Turtle bot wheel base = .23M

    global poleRate
    global tolDistance
    global tolAngle
    poleRate = .1 #rate at which the robot checks if it has reached its goal
    tolDistance = .1 #goal tollerance
    tolAngle = .05

    global bumper
    bumper = 0

    global teleop_pub
    global pose
    global odom_tf
    #global odom_list

    teleop_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    #pub = rospy.Publisher('...', ...) # Publisher for commanding robot motion
    sub = rospy.Subscriber('odom', Odometry, read_odometry, queue_size=1) # Callback function to read in robot Odometry messages

    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    publishTwist(0, 0)
    # Use this object to get the robot's Odometry
    #odom_list = tf.TransformListener()
    rospy.sleep(3)

    rospy.on_shutdown(shutdown)
