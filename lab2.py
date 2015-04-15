#!/usr/bin/env python



import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent


import tf
import numpy
import math 

#This takes linear/angular velocities makes a Twist message, and then publishes it.

def publishTwist(lin_vel, ang_vel):
    global pub

    twist_msg = Twist();		#Create the Twist Message

    twist_msg.linear.x = lin_vel	#Put data in the message
    twist_msg.angular.z = ang_vel

    #print "Publishing " + str(twist_msg)

    pub.publish(twist_msg)		#Send the Message
 
#This function accepts velocies for the two wheels and a time.

def spinWheels(u1, u2, time):
    global pub

    lin_vel = 0.5 * (u1 + u2)			#Calculate the linear velocity of the robot based on the wheel's rotatation. 
    ang_vel = (1 / .352) * (u1 - u2)		#Calculates the angular velocity of of the robot based on the wheel's rotatation..

    twist_msg = Twist();			#Makes a name-maker

    stop_msg = Twist();

    twist_msg.linear.x = lin_vel		#Populate the messages with the data.

    twist_msg.angular.z = ang_vel
   
    stop_msg.linear.x = 0

    stop_msg.angular.z = 0
    
    #While the specified amount of time has not elapsed, send Twist messages.
    now = rospy.Time.now().secs

    while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
        pub.publish(twist_msg)
    pub.publish(stop_msg)
    

#This method takes a speed and distance for the turtlebot and drives straight 
def driveStraight(speed, distance):
    global odom_list
    global pose 	

    x0 = pose.pose.position.x	#Set an origin

    y0 = pose.pose.position.y

    #The code loops until the distance equals the distance requested
    done = False

    while (not done and not rospy.is_shutdown()):
        x1 = pose.pose.position.x
        y1 = pose.pose.position.y

#Distance formula works by squaring the differences of the x and y values, summing them, and taking the square root.
        d = math.sqrt( (x1 - x0)**2 + (y1 - y0)**2 )	
        print "  " + str(distance) + " " + str(d)
        if (d >= distance):
            done = True
            publishTwist(0, 0)
        else:
            publishTwist(speed, 0)
        rospy.sleep(rospy.Duration(0, 500000))

# Go back and fix this    

#Accepts an angle and makes the robot rotate about it.
def rotate(angle):
    global odom_list
    global pose

    transformer = tf.TransformerROS()	
    rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0],	#Create th goal rotation
                            [math.sin(angle), math.cos(angle), 0],
                            [0,          0,          1]])

 

#Get all the transforms for frames
    odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    T_o_t = transformer.fromTranslationRotation(trans, rot)
    R_o_t = T_o_t[0:3,0:3]

 #Setup goal matrix
    goal_rot = numpy.dot(rotation, R_o_t)
    goal_o = numpy.array([[goal_rot[0,0], goal_rot[0,1], goal_rot[0,2], T_o_t[0,3]],
                 [goal_rot[1,0], goal_rot[1,1], goal_rot[1,2], T_o_t[1,3]],
                 [goal_rot[2,0], goal_rot[2,1], goal_rot[2,2], T_o_t[2,3]],
                 [0,             0,             0,             1]])

    #This code continuously creates and matches coordinate transforms.
    done = False
    while (not done and not rospy.is_shutdown()):
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        state = transformer.fromTranslationRotation(trans, rot)
        within_tolerance = abs((state - goal_o)) < .1
        if ( within_tolerance.all() ):
            spinWheels(0,0,0)
            done = True
        else:
            if (angle > 0):
                spinWheels(.1,-.1,.1)
            else:
                spinWheels(-.1,.1,.1)
	rospy.sleep(rospy.Duration(0, 500000))



#This function works the same as rotate except that it doesn't publish the linear velocities.

def driveArc(radius, speed, angle):
    global odom_list
    w = speed / radius
    v1 = w * (radius + .5*.352)
    v2 = w * (radius - .5*.352)

    #transforms
    transformer = tf.TransformerROS()	
    rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0],
                            [math.sin(angle), math.cos(angle), 0],
                            [0,          0,          1]])
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    T_o_t = transformer.fromTranslationRotation(trans, rot)
    R_o_t = T_o_t[0:3,0:3]
    goal_rot = numpy.dot(rotation, R_o_t)

    done = False
    while (not done and not rospy.is_shutdown()):
        #Gets the transforms
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        state = transformer.fromTranslationRotation(trans, rot)
        state_rot = state[0:3,0:3]

        #Decides if they are within tolerance or not
        within_tolerance = abs((state_rot - goal_rot)) < .3
        if (within_tolerance.all()):
            spinWheels(0,0,0)
            done = True
        else:
            if (angle >  0):
                spinWheels(v1,v2,.1)
            else:
                spinWheels(v2,v1,.1)

#This code step-wise calls methods to execute trajectories.
def executeTrajectory():
    stepSleep = rospy.Duration(.5)
    rotate(-90)
    driveStraight(.25, .6)
    rospy.sleep(stepSleep)
    rotate(-90)
    rospy.sleep(stepSleep)
    driveArc(.15, .1, 180)
    rospy.sleep(stepSleep)
    rotate(135)
    rospy.sleep(stepSleep)
    driveStraight(.25, .42)


#Odom "Callback" function.
def readOdom(msg):
    global pose
    global odom_tf

    pose = msg.pose
    geo_quat = pose.pose.orientation
  
    odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
        (pose.pose.orientation.x, pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w),
        rospy.Time.now(),
        "base_footprint",
        "odom")

#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        print "The Bumper was just Pressed"
        rospy.Timer(rospy.Duration(.01), timerCallback)
        executeTrajectory()

# This method was found online (Source:
#The timer is used when it is saving data to csv files for analysis.
def timerCallback(event):
    global pose

    x = pose.pose.position.x
    y = pose.pose.position.y
    quaternion = (pose.pose.orientation.x,
                  pose.pose.orientation.y,
                  pose.pose.orientation.z,
                  pose.pose.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    theta = euler[2]



#MAIN
def run():
    global pub
    global pose
    global odom_tf
    global odom_list

    cmds = [[1, 0], [2, 0], [0.5, 0], [0, 1], [0, 2], [0, 0], [1, 3.14]]
    
#Talked to Zac about initializing the node within the script, found the following 17 lines of code online.

    rospy.init_node('lab2')
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
    sub = rospy.Subscriber('odom', Odometry, readOdom, queue_size=5)
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper)

    odom_list = tf.TransformListener()
    odom_tf = tf.TransformBroadcaster()

    if not odom_tf:
        print "odom_tf is not running. Quitting."
        return
    else: 
        print odom_tf

    odom_tf.sendTransform((0, 0, 0),
        (0, 0, 0, 1),
        rospy.Time.now(),
        "base_footprint",
        "odom")
    sleeper = rospy.Duration(1)
    rospy.sleep(sleeper)


    #driveStraight(.1, 1)
    #driveArc(.5, .5, math.pi/2)
    #rotate(math.pi/2)
    #executeTrajectory()
    rospy.spin()
    #spinWheels(0.125, .25, 2)
    rospy.sleep(sleeper)
    rospy.loginfo("Done")
    

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
