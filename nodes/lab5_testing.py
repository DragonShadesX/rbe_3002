#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import GridCells
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header
from src.point2point_move_function import *
from std_msgs.msg import String
import rospy, math

NAME = 'lab5_testing'

def genHeader():
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    return header

'''
  * Generates a pose for the robot to atain
'''
def generate_pose(x, y, theta):
    stamped_pose = PoseStamped()
    stamped_pose.header = genHeader()

    pose = Pose()

    point = Point()
    point.x = x; point.y = y; point.z = 0;
    pose.position = point
    quat = Quaternion()
    newQuat = quaternion_from_euler(0, 0, theta)
    quat.x = newQuat[0]; quat.y = newQuat[1]; quat.z = newQuat[2]; quat.w = newQuat[3]
    pose.orientation = quat
    stamped_pose.pose = pose
    return stamped_pose

def status_callback(ret):
    #ret = GoalStatusArray()
    if len(ret.status_list) > 0: # Prevents print spamming
        #print "Status callback"
        #print ret
        global status_list
        status_list = ret.status_list

def getStatus(id):
    global status_list
    for element in status_list:
        if element.goal_id.id is str(id):
            return element.status
    return -1

def getHighestStatus():
    global status_list
    low = 0
    for element in status_list:
        if low < element.status:
            low = element.status
    return low


def driveTo(x,y, theta):
    rospy.loginfo("Drive to (%f, %f, %f)", x, y, theta)
    newPose = generate_pose(x, y, 0)
    #print newPose
    actionGoal = MoveBaseActionGoal()
    actionGoal.header = genHeader()
    actionGoal.goal_id.id = str(driveTo.goalID)
    actionGoal.goal_id.stamp = rospy.Time.now()
    goal = MoveBaseGoal()
    goal.target_pose = newPose
    actionGoal.goal = goal

    # Publish the goal to the robot
    global actionGoalPublisher
    actionGoalPublisher.publish(actionGoal)

    # Wait for the robot's status to to have reached the goal
    timeOutCounter = 0
    while not rospy.is_shutdown(): # This is done so that status can be checked and used
        rospy.sleep(4.)
        timeOutCounter += 1
        currentStatus = getStatus(driveTo.goalID)
        global cant_reach_list
        print "Status: %d, GoalID: %d, Driving to: (%f, %f, %f), # unreachable: %d" % (currentStatus, driveTo.goalID, x, y, theta, len(cant_reach_list  ))
        if currentStatus == GoalStatus.ABORTED or timeOutCounter > 20:
            print "The goal was aborted"

            cant_reach_list.append((x, y))
            break
        elif currentStatus == GoalStatus.REJECTED:
            print "The goal was rejected"
            break
        elif currentStatus == GoalStatus.LOST:
            print "The robot is lost, exiting driving"
            #TODO Should we send a cancel message?
            exit(1)
            break
        elif currentStatus == GoalStatus.SUCCEEDED:
            print "Drive to complete!"
            break
    driveTo.goalID += 1


def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

def frontier_callback(ret):
    rospy.loginfo("frontier_callback")
    print ret
    #ret = GridCells() #Used for autocompletion
    global frontierList
    frontierList = ret.cells


def getClosestGoal(minDistance):
    global frontierList
    (trans, rot) = getLocation()
    shortestDistance = None
    closestPoint = None
    global cant_reach_list
    for cell in frontierList:
        #cell = Point()
        point = (cell.x, cell.y, rot[2])
        newDistance = distance(point, trans)
        # If this is a point we are unable to reach
        if point in cant_reach_list:
            continue
        if (shortestDistance == None or shortestDistance > newDistance) and  abs(newDistance) > minDistance:
            shortestDistance = newDistance
            closestPoint = point


    if closestPoint == None and minDistance != 0:# If we were unable to find a point because all were unreachable
        #cant_reach_list = []
        return getClosestGoal(0)
    elif minDistance == 0:
        print "WE ARE DONE?"
        exit(0)
    else: # Now set the closes goal

        return closestPoint

def main():
    rospy.init_node(NAME)
    # Init Globals with their type
    global status_list
    status_list = []
    global cant_reach_list
    cant_reach_list = []

    global actionGoalPublisher
    init_point2pont()
    actionStatus = rospy.Subscriber('move_base/status', GoalStatusArray, status_callback)
    actionGoalPublisher = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)
    frontierSub = rospy.Subscriber('grid_frontier', GridCells, frontier_callback)
    rospy.wait_for_message('grid_frontier', GridCells)

    rospy.sleep(1)
    driveTo.goalID = getHighestStatus()

    global closestGoal
    while not rospy.is_shutdown():
        rospy.loginfo("Waiting for a little while")
        rospy.sleep(2) #Allow the goal to be calculated
        rospy.loginfo("Getting the closest goal")
        closesGoalCopy = getClosestGoal(2)
        rospy.loginfo("Getting the current location")
        driveTo(closesGoalCopy[0], closesGoalCopy[1], closesGoalCopy[2])

    rospy.spin()

if __name__ == '__main__':
    main()
