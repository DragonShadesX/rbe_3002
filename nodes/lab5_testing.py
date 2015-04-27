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
    ret = GoalStatusArray()
    if len(ret.status_list) > 0: # Prevents print spamming
        print "Status callback"
        print ret
        global status_list
        status_list = ret.status_list

def getStatus(id):
    global status_list
    for element in status_list:
        if element.goal_id.id == String(str(id)):
            return element.status
    return -1


def driveTo(x,y, theta):
    print "drive to"
    newPose = generate_pose(x, y, 0)
    #print newPose
    actionGoal = MoveBaseActionGoal()
    actionGoal.header = genHeader()
    actionGoal.goal_id.id = String(str(driveTo.goalID))
    actionGoal.goal_id.stamp = rospy.Time.now()
    goal = MoveBaseGoal()
    goal.target_pose = newPose
    actionGoal.goal = goal

    # Publish the goal to the robot
    global actionGoalPublisher
    actionGoalPublisher.publish(actionGoal)

    delayRate = rospy.Rate(1)
    # Wait for the robot's status to to have reached the goal
    while not rospy.is_shutdown(): # This is done so that status can be checked and used
        delayRate.sleep()
        currentStatus = getStatus(driveTo.goalID)
        if currentStatus == GoalStatus.ABORTED:
            print "The goal was aborted"
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
driveTo.goalID = 0

def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

def frontier_callback(ret):
    print "frontier_callback"
    print ret
    #ret = GridCells() #Used for autocompletion
    (trans, rot) = getLocation()

    shortestDistance = None
    closestPoint = None
    for cell in ret.cells:
        cell = Point()
        point = (cell.x, cell.y)
        newDistance = distance(point, trans)
        if shortestDistance == None or shortestDistance > newDistance :
            shortestDistance = newDistance
            closestPoint = point
    # Now set the closes goal
    global closestGoal
    closestGoal = closestPoint

def main():
    rospy.init_node(NAME)
    # Init Globals with their type
    global status_list
    status_list = []

    global actionGoalPublisher
    init_point2pont()
    actionStatus = rospy.Subscriber('move_base/status', GoalStatusArray, status_callback)
    actionGoalPublisher = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)
    frontierSub = rospy.Subscriber('grid_frontier', GridCells, frontier_callback)
    rospy.wait_for_message('grid_frontier', GridCells)

    global closestGoal

    while 1:
        closesGoalCopy = closestGoal
        driveTo(closesGoalCopy[0], closesGoalCopy[1], 0)
        rospy.sleep(1)

    rospy.spin()

if __name__ == '__main__':
    main()
