#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header
import rospy

NAME = 'lab5_testing'

'''
  * Generates a pose for the robot to atain
'''
def generate_pose(x, y, theta):
    stamped_pose = PoseStamped()
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    stamped_pose.header = header

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


def main():
    rospy.init_node(NAME)
    posePublisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)
    newPose = generate_pose(0, 0, 0)
    posePublisher.publish(newPose)
    print newPose
    rospy.spin()

if __name__ == '__main__':
    main()
