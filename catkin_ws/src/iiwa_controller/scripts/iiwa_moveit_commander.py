#!/usr/bin/env python

import moveit_commander
import rospy
from geometry_msgs.msg import Pose, PoseStamped


def cmd_cb(msg):
    global move_group
    msg.pose.position.z += 0.978
    move_group.set_pose_target(msg.pose)
    move_group.go()


if __name__ == '__main__':
    rospy.init_node("iiwa_moveit_commander")
    move_group = moveit_commander.MoveGroupCommander("manipulator",robot_description="/iiwa/robot_description",ns="iiwa")
    move_group.set_end_effector_link('iiwa_link_ee')
    
    cmd_sub = rospy.Subscriber('/iiwa/command/CartesianPose', PoseStamped, cmd_cb)
    rospy.spin()

        