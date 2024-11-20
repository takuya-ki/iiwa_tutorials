#!/usr/bin/env python

from __future__ import print_function
import math

import tf
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from std_srvs.srv import (
    Trigger, 
    TriggerResponse
)


def euler_to_quaternion(euler):
    """Convert Euler Angles to Quaternion

    euler: geometry_msgs/Vector3
    quaternion: geometry_msgs/Quaternion
    """
    q = tf.transformations.quaternion_from_euler(
        euler.x, euler.y, euler.z)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def get_initial_pose():
    iiwa_initial_pose = PoseStamped()
    iiwa_initial_pose.header.frame_id = 'tool_link_ee'
    iiwa_initial_pose.header.stamp = rospy.Time.now()
    iiwa_initial_pose.pose.position = Vector3(0.610,0.340,0.400)
    iiwa_initial_pose.pose.orientation = \
        euler_to_quaternion(
            Vector3(math.radians(0.0),
                    math.radians(180.0), 
                    math.radians(0.0)))
    return iiwa_initial_pose


def initial_pose_align(req):
    global tool_pose

    # pose initialization
    pose_pub.publish(get_initial_pose())
    rospy.loginfo("initialized")
    if False:
        pass
    return TriggerResponse(success=True)


def storeCartesianPose(p):
    global tool_pose
    tool_pose = p


if __name__ == '__main__':
    rospy.init_node('iiwa_initial_pose_align')
    tool_pose = PoseStamped()
    pose_pub = rospy.Publisher(
        '/iiwa/command/CartesianPose', PoseStamped, queue_size=1)
    rospy.Subscriber(
        '/iiwa/state/CartesianPose', PoseStamped, storeCartesianPose)
    rospy.Service(
        'initial_pose_align', Trigger, initial_pose_align)
    rospy.spin()
