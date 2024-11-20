#!/usr/bin/env python

from __future__ import print_function
import math
import serial
import numpy as np

import tf
import rospy
import tf2_ros
from geometry_msgs.msg import (
    PoseStamped, 
    Vector3, 
    Quaternion,
    PointStamped,
    Point
)
from std_srvs.srv import (
    Trigger, 
    TriggerResponse
)

from iiwa_initial_pose_align import get_initial_pose


class iiwa_task_manager:
    def __init__(self):
        self.use_conveyor = rospy.get_param("/iiwa_task_manager/use_conveyor")
        self.use_gripper = rospy.get_param("/iiwa_task_manager/use_gripper")
        self.use_vision_sensor = rospy.get_param("/iiwa_task_manager/use_vision_sensor")
        self.ser = None
        if self.use_conveyor and self.use_gripper:
            udevname = rospy.get_param("/iiwa_task_manager/udevname")
            portrate = rospy.get_param("/iiwa_task_manager/portrate")
            self.ser = serial.Serial(udevname,portrate)

        self.pose_pub = rospy.Publisher('/iiwa/command/CartesianPose',PoseStamped,queue_size=1)
        rospy.Subscriber("pick_positions",PointStamped,self.store_pick_positions)
        self.pick_positions = {}
        rospy.Subscriber("place_positions",PointStamped,self.store_place_positions)
        self.place_positions = {}
        rospy.Service('move_to_pp',Trigger,self.move_to_pp)
        self.goal_pose = get_initial_pose()
    
    def __del__(self):
        if self.ser is None:
            rospy.loginfo("no serial connection has been closed")
            return
        self.ser.close()

    def store_pick_positions(self,data):
        self.pick_positions[data.header.frame_id] = data.point

    def store_place_positions(self,data):
        self.place_positions[data.header.frame_id] = data.point

    def drive_gripper(self):
        if not self.use_gripper:
            return
        if self.ser is None:
            rospy.loginfo("could not drive the gripper because the serial connection is not established")
            return
        self.ser.write('4')
        rospy.sleep(rospy.Duration(3.0)) # sec

    def drive_conveyour(self):
        if not self.use_conveyor:
            return
        if self.ser is None:
            rospy.loginfo("could not drive the conveyour because the serial connection is not established")
            return
        self.ser.write('5') # slow speed
        rospy.sleep(rospy.Duration(3.0)) # sec
    
    def initializing_pose(self,wait_sec=3.0):
        self.goal_pose = get_initial_pose()
        self.goal_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("initializing pose")
        rospy.loginfo(self.goal_pose.pose.position)
        self.pose_pub.publish(self.goal_pose)
        rospy.sleep(rospy.Duration(wait_sec)) # sec  

    def approaching_conveyor(self,approach_dist,wait_sec=3.0):
        # going down
        self.goal_pose.header.stamp = rospy.Time.now()
        self.goal_pose.pose.position.z -= approach_dist
        rospy.loginfo("approaching to conveyor")
        rospy.loginfo(self.goal_pose.pose.position)
        self.pose_pub.publish(self.goal_pose)
        rospy.sleep(rospy.Duration(wait_sec)) # sec    

    def departing_conveyor(self,depart_dist,wait_sec=3.0):
        # going up
        self.goal_pose.header.stamp = rospy.Time.now()
        self.goal_pose.pose.position.z += depart_dist
        rospy.loginfo("departing from conveyor")
        rospy.loginfo(self.goal_pose.pose.position)
        self.pose_pub.publish(self.goal_pose)
        rospy.sleep(rospy.Duration(wait_sec)) # sec

    def picking(self,det_point,wait_sec=3.0):
        # moving to above target
        self.goal_pose.header.stamp = rospy.Time.now()
        self.goal_pose.pose.position.z -= 0.075

        # TODO: calculating the distances in x-y plane based on detected points
        # self.goal_pose.pose.position.x =
        # self.goal_pose.pose.position.y =

        rospy.loginfo("going down initially to picking")
        rospy.loginfo(self.goal_pose.pose.position)
        self.pose_pub.publish(self.goal_pose)
        rospy.sleep(rospy.Duration(wait_sec))
        
        approach_dist = 0.075
        rospy.loginfo("distance to approach: "+str(approach_dist))
        depart_dist = 0.150
        rospy.loginfo("distance to depart: "+str(depart_dist))

        self.approaching_conveyor(approach_dist,wait_sec)
        self.drive_gripper() # grasping after stop of robot    
        self.departing_conveyor(depart_dist,wait_sec)

    def transporting(self,point,wait_sec=3.0):
        self.goal_pose.header.stamp = rospy.Time.now()
        self.goal_pose.pose.position = point
        rospy.loginfo("moving to transporting")
        rospy.loginfo(self.goal_pose.pose.position)
        self.pose_pub.publish(self.goal_pose)
        rospy.sleep(rospy.Duration(wait_sec)) # sec

    def placing(self,point,wait_sec=3.0):
        rospy.loginfo("placing")
        self.transporting(point,wait_sec)
        self.drive_gripper() # releasing after stop of robot

    def move_to_pp_execution(self):
        self.drive_conveyour()

        if not self.use_vision_sensor:
            self.pick_positions = {"label0":Point(0.0,0.0,0.0)} # dummy
            self.place_positions = {"label0":Point(0.150,0.700,0.400)} # dummy

        for pick_object in self.pick_positions:
            self.picking(self.pick_positions[pick_object], 3.0)
            self.placing(self.place_positions[pick_object], 3.0)
            self.initializing_pose(3.0)

    def move_to_pp(self,req):
        rospy.loginfo("move_to_pp")
        self.move_to_pp_execution()
        return TriggerResponse(success=True)


if __name__ == '__main__':        
    rospy.init_node('iiwa_task_manager',anonymous=True)
    iiwa_tmanager = iiwa_task_manager()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("shutting down: "+rospy.get_name())
