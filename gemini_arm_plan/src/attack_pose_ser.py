#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi
from arm_plan.srv import PoseSrv, PoseSrvResponse
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import MoveGroupAction


add_odj = None
group = None
plan_res =None

class add_object_atc(object):
    def __init__(self):
        super(add_object_atc, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        group_name = "arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.scene = moveit_commander.PlanningSceneInterface()

    def add_obj_A(self):
        box_name = ' '
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x=0.0
        box_pose.pose.position.y=0.815 #0.65
        box_pose.pose.position.z=0.2
        box_pose.pose.orientation.w = 1.0
        box_name = "areaA_box"
        self.scene.add_box(box_name, box_pose, size=(0.91, 0.47, 0.87))

        return box_name

    def add_obj_B(self):
        box_name = ' '
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x=0.0
        box_pose.pose.position.y=-0.843 #0.65
        box_pose.pose.position.z=0.25
        box_pose.pose.orientation.w = 1.0
        box_name = "areaA_box"
        self.scene.add_box(box_name, box_pose, size=(1.152, 0.326, 0.898))

        return box_name

    def add_obj_C(self):
        box_name = ' '
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x=-0.93
        box_pose.pose.position.y=0.0 #0.65
        box_pose.pose.position.z=0.2
        box_pose.pose.orientation.w = 1.0
        box_name = "areaC_box"
        self.scene.add_box(box_name, box_pose, size=(0.6, 1.0, 0.729))

        return box_name


    def go_to_pose_goal(self, x, y, z):

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation = Quaternion(*quaternion_from_euler(1.75, 0.00, 0.00, 'szyx'))
        self.group.set_pose_target(pose_goal)
        plan_res = self.group.go()

        return plan_res

def planner(req):
    global add_odj

    # add box
    rospy.loginfo('add object')
    if req.str_box_ind == 'a':
        re_box_name = add_odj.add_obj_A()
    elif req.str_box_ind == 'b':
        re_box_name = add_odj.add_obj_B()
    elif req.str_box_ind == 'c':
        re_box_name = add_odj.add_obj_C()
    
    rospy.sleep(1)
    # go to attack pose
    print('pick_pose = ({}, {}, {})'.format(
        req.pose.position.x, req.pose.position.y, req.pose.position.z))
    res = add_odj.go_to_pose_goal(
        req.pose.position.x, req.pose.position.y, req.pose.position.z)

    rospy.sleep(0.5)
    print(res)
    if res == True:
	return PoseSrvResponse(True)
    else:
	return PoseSrvResponse(False)

if __name__ == '__main__':
  rospy.init_node('attacking_pose')

  add_odj = add_object_atc()

  pose_srv = rospy.Service('attacking_pose', PoseSrv, planner)

  rospy.spin()
