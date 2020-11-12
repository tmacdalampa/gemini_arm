#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from control_msgs.msg import GripperCommand
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import MoveGroupAction, MoveGroupActionResult


group = None
gripperClient = None
gripperSrv = None
offset_z = 0.05
sim = True
plantext = None
mgar =None

def go_idel():
    global group
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0.78
    joint_goal[2] = -1.54
    joint_goal[3] = 0
    joint_goal[4] = -0.13
    joint_goal[5] = 0
    joint_goal[6] = 0
    plan_res = group.go(joint_goal)

    group.stop()
    return plan_res

def pickCB():
    global group, offset_z, sim
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0.3
    pose_goal.position.y = 0.029
    pose_goal.position.z = 1.413
    pose_goal.orientation = Quaternion(*quaternion_from_euler(1.75, 0.00, 0.00, 'szyx'))
    group.set_pose_target(pose_goal)
    rospy.loginfo('GO')
    result = group.go()
    rospy.loginfo('Finish')

    return result


if __name__ == '__main__':
    rospy.init_node('pick_and_place')
    moveGroupClient = actionlib.SimpleActionClient('move_group', MoveGroupAction)
    #moveGroupClient.wait_for_server(rospy.Duration())
    group = moveit_commander.MoveGroupCommander('arm')
    rospy.loginfo('Ready to plan.')
    raw_input()
    #pickCB()
    go_idel()
    rospy.spin()
