#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from control_msgs.msg import GripperCommand
from std_msgs.msg import String
from std_msgs.msg import UInt16
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import MoveGroupAction
from gemini_arm_plan.srv import degree, degreeResponse

group = None
gripperClient = None
gripperSrv = None
offset_z = 0.05
sim = True
global group

def go_idel():
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 0
  joint_goal[1] = 0
  joint_goal[2] = 0
  joint_goal[3] = 0
  joint_goal[4] = 0
  joint_goal[5] = 0
  group.go(joint_goal, wait = True)
  group.stop()
  return 

def go_attack():
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 0.0
  joint_goal[1] = 0.17453080415725708
  joint_goal[2] = -1.7453327178955078
  joint_goal[3] = 0.0
  joint_goal[4] = 0.0
  joint_goal[5] = 0.0
  group.go(joint_goal, wait = True)
  group.stop()

  return 

def go_pick():
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = -0.1745244860649109
  joint_goal[1] = -0.2617962062358856
  joint_goal[2] = -1.745240569114685
  joint_goal[3] = 0.0
  joint_goal[4] = 0.4363254904747009
  joint_goal[5] = 0.0
  group.go(joint_goal, wait = True)
  
  joint_goal1 = group.get_current_joint_values()
  joint_goal1[0] = -0.1745244860649109
  joint_goal1[1] = -0.6108578443527222
  joint_goal1[2] = -1.30899178981781
  joint_goal1[3] = 0.0
  joint_goal1[4] = 0.34905731678009033
  joint_goal1[5] = 0.0
  group.go(joint_goal1, wait = True)
  go(150)
  rospy.sleep(0.5)

  joint_goal2 = group.get_current_joint_values()
  joint_goal2[0] = -0.1745244860649109
  joint_goal2[1] = -0.2617962062358856
  joint_goal2[2] = -1.745240569114685
  joint_goal2[3] = 0.0
  joint_goal2[4] = 0.4363254904747009
  joint_goal2[5] = 0.0
  group.go(joint_goal2, wait = True)
  group.stop()

  return 

def go_place():
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 0.2617943286895752
  joint_goal[1] = -0.2617962062358856
  joint_goal[2] = -1.8326008319854736
  joint_goal[3] = 0.0
  joint_goal[4] = 0.2617891728878021
  joint_goal[5] = 0.0
  group.go(joint_goal, wait = True)

  joint_goal1 = group.get_current_joint_values()
  joint_goal1[0] = 0.26177912950515747
  joint_goal1[1] = -0.6108578443527222
  joint_goal1[2] = -1.30899178981781
  joint_goal1[3] = 0.0
  joint_goal1[4] = 0.34905731678009033
  joint_goal1[5] = 0.0
  group.go(joint_goal1, wait = True)
  go(30)
  rospy.sleep(0.5)

  joint_goal2 = group.get_current_joint_values()
  joint_goal2[0] = 0.2617943286895752
  joint_goal2[1] = -0.2617962062358856
  joint_goal2[2] = -1.8326008319854736
  joint_goal2[3] = 0.0
  joint_goal2[4] = 0.2617891728878021
  joint_goal2[5] = 0.0
  group.go(joint_goal2, wait = True)
  group.stop()

  return 

def task():
    go_attack()
    #rospy.sleep(0.5)
    go_pick()
    #rospy.sleep(0.5)
    #rospy.sleep(1)
    go_attack()
    #rospy.sleep(0.5)
    go_place()
    #rospy.sleep(0.5)
    #rospy.sleep(1)
    go_attack()


if __name__ == '__main__':
  rospy.init_node('pick_and_place')


  #offset_z = rospy.get_param('~above_target_dist', 0.05)
  moveGroupClient = actionlib.SimpleActionClient('move_group', MoveGroupAction)
  #moveGroupClient.wait_for_server(rospy.Duration())
  #group = moveit_commander.MoveGroupCommander(
  group = moveit_commander.MoveGroupCommander('gemini_arm')
  group.set_goal_orientation_tolerance(0.1)
  group.set_goal_position_tolerance(0.01)
    
  rospy.wait_for_service('servo_gripper')
  go = rospy.ServiceProxy('servo_gripper', degree)

  rospy.loginfo('Ready to plan.')
  raw_input()
  task()

  rospy.spin()