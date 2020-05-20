#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import time
import tf
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from control_msgs.msg import GripperCommand
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import MoveGroupAction

group = None
gripperClient = None
gripperSrv = None
offset_z = 0.05
sim = True
pi = 3.14159

scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()

#roll pitch yaw are radians
def GoToPoint(X,Y,Z,roll,pitch,yaw):
  global group
  psg = group.get_current_pose().pose

  psg.position.x = X
  psg.position.y = Y
  psg.position.z = Z

  #roll = roll * 3.1416 / 180
  #pitch = pitch * 3.1416 / 180
  #yaw = yaw * 3.1416 / 180

  q = quaternion_from_euler(roll,pitch,yaw)
  psg.orientation = Quaternion(*q)

  group.set_pose_target(psg)
  time.sleep(2)

  plan_res = group.go(wait=True)
  group.stop()

  return plan_res

def GoToJoint(joint0,joint1,joint2,joint3,joint4,joint5):
  global group
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = joint0 * 3.1416 / 180
  joint_goal[1] = joint1 * 3.1416 / 180
  joint_goal[2] = joint2 * 3.1416 / 180
  joint_goal[3] = joint3 * 3.1416 / 180
  joint_goal[4] = joint4 * 3.1416 / 180
  joint_goal[5] = joint5 * 3.1416 / 180

  plan_res = group.go(joint_goal)
  group.stop()

  return plan_res

def MoveRelativeJoint(relative0,relative1,relative2,relative3,relative4,relative5):
  global group
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = joint_goal[0] + (relative0 * 3.1416 / 180)
  joint_goal[1] = joint_goal[1] + (relative1 * 3.1416 / 180)
  joint_goal[2] = joint_goal[2] + (relative2 * 3.1416 / 180)
  joint_goal[3] = joint_goal[3] + (relative3 * 3.1416 / 180)
  joint_goal[4] = joint_goal[4] + (relative4 * 3.1416 / 180)
  joint_goal[5] = joint_goal[5] + (relative5 * 3.1416 / 180)

  plan_res = group.go(joint_goal)
  group.stop()

  return plan_res

def MoveRelativeXYZ(relativeX,relativeY,relativeZ,relative_roll,relative_pitch,relative_yaw):
  global group
  psg = group.get_current_pose().pose
  #psg1 = group.get_current_pose().pose
  #print(psg)

  # Convert from quat to euler
  quaternion = (psg.orientation.x, psg.orientation.y, psg.orientation.z, psg.orientation.w)
  euler = tf.transformations.euler_from_quaternion(quaternion)
  roll = euler[0]
  pitch = euler[1]
  yaw = euler[2]

  #angle to radians
  relative_roll = relative_roll * 3.1416 / 180
  relative_pitch = relative_pitch * 3.1416 / 180
  relative_yaw = relative_yaw * 3.1416 / 180

  psg.position.x = psg.position.x + relativeX
  psg.position.y = psg.position.y + relativeY
  psg.position.z = psg.position.z + relativeZ
  
  q = quaternion_from_euler(roll + relative_roll,pitch + relative_pitch,yaw + relative_yaw)
  psg.orientation = Quaternion(*q)

  group.set_pose_target(psg)
  time.sleep(2)

  plan_res = group.go(wait=True)
  group.stop()

  return plan_res

def go_idel():
  global group
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 0
  joint_goal[1] = 0.78 #45 #
  joint_goal[2] = 0.78
  joint_goal[3] = 0
  joint_goal[4] = 0.13 #-7.448 #
  joint_goal[5] = 0

  #joint_goal = joint_goal * pi/180
  plan_res = group.go(joint_goal)

  group.stop()
  group.clear_pose_targets()

  print('pose idel reached')
  return plan_res

#vertical
def go_ps1():
  global group
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 1.57
  joint_goal[1] = -1.24
  joint_goal[2] = -0.689
  joint_goal[3] = 0.0
  joint_goal[4] = -1.929
  joint_goal[5] = 0
  plan_res = group.go(joint_goal)

  group.stop()
  print('pose 1 reached')
  return plan_res

#horizontal
def go_ps2():
  global group
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 0
  joint_goal[1] = 0.093
  joint_goal[2] = -1.348
  joint_goal[3] = -0.002
  joint_goal[4] = -1.254
  joint_goal[5] = 0
  plan_res = group.go(joint_goal)

  group.stop()
  return plan_res

#horizontal with angle
def go_ps3():
  global group
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 0
  joint_goal[1] = -0.230
  joint_goal[2] = -0.444
  joint_goal[3] = 0.030
  joint_goal[4] = -0.0673
  joint_goal[5] = 0
  plan_res = group.go(joint_goal)

  group.stop()
  return plan_res

#horizontal with angle
def go_ps4():
  global group
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 0
  joint_goal[1] = 0.093
  joint_goal[2] = -1.648
  joint_goal[3] = -0.002
  joint_goal[4] = -2.054
  joint_goal[5] = 0
  plan_res = group.go(joint_goal)

  group.stop()
  return plan_res

#horizontal flipped
def go_ps5():
  global group
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 0
  joint_goal[1] = 0.093
  joint_goal[2] = -1.348
  joint_goal[3] = -0.002
  joint_goal[4] = -1.254
  joint_goal[5] = 3.14
  plan_res = group.go(joint_goal)

  group.stop()
  return plan_res

#vertical with angel
def go_ps6():
  global group
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 0
  joint_goal[1] = 0.093
  joint_goal[2] = -1.348
  joint_goal[3] = -0.002
  joint_goal[4] = -1.254
  joint_goal[5] = 0.78;
  plan_res = group.go(joint_goal)

  group.stop()
  return plan_res

#pick
def go_ps7():
  global group
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 0
  joint_goal[1] = -0.78
  joint_goal[2] = -0.78
  joint_goal[3] = 0
  joint_goal[4] = 0
  joint_goal[5] = 0
  plan_res = group.go(joint_goal)

  group.stop()
  return plan_res
#a little bit lower from ps7
def go_ps7_lower():
  global group
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 0
  joint_goal[1] = -0.809
  joint_goal[2] = -0.966
  joint_goal[3] = 0
  joint_goal[4] = -0.346
  joint_goal[5] = 0.015
  plan_res = group.go(joint_goal)

  group.stop()
  return plan_res
def go_ps7_cart(): 
  global group
  psg = Pose()
  psg = group.get_current_pose().pose
  #psg.orientation.x =  -0.996
  #psg.orientation.y = -0.040
  #psg.orientation.z = 0.071
  #psg.orientation.w = 0.030
  psg.position.x = -1.105
  psg.position.y = 0.077
  psg.position.z = 0.827
  #psg.orientation = Quaternion(*quaternion_from_euler(-3.075, 0.140, 0.086, 'syxz'))
  group.set_pose_target(psg)
  plan_res = group.go(wait=True)
  group.stop()
  group.clear_pose_targets()
  print('ps_7_cart reached')
  return

def go_ps7_lower_cart(): 
  global group
  psg = Pose()
  psg = group.get_current_pose().pose
  #psg.orientation.x =  -1.105
  #psg.orientation.y = -0.040
  #psg.orientation.z = 0.072
  #psg.orientation.w = 0.028
  psg.position.x = -1.105
  psg.position.y = 0.076
  psg.position.z = 0.728
  #psg.orientation = Quaternion(*quaternion_from_euler(-3.079, 0.142, 0.084, 'syxz'))
  group.set_pose_target(psg)
  plan_res = group.go(wait=True)
  group.stop()
  group.clear_pose_targets()
  print('ps_7_lower_cart reached')
  return

def go_ps7_updown():
  global group
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 0
  joint_goal[1] = -0.78
  joint_goal[2] = -0.78
  joint_goal[3] = 0
  joint_goal[4] = -0.13
  joint_goal[5] = 0
  plan_res = group.go(joint_goal)
  group.stop()
  print('ps7 reached')

  psg = Pose()

  psg = group.get_current_pose().pose
  print(psg)
  psg.position.z = psg.position.z - 0.1
  group.set_pose_target(psg)
  print('sleep 2 sec')
  time.sleep(2)
  
  print('go down')
  plan_res = group.go(wait=True)
  group.stop()
  group.clear_pose_targets()
  print('ps_7_lower_cart reached')
  psg = Pose()
  psg = group.get_current_pose().pose
  print(psg)
  action('c')
  return plan_res

def go_zero():
  global group
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 0.0
  joint_goal[1] = 0.0
  joint_goal[2] = 0.0
  joint_goal[3] = 0.0
  joint_goal[4] = 0.0
  joint_goal[5] = 0.0
  plan_res = group.go(joint_goal)

  group.stop()
  print('zero pose reached')
  return plan_res 

def go_ps123():

  ps1 = [0.004, 0, 0.512, 0 ,0.55, 0]
  ps2 = [0.112, -0.189, 0.452, 1.508, -1.141, -0.27]
  ps3 = [-0.136, -0.057, 0.505, -0.146, 0.451, -2.608]
  ps123 = [ps1, ps2, ps3]
    
  global group
  psg = group.get_current_pose().pose


  #roll = roll * 3.1416 / 180
  #pitch = pitch * 3.1416 / 180
  #yaw = yaw * 3.1416 / 180

  #q = quaternion_from_euler(roll,pitch,yaw)
  #psg.orientation = Quaternion(*q)
  '''
  group.set_pose_target(ps1)
  plan_res = group.go(wait=True)
  #time.sleep(2)
  group.stop()
  print('ps1 reach')
  
  #ps1 = group.get_current_pose().pose
  group.set_pose_target(ps2)
  plan_res = group.go(wait=True)
  #time.sleep(2)
  group.stop()
  print('ps2 reach')

  #ps2 = group.get_current_pose().pose
  group.set_pose_target(ps3)
  plan_res = group.go(wait=True)
  #time.sleep(2)
  group.stop()
  print('ps3 reach')
  '''
  group.set_pose_targets(ps123)
  plan_res = group.go(wait=True)
  group.stop()
  print('finish')
  
  return plan_res
    

#--------------20200420--------------
if __name__ == '__main__':
  rospy.init_node('pick_and_place')
  
  moveGroupClient = actionlib.SimpleActionClient('move_group', MoveGroupAction)
  group = moveit_commander.MoveGroupCommander('gemini_arm')
  group.set_goal_orientation_tolerance(0.001)
  group.set_goal_position_tolerance(0.001)

  rospy.loginfo('Ready to plan.')
  raw_input()
  


  start_time = time.time()
  
  if go_idel():
      time.sleep(1)

  end_time = time.time()

  print(end_time-start_time)

  rospy.spin()
