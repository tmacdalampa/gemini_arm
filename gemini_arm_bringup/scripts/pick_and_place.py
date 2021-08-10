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
offset_z = 0.05
sim = True
pi = 3.14159

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

  plan_res = group.go(wait=True)
  group.stop()

  return plan_res

def GoToJoint(joint0,joint1,joint2,joint3,joint4,joint5):
  global group
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = joint0
  joint_goal[1] = joint1
  joint_goal[2] = joint2
  joint_goal[3] = joint3
  joint_goal[4] = joint4
  joint_goal[5] = joint5

  plan_res = group.go(joint_goal,wait=True)
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

  plan_res = group.go(joint_goal, wait=True)
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
  plan_res = group.go(wait=True)
  group.stop()

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
  for i in range (1):
    GoToJoint(0, 0.52, 1.05, 0, -1.56, 0)
    time.sleep(1)
  
    MoveRelativeXYZ(0, 0, -0.1-0.05*i, 0, 0, 0)
    time.sleep(2)
  
    MoveRelativeXYZ(0, 0, 0.05, 0, 0, 0)
    time.sleep(1)
    GoToJoint(0.5, 0.52, 0.96, 0, 0, 0)
    time.sleep(1)
    GoToJoint(-0.5, 0.52, 0.96, 0, 0, 0)
    time.sleep(1)
    GoToJoint(-0.6, 0.52, 1.05, 0, -1.56, 0)
    time.sleep(1)
    MoveRelativeXYZ(0, 0, -0.1+0.05*i, 0, 0, 0)
    time.sleep(2)
    GoToJoint(0, 0, 0, 0, 0, 0)
  

  end_time = time.time()

  print(end_time-start_time)

  #rospy.spin()
