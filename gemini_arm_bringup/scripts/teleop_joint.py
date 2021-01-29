#!/usr/bin/env python
import rospy
import actionlib
import select, termios, tty, sys, moveit_commander, random
import copy
import numpy as np
import math

from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import PickupAction, PickupGoal, Grasp
from moveit_msgs.msg import MoveItErrorCodes, DisplayTrajectory, RobotTrajectory
from actionlib import SimpleActionClient, GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseArray, Quaternion, Point
from sensor_msgs.msg import JointState


COORD_DIFF = 5*math.pi/180

msg_hint="""
===================================
Moving around:
   q    w    e    r    t    y
   a    s    d    f    g    h
q/a : axis1 joint +/- 5 degrees
w/s : axis2 joint +/- 5 degrees
e/d : axis3 joint +/- 5 degrees
r/f : axis4 joint +/- 5 degrees
t/g : axis5 joint +/- 5 degrees
y/h : axis6 joint +/- 5 degrees
b: exit
===================================
"""
scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()

def getKey():
  tty.setraw(sys.stdin.fileno())
  select.select([sys.stdin], [], [], 0)
  key = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key


if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)

  rospy.init_node('gemini_arm_teleop_joint')
  moveit_commander.roscpp_initialize(sys.argv)
  
  group_arm = moveit_commander.MoveGroupCommander("gemini_arm")



  # Get current pose
  #ps = Pose()
  #ps = group_arm.get_current_pose().pose

  global joint_goal
  #joint_goal = group_arm.get_current_joint_values()

  #for i in range(0, 5, 1):
  #  print joint_goal[i]

  coord_diff = COORD_DIFF
  status = 0

  print(msg_hint)

  while not rospy.is_shutdown():
    key = getKey()

    joint_goal = group_arm.get_current_joint_values()

    # Move around
    if key == 'b':
      print('break')
      break 
    elif key == 'q':
      print('q')
      joint_goal[0] = joint_goal[0] + coord_diff
      #print(joint_goal[0])

    elif key == 'a':
      print('a')
      joint_goal[0] = joint_goal[0] - coord_diff

    elif key == 'w':
      print('w')
      joint_goal[1] = joint_goal[1] + coord_diff

    elif key == 's':
      print('s')
      joint_goal[1] = joint_goal[1] - coord_diff

    elif key == 'e':
      print('e')
      joint_goal[2] = joint_goal[2] + coord_diff
      
    elif key == 'd':
      print('d')
      joint_goal[2] = joint_goal[2] - coord_diff
    
    elif key == 'r':
      print('r')
      joint_goal[3] = joint_goal[3] + coord_diff
    
    elif key == 'f':
      #print('f')
      joint_goal[3] = joint_goal[3] - coord_diff
      print('f')
      
    elif key == 't':
      print('t')
      joint_goal[4] = joint_goal[4] + coord_diff
      
    elif key == 'g':
      print('g')
      joint_goal[4] = joint_goal[4] - coord_diff
      
    elif key == 'y':
      print('y')
      joint_goal[5] = joint_goal[5] + coord_diff
      
    elif key == 'h':
      #print('h')
      joint_goal[5] = joint_goal[5] - coord_diff
      print('h')

  

    
    else:
      continue

    group_arm.go(joint_goal)