#!/usr/bin/env python
import rospy
import actionlib
import select, termios, tty, sys, moveit_commander, random
import copy
import numpy as np

from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import PickupAction, PickupGoal, Grasp
from moveit_msgs.msg import MoveItErrorCodes, DisplayTrajectory, RobotTrajectory
from actionlib import SimpleActionClient, GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseArray, Quaternion, Point
from sensor_msgs.msg import JointState


COORD_DIFF = 0.05 #0.01

msg_hint="""

===================================
Moving around:
   u    i    o
   j    k    l
   m    ,    .

t : up (+z)
b : down (-z)

q/z : increase/decrease max speeds by 10%
e: exit
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

  rospy.init_node('gemini_arm_teleop_xyz')
  moveit_commander.roscpp_initialize(sys.argv)
  group_arm = moveit_commander.MoveGroupCommander("gemini_arm")

  # Get current pose
  ps = Pose()
  ps = group_arm.get_current_pose().pose

  # Subscribe to client
  #client = actionlib.SimpleActionClient(
  #  'arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
  #client.wait_for_server()


  coord_diff = COORD_DIFF
  status = 0

  print(msg_hint)

  while not rospy.is_shutdown():
    key = getKey()

    # Print hint
    if (status == 14):
      print(msg_hint)

    # Move around 
    if key == 'u':
      ps.position.x += coord_diff / 1.414
      ps.position.y += coord_diff / 1.414

    elif key == 'i':
      ps.position.x += coord_diff 

    elif key == 'o':
      ps.position.x += coord_diff / 1.414
      ps.position.y -= coord_diff / 1.414

    elif key == 'j':
      ps.position.y += coord_diff 

    elif key == 'k':
      ps.position.x += 0 
      ps.position.y += 0 
      ps.position.z += 0 

    elif key == 'l':
      ps.position.y -= coord_diff 

    elif key == 'm':
      ps.position.x -= coord_diff / 1.414
      ps.position.y += coord_diff / 1.414

    elif key == ',':
      ps.position.x -= coord_diff 

    elif key == '.':
      ps.position.x -= coord_diff / 1.414
      ps.position.y -= coord_diff / 1.414

    elif key == 't':
      ps.position.z += coord_diff 

    elif key == 'b':
      ps.position.z -= coord_diff 

    # Accelerate
    elif key == 'q':
      coord_diff *= 1.1 
      print("coord_diff = {}".format(coord_diff))
      status = (status + 1) % 15
      continue

    # Decelerate
    elif key == 'z':
      coord_diff *= 0.9 
      print("coord_diff = {}".format(coord_diff))
      status = (status + 1) % 15
      continue

    elif key == 'e':
      print("exit")
      break

    else:
      continue

    group_arm.set_pose_target(ps)
    group_arm.go()


