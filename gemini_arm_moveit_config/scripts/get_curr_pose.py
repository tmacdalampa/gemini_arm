#!/usr/bin/env python

import moveit_commander
import rospy, sys
import tf
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler


rospy.init_node('get_pose')
moveit_commander.roscpp_initialize(sys.argv)

group = moveit_commander.MoveGroupCommander("gemini_arm")

# Get current pose
ps = Pose()
ps = group.get_current_pose().pose

# Convert from quat to euler
quaternion = (ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w)
euler = tf.transformations.euler_from_quaternion(quaternion)
roll = euler[0]
pitch = euler[1]
yaw = euler[2]

print("position:")
print("  x = {:.3f}".format(ps.position.x))
print("  y = {:.3f}".format(ps.position.y))
print("  z = {:.3f}".format(ps.position.z))

print("orientation (quat):")
print("  x = {:.3f}".format(ps.orientation.x))
print("  y = {:.3f}".format(ps.orientation.y))
print("  z = {:.3f}".format(ps.orientation.z))
print("  w = {:.3f}".format(ps.orientation.w))

print("orientation (euler):")
print("  roll  = {:.3f}".format(roll))
print("  pitch = {:.3f}".format(pitch))
print("  yaw   = {:.3f}".format(yaw))