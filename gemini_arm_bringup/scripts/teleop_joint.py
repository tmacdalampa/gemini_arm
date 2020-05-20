#!/usr/bin/env python
import rospy
import select, termios, tty, sys
import copy
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from maxon_epos_msgs.msg import MotorStates, MotorState



COORD_DIFF = 5*math.pi/180

msg_hint="""

===================================
Moving around:
   q    w    e    r    t    y
   a    s    d    f    g    h

q/a : axis1 joint +/-
w/s : axis2 joint +/-
e/d : axis3 joint +/-
r/f : axis4 joint +/-
t/g : axis5 joint +/-
y/h : axis6 joint +/-


c: exit
===================================
"""
 
def getKey():
  tty.setraw(sys.stdin.fileno())
  select.select([sys.stdin], [], [], 0)
  key = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key


if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)
  pub = rospy.Publisher('/maxon_bringup/set_all_states', Float32MultiArray, queue_size=10)
  rospy.init_node('gemini_arm_teleop_joint', anonymous=True)
  rate = rospy.Rate(10)

  psg = MotorStates()
  psg.data = [0,0,0,0,0,0]

  coord_diff = COORD_DIFF
  status = 0

  print(msg_hint)

  while not rospy.is_shutdown():
    key = getKey()

    # Print hint
    if (status == 14):
      print(msg_hint)

    # Move around 
    if key == 'q':
      psg.data[0] = psg.data[0] + coord_diff
    
    elif key == 'a':
      psg.data[0] = psg.data[0] - coord_diff 

    elif key == 'w':
      psg.data[1] = psg.data[1] + coord_diff

    elif key == 's':
      psg.data[1] = psg.data[1] + coord_diff

    elif key == 'e':
      psg.data[2] = psg.data[2] + coord_diff

    elif key == 'd':
      psg.data[2] = psg.data[2] - coord_diff

    elif key == 'r':
      psg.data[3] = psg.data[3] + coord_diff

    elif key == 'f':
      psg.data[3] = psg.data[3] - coord_diff

    elif key == 't':
      psg.data[4] = psg.data[4] + coord_diff

    elif key == 'g':
      psg.data[4] = psg.data[4] - coord_diff

    elif key == 'y':
      psg.data[5] = psg.data[5] + coord_diff

    elif key == 'h':
      psg.data[5] = psg.data[5] - coord_diff

    elif key == 'c':
      print("exit")
      break

    else:
      continue

    pub.publish(psg)
    rate.sleep()


