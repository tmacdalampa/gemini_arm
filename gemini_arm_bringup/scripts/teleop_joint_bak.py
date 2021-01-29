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
  pub = rospy.Publisher('/maxon_bringup/set_all_states', MotorStates, queue_size=10)
  rospy.init_node('gemini_arm_teleop_joint', anonymous=True)
  rate = rospy.Rate(10)

  msg1 = MotorState()
  msg2 = MotorState()
  msg3 = MotorState()
  msg4 = MotorState()
  msg5 = MotorState()
  msg6 = MotorState()
  msg = MotorStates()

  

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
      msg1.position = msg1.position + coord_diff
    
    elif key == 'a':
      msg1.position = msg1.position - coord_diff 

    elif key == 'w':
      msg2.position = msg2.position + coord_diff

    elif key == 's':
      msg2.position = msg2.position + coord_diff

    elif key == 'e':
      msg3.position = msg3.position + coord_diff

    elif key == 'd':
      msg3.position = msg3.position - coord_diff

    elif key == 'r':
      msg4.position = msg4.position + coord_diff

    elif key == 'f':
      msg4.position = msg4.position - coord_diff

    elif key == 't':
      msg5.position = msg5.position + coord_diff

    elif key == 'g':
      msg5.position = msg5.position - coord_diff

    elif key == 'y':
      msg6.position = msg6.position + coord_diff

    elif key == 'h':
      msg6.position = msg6.position - coord_diff

    elif key == 'c':
      print("exit")
      break

    else:
      continue
    
    msg.states.append(msg1);
    msg.states.append(msg2);
    msg.states.append(msg3);
    msg.states.append(msg4);
    msg.states.append(msg5);
    msg.states.append(msg6);

    pub.publish(msg)
    rate.sleep()


