#!/usr/bin/env python

import rospy

from std_msgs.msg import UInt16
from servo_gripper.srv import GripperState, gripperResponse,gripperRequest
#from arm_plan.srv import PickPlace, PickPlaceResponse

def Pose_client(str_state):
    rospy.wait_for_service('servo_gripper')

    try:
      go = rospy.ServiceProxy('servo_gripper', GripperState)
      res = go(str_state)
      return res.result

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
   print("start")

   str_command = 'c'
   res = Pose_client(str_command)
   print(res)