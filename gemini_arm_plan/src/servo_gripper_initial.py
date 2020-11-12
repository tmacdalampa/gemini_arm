#!/usr/bin/env python

import rospy

from std_msgs.msg import UInt16
from gemini_arm_plan.srv import degree, degreeResponse
#from arm_plan.srv import PickPlace, PickPlaceResponse

def Pose_client(num):
    rospy.wait_for_service('servo_gripper')
    #print(num)
    try:
      go = rospy.ServiceProxy('servo_gripper', degree)
      res = go(num)
      return res.result

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
   rospy.loginfo('GRIPPER INITAIL!')
   
   Pose_client(30)
   rospy.sleep(1)
   Pose_client(120)
   rospy.sleep(1)
   Pose_client(30)