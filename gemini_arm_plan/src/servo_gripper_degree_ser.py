#!/usr/bin/env python
import rospy

from std_msgs.msg import UInt16
from gemini_arm_plan.srv import degree, degreeResponse

def initservo():
    rospy.sleep(1)
    pub.publish(30)
    rospy.sleep(1)
    pub.publish(120)
    rospy.sleep(1)
    pub.publish(30)
    
    return

def action(req):
    #print('req_msg = '.format(req.degree))
    pub.publish(req.degree)  
    res = True

    if res == True:
        return degreeResponse(True)
    else:
	      return degreeResponse(False)


if __name__ == '__main__':
    rospy.init_node('servo_gripper')
    
    pub = rospy.Publisher('servo', UInt16)

    # Initial Servo Gripper
    rospy.loginfo('Initial Servo Gripper.')
    initservo()

    # Servo gripper is ready
    rospy.loginfo('Servo Gripper is Ready.')
    rospy.Service('servo_gripper', degree, action)

    rospy.spin()