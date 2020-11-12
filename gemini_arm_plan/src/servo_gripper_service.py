#!/usr/bin/env python
import rospy

from std_msgs.msg import UInt16
from gemini_arm_plan.srv import PoseSrv, PoseSrvResponse

close_degree = 100
open_degree = 30

def action(req):
    #print('req_msg = '.format(req.str_box_ind))
    state = req.str_box_ind
    if state == 'c':
        pub.publish(close_degree)  
    elif state == 'o':
        pub.publish(open_degree)
    res = True

    if res == True:
        return PoseSrvResponse(True)
    else:
	      return PoseSrvResponse(False)


if __name__ == '__main__':
    rospy.init_node('servo_gripper')
    
    pub = rospy.Publisher('servo', UInt16)

    # Servo gripper is ready
    rospy.loginfo('Servo Gripper is Ready.')
    rospy.Service('servo_gripper', PoseSrv, action)

    rospy.spin()