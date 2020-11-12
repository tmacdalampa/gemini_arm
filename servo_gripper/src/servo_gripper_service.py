#!/usr/bin/env python
import rospy

from std_msgs.msg import UInt16
from servo_gripper.srv import GripperState, GripperStateResponse

close_degree = 45
open_degree = 0

def action(req):
    print('req_msg = '.format(req.gripper_state))
    state = req.gripper_state
    if state == 'c':
        pub.publish(close_degree)  
    elif state == 'o':
        pub.publish(open_degree)

if __name__ == '__main__':
    rospy.init_node('servo_gripper')
    
    pub = rospy.Publisher('servo', UInt16)

    # Servo gripper is ready
    rospy.loginfo('Servo Gripper is Ready.')
    rospy.Service('servo_gripper', GripperState, action)

    rospy.spin()