#!/usr/bin/env python3
import numpy as np
from math import pi
import rospy
from std_msgs.msg import Float32
# theta1, theta2 and d are joint angles of robot

theta1 = np.arange(0,pi/2,20)
theta2 = np.arange(0,pi/2,20)
d = np.arange(0,-0.08,20)



def control_commander():
    pub1 = rospy.Publisher('/scara_control/joint1_position_controller/command', Float32, queue_size=10)
    
    
    rospy.init_node('control_commander', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        control_cmd1 = 1.5
        rospy.loginfo(control_cmd1)
        pub1.publish(control_cmd1)
        rate.sleep()

if __name__ == '__main__':
    try:
        control_commander()
    except rospy.ROSInterruptException:
        pass