#!/usr/bin/env python3
import numpy as np
from math import pi
import rospy
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState

pose = rospy.get_param('poses/goal_pose')

prev_state = JointState()
curr_state = JointState()

def callback(data):

    global pose

    curr_state = data
    e = np.array([prev_state[0] - curr_state.position[0], prev_state[1] - curr_state.position[1], prev_state[2] - curr_state.position[2]])

    cmd_pos1 = pose[0] - data.position[0]
    cmd_pos2 = pose[1] - data.position[1]
    cmd_pos3 = pose[2] - data.position[2]
    
    prev_state = data

    pub1.publish(cmd_pos1)
    pub2.publish(cmd_pos2)
    pub3.publish(cmd_pos3)


rospy.init_node('scara_planner', anonymous=True)

pub1 = rospy.Publisher('/scara_control/joint1_position_controller/command', Float64, queue_size=10)
pub2 = rospy.Publisher('/scara_control/joint2_position_controller/command', Float64, queue_size=10)
pub3 = rospy.Publisher('/scara_control/joint3_position_controller/command', Float64, queue_size=10)    
sub = rospy.Subscriber('/joint_states', JointState, callback)
rospy.spin()