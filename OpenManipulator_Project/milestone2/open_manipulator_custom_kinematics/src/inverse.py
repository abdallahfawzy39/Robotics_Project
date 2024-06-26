#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Float64

#globale variables from robot config
dist = 0.077
len2 = 0.13
len3 = 0.25
theta0 = np.arctan2(0.024, 0.128)

target_goal = Float32MultiArray()

def inverse_kinematics_callback(msg):
    # get the goal cooardinates
    target_x = msg.data[0]
    target_y = msg.data[1]
    target_z = msg.data[2]
    # Perform inverse kinematics calculations here to determine joint angles

    ang1 = np.arctan(target_y/target_x)
    if target_x > 0 and target_y < 0: #4th quad
        ang1 = -ang1
    elif target_x < 0 and target_y < 0: #3rd quad
        ang1 = ang1 + np.pi
    elif target_x < 0 and target_y > 0: #2nd quad
        ang1 = ang1 + np.pi/2

    #trignometric method
    eucl = np.sqrt(target_x**2 + target_y**2) #euclidean distance between x and y target
    s = np.sqrt((target_z - dist)**2 +eucl**2) # euclidean distance but with difference between z_goal,dist(robot config) and previous variable

    ang3 = np.arccos((s**2-len2**2-len3**2)/(2*len2*len3)) #cosine rule
    if (0<ang3<np.pi/4) or (3/4*np.pi<ang3<2*np.pi): #if 1st or 4th quad
        ang2 = np.arctan2(target_z - dist, eucl) - np.arcsin(len3 * np.sin(ang3)/s)
    else: 
        ang2 = np.arctan2(target_z - dist, eucl) + np.arcsin(len3 * np.sin(ang3)/s)
        ang3 = -ang3

    # joint angles according to confinguration of the robot

    joint1_angle = ang1  # Calculate joint 1 angle
    joint2_angle = -ang2 - theta0 + np.pi/2  # Calculate joint 2 angle
    joint3_angle = -ang3 + theta0 - np.pi/2  # Calculate joint 3 angle
    rospy.loginfo("Joint angles from 1,2,3: ")
    print(joint1_angle, joint2_angle, joint3_angle)

    # Publish joint angles to control the robot in Gazebo
    pub_j1.publish(joint1_angle)
    pub_j2.publish(joint2_angle)
    pub_j3.publish(joint3_angle)

if __name__ == '__main__':
    rospy.init_node('ikine_node')
    rospy.loginfo("starting inverse kinematic node") #terminal
    # Subscriber for target goal position
    rospy.Subscriber('/target_goal', Float32MultiArray, inverse_kinematics_callback)

    # Publishers for joint angles
    pub_j1 = rospy.Publisher('/joint1_position/command', Float64, queue_size=10)
    pub_j2 = rospy.Publisher('/joint2_position/command', Float64, queue_size=10)
    pub_j3 = rospy.Publisher('/joint3_position/command', Float64, queue_size=10)
    rate = rospy.Rate(10)
    rospy.spin()
