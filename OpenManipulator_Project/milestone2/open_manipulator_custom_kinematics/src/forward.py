#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Float64
from sensor_msgs.msg import JointState

def STD_DH (theta, d, a, alpha):
    # transfer function using DH parameters
    transform = np.array([
            [np.cos(theta),     -np.sin(theta)*np.cos(alpha),     np.sin(theta)*np.sin(alpha),   a*np.cos(theta)],
            [np.sin(theta),      np.cos(theta)*np.cos(alpha),    -np.cos(theta)*np.sin(alpha),   a*np.sin(theta)],
            [0.0,                np.sin(alpha),                   np.cos(alpha),                 d              ],
            [0.0,                0.0,                             0.0,                           1.0            ]
        ])
    return transform


def calculate_forward_kinematics(theta1, theta2, theta3, theta4):
    #initial configuration for the robot required to move
    theta0 = np.arctan2(0.024, 0.128)
    
    # calculation of transformation matrix of each joint compared to previous
    #              [theta,                          d,            a,         alpha  ]
    
    T0_1 = STD_DH  ( theta1,                        0.077,        0,         np.pi/2)
    T1_2 = STD_DH  (-theta2-theta0+np.pi/2,         0,            0.13,      0      )
    T2_3 = STD_DH  (-theta3+theta0-np.pi/2,         0,            0.135,     0      )
    T3_4 = STD_DH  (-theta4,                        0,            0.126,     0      )
    
    # general transformation matrix
    T0_4 = T0_1 @ T1_2 @ T2_3 @ T3_4
    
    return T0_4

def rotation_matrix_to_euler(rotation_matrix):

    # get position from transformation matrix: [x,y,z]
    x = rotation_matrix[0,3]
    y = rotation_matrix[1,3]
    z = rotation_matrix[2,3]
    
    # Convert rotation matrix to Euler angles using roll-pitch-yaw convention (XYZ)
    roll  = np.arctan2  (rotation_matrix[2, 1],   rotation_matrix[2, 2])
    pitch = np.arctan2  (-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1] * 2 + rotation_matrix[2, 2] * 2))
    yaw   = np.arctan2  (rotation_matrix[1, 0],   rotation_matrix[0, 0])
    
    #publish position to robot_pose topic
    pos = [x, y, z, roll, pitch, yaw]
    rospy.loginfo("forward kinematic pose x,y,z,roll,pitch,yaw = ")
    rospy.loginfo(np.array(pos).astype(np.float16))
    pose_msg = Float32MultiArray()
    pose_msg.data = pos
    pub_pos.publish(pose_msg)
      

def joint_angles_callback(msg):
    # angles of robot are from the third element to sixth element in the joint_state topic
    ang1 = msg.position[2]
    ang2 = msg.position[3]
    ang3 = msg.position[4]
    ang4 = msg.position[5]
    major_transform = calculate_forward_kinematics(ang1, ang2, ang3, ang4)
    rotation_matrix_to_euler(major_transform)


if __name__ == '__main__':
    rospy.init_node('fkine_node') #node start
    rospy.loginfo("starting forward kinematic node") #terminal
    pub_pos = rospy.Publisher('/robot_pose', Float32MultiArray, queue_size=10) #publish start
    rospy.Subscriber('/joint_states', JointState, joint_angles_callback) #subscribe to gazebo joint state topic
    rospy.spin()