#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

if __name__ == '__main__':
    rospy.init_node("TargetGoal_publisher") #start goal node
    rospy.loginfo("publishing x,y,z of goal") #terminal
    pub = rospy.Publisher('/target_goal', Float32MultiArray, queue_size=10) #publish in target goal topic
    rate = rospy.Rate(10) #rate
    
    # GOAL Cordinates
    x = 0.25
    y = 0.11
    z = 0.05

    while not rospy.is_shutdown():
        #send goal by multiarray while ros running
        goal_msg = Float32MultiArray()
        goal_msg.data = [x, y, z]
        pub.publish(goal_msg)
        rate.sleep()
