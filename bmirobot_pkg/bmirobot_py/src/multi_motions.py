#!/usr/bin/env python
"""
Multi motions defined by users
Authour: zhs 
Date: 2020.2.15
"""
import rospy
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Float64MultiArray

from util_functions import move_hand, move_joints

step = 0
joints_list = [0]*7

if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    grasp_pub = rospy.Publisher('/bmirobot/right_group_hand_controller/command', Float64MultiArray, queue_size=1)
    joint_pub = rospy.Publisher('/bmirobot/right_group_controller/command', Float64MultiArray, queue_size=1)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        print "============ Press `Enter` to begin the motion(press ctrl-d to exit) ..."
        raw_input()
        if step == 0:
            rospy.loginfo("Current phase: pre-grasping")
            joints_list[2] = -pi/4
            joints_list[3] = -pi/8
            move_joints(joint_pub, joints_list)

            move_hand(grasp_pub, 0.5)
            rospy.loginfo("End effector open!")
            step += 1
            rospy.sleep(3)
        
        if step == 1:
            rospy.loginfo("Current phase: grasping")
            joints_list[3] = 0
            move_joints(joint_pub, joints_list)
            rospy.sleep(2)
            move_hand(grasp_pub, -0.4)
            rospy.loginfo("End effector close!")
            step += 1
            rospy.sleep(3)
        
        if step == 2:
            rospy.loginfo("Current phase: delivering")
            joints_list[2] = pi/8
            move_joints(joint_pub, joints_list)
            rospy.sleep(2)
            move_hand(grasp_pub, 0.5)
            rospy.loginfo("Delivered the bottle!")
            step += 1
            rospy.sleep(3)
        
        if step == 3:
            rospy.loginfo("Current phase:returning")
            joints_list = [0]*7
            move_joints(joint_pub, joints_list)
            move_hand(grasp_pub, 0.0)
            rospy.loginfo("Returned to the normal state!")
            step = 0
            rospy.sleep(5)
        
        # if step == 4:
        #     rospy.loginfo("Current phase: %s" % step)
        #     joints_list[step] = -pi/4
        #     move_joints(joint_pub, joints_list)
        #     move_hand(grasp_pub, 0.5)
        #     rospy.loginfo("End effector open!")
        #     step = 0
        #     rospy.sleep(1)
        

        rate.sleep()