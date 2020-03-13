#!/usr/bin/env python
"""
Util functions to move arm joints and hand
Authour: zhs 
Date: 2020.2.7
"""
import rospy
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Float64MultiArray


def move_hand(grasp_pub, angle):
    """
    angle: positive->open; negative->close
    """
    msg = Float64MultiArray()
    msg.data = [angle, -angle]

    grasp_pub.publish(msg)

def move_joints(joint_pub, joints):
    msg = Float64MultiArray()
    msg.data = joints

    joint_pub.publish(msg)

def test():
    rospy.init_node('talker', anonymous=True)
    grasp_pub = rospy.Publisher('/bmirobot/right_group_hand_controller/command', Float64MultiArray, queue_size=1)
    joint_pub = rospy.Publisher('/bmirobot/right_group_controller/command', Float64MultiArray, queue_size=1)
    rate = rospy.Rate(10) # 10hz

    joints_list = [0]*7
    while not rospy.is_shutdown():
        rospy.loginfo("Start moving!")
        # joints_list[2] = -pi/4
        # joints_list[3] = -pi/4

        move_hand(grasp_pub, -0.2)
        #move_joints(joint_pub, joints_list)

        rate.sleep()


if __name__ == '__main__':
    test()