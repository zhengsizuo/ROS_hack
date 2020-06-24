#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Util functions to move arm joints and hand
Authour: zhs 
Date: 2020.2.7
"""
import sys
import rospy
import geometry_msgs.msg
import moveit_commander
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


class MoveGroupPythonInteface(object):
    """MoveGroupPythonInteface"""

    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('move_group_python_interface', anonymous=True)

        group_name = "right_arm"
        group = moveit_commander.MoveGroupCommander(group_name)

        self.group = group

        self.grasp_pub = rospy.Publisher('/bmirobot/right_group_hand_controller/command', Float64MultiArray,
                                         queue_size=1)

    def go_to_joint_state(self, joint_goal):
        group = self.group
        ## Planning to a Joint Goal
        # We can get the joint values from the group and adjust some of the values:
        try:
            group.set_joint_value_target(joint_goal)
        except:
            group.set_joint_value_target([0]*7)
        plan = group.plan()
        group.execute(plan, wait=True)

    def move_hand(self, angle):
        """
        angle: positive->open; negative->close
        """
        msg = Float64MultiArray()
        msg.data = [angle, -angle]

        self.grasp_pub.publish(msg)

def test():
    rospy.init_node('talker', anonymous=True)
    grasp_pub = rospy.Publisher('/bmirobot/right_group_hand_controller/command', Float64MultiArray, queue_size=1)
    joint_pub = rospy.Publisher('/bmirobot/right_group_controller/command', Float64MultiArray, queue_size=1)
    rate = rospy.Rate(10) # 10hz

    joints_list = [0]*7
    count = 0
    move_joints(joint_pub, joints_list)  # return to initial state
    while not rospy.is_shutdown():
        rospy.loginfo("Start moving!")
        if count > 3:
            break
        # joints_list[5] = -pi/2 + count*pi/8
        joints_list[3] = -pi/4 + count*pi/8

        move_hand(grasp_pub, 0.2)
        move_joints(joint_pub, joints_list)
        count += 1
        #rate.sleep()
        rospy.sleep(2)


if __name__ == '__main__':
    test()