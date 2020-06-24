#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Author: Zheng Haosi
Date: 2020.3.4
Description: subcribe Gemagic Touch rostopics from another ROS terminal to control the arm.
"""
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Float64MultiArray
from omni_msgs.msg import OmniButtonEvent
from geometry_msgs.msg import PoseStamped

motion_step1 = [0, 0, -pi/8, -pi/8, 0, 0, 0]
motion_step2 = [0, 0, -pi/4+0.05, 0.03, 0, 0, 0]
motion_step3 = [0, 0, pi/8, 0, 0, 0, 0]
motion_step4 = [0]*7

motion_rise = [0, 0, -pi/4, -0.3, 0, 0, 0]


class MoveGroupPythonInteface(object):
  """MoveGroupPythonInteface"""
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)

    group_name = "right_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    self.group = group
    self.success_flag = ''
    self.move_flag = ''
    self.grasp_pub = rospy.Publisher('/bmirobot/right_group_hand_controller/command', Float64MultiArray, queue_size=1)
    rospy.Subscriber('/phantom/button', OmniButtonEvent, self.buttonCallback)
    rospy.Subscriber('/phantom/pose', PoseStamped, self.poseCallback)

  def buttonCallback(self, buttonMsg):
    # press grey button, grasp yes
    if buttonMsg.grey_button == 1:
        self.success_flag = 'Y'
    elif buttonMsg.white_button == 1:
        self.success_flag = 'N'
  
  def poseCallback(self, poseMsg):
    pose = poseMsg.pose.position
    x = pose.x *1000 # transfer to mm
    if x < -110:
        self.move_flag = 'L'
    elif x > 100:
        self.move_flag = 'R'
    elif -110< x <100:
        self.move_flag = ''

  def go_to_joint_state(self, joint_goal):
    group = self.group
    ## Planning to a Joint Goal
    # We can get the joint values from the group and adjust some of the values:
    group.set_joint_value_target(joint_goal)

    plan = group.plan()
    group.execute(plan, wait=True)

  def move_hand(self, angle):
    """
    angle: positive->open; negative->close
    """
    msg = Float64MultiArray()
    msg.data = [angle, -angle]

    self.grasp_pub.publish(msg)


def main():
  global step 
  step = 0
  move_group = MoveGroupPythonInteface()
  while not rospy.is_shutdown():

    if step == 0:
      print "============ Move left to begin the motion..."
      print move_group.move_flag
      if move_group.move_flag == 'L':
        rospy.loginfo("Current phase: pre-grasping")
        move_group.go_to_joint_state(motion_step1)

        move_group.move_hand(0.5)
        rospy.loginfo("End effector open!")
        step += 1
        # move_group.move_flag = ''
        rospy.sleep(2)
      move_group.move_flag = ''

    if step == 1:
      rospy.loginfo("Current phase: grasping")
      move_group.go_to_joint_state(motion_step2)
      rospy.sleep(1)
      move_group.move_hand(-0.15)
      rospy.loginfo("End effector close!")
      rospy.sleep(1)
      rospy.loginfo("Slightly rise the elbow!")
      move_group.go_to_joint_state(motion_rise)
      step += 1
      rospy.sleep(2)
    
    if step == 2:
      print "============ Press grey button if the grasping succussed ..."
      print "============ Press white if the grasping failed ..."
      
      if move_group.success_flag == 'Y':
          step += 1
          move_group.success_flag = ''
      if move_group.success_flag == 'N':
          #move_group.success_flag = ''
          move_group.go_to_joint_state(motion_step4)
          move_group.move_hand(0.0)
          rospy.loginfo("Returned to the normal state!")
          step = 0
          move_group.success_flag = ''
          rospy.sleep(2)

    if step == 3:
      rospy.loginfo("Current phase: delivering")
      print "============ Move right to deliver the bottle ..."
      if move_group.move_flag == 'R':
        move_group.go_to_joint_state(motion_step3)
      
        if move_group.success_flag == 'Y' or move_group.success_flag == 'N':
          move_group.move_hand(0.5)
          rospy.loginfo("Delivered the bottle!")
          step += 1
          move_group.move_flag = ''
          rospy.sleep(2)

    if step == 4:
      rospy.loginfo("Current phase:returning")
      move_group.go_to_joint_state(motion_step4)
      move_group.move_hand(0.0)
      rospy.loginfo("Returned to the normal state!")
      step = 0
      rospy.sleep(5)

if __name__ == '__main__':
  main()
