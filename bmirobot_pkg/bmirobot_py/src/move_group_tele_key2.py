#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Author: Zheng Haosi
Date: 2020.2.27
Grasp from the right side.
"""
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Float64MultiArray

motion_step1 = [0, 0, -pi/8, -pi/8, 0, 0, 0]
motion_step2 = [0, 0, -pi/4+0.05, 0, 0, 0, 0]
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

    self.grasp_pub = rospy.Publisher('/bmirobot/right_group_hand_controller/command', Float64MultiArray, queue_size=1)

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
  step = 0
  move_gruop = MoveGroupPythonInteface()
  while not rospy.is_shutdown():

    if step == 0:
      print "============ Press `Enter` to begin the motion(press ctrl-d to exit) ..."
      raw_input()
      rospy.loginfo("Current phase: pre-grasping")
      move_gruop.go_to_joint_state(motion_step1)

      move_gruop.move_hand(0.5)
      rospy.loginfo("End effector open!")
      step += 1
      rospy.sleep(2)

    if step == 1:
      rospy.loginfo("Current phase: grasping")
      move_gruop.go_to_joint_state(motion_step2)
      rospy.sleep(1)
      move_gruop.move_hand(-0.2)
      rospy.loginfo("End effector close!")
      rospy.sleep(1)
      rospy.loginfo("Slightly rise the elbow!")
      move_gruop.go_to_joint_state(motion_rise)

      print "============ Press `Y/y` if the grasping succussed ..."
      print "============ Press `N/n` if the grasping failed ..."
      recieve_key = raw_input()
      if recieve_key == 'Y' or recieve_key == 'y':
          step += 1
      if recieve_key == 'N' or recieve_key == 'n':
          move_gruop.go_to_joint_state(motion_step4)
          move_gruop.move_hand(0.0)
          rospy.loginfo("Returned to the normal state!")
          step = 0
      rospy.sleep(2)

    if step == 2:
      rospy.loginfo("Current phase: delivering")
      move_gruop.go_to_joint_state(motion_step3)
      print "============ Press `Enter` to deliver the bottle(press ctrl-d to exit) ..."
      raw_input()
      move_gruop.move_hand(0.5)
      rospy.loginfo("Delivered the bottle!")
      step += 1
      rospy.sleep(2)

    if step == 3:
      rospy.loginfo("Current phase:returning")
      move_gruop.go_to_joint_state(motion_step4)
      move_gruop.move_hand(0.0)
      rospy.loginfo("Returned to the normal state!")
      step = 0
      rospy.sleep(5)

if __name__ == '__main__':
  main()
