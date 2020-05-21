#!/usr/bin/env python
"""
Author: Zheng Haosi
Date: 2020.5.11
"""
import sys
import rospy
import geometry_msgs.msg
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt
import numpy as np
# import pandas as pd
import csv


pose_list = []
def poseCallback(poseMsg):
    pose = poseMsg.pose.position
    pose_list.append([pose.x, pose.y, pose.z])

if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber('/phantom/pose', PoseStamped, poseCallback, queue_size=1)
    print "============ Press `Enter` to record the poses(press ctrl-d to exit) ..."
    raw_input()
    file_name = '/home/zhengsizuo/robot_ws/src/ROS_hack/bmirobot_pkg/bmirobot_py/src/trajectory_3.csv'
    with open(file_name, 'w') as f:
        f_csv = csv.writer(f)
        f_csv.writerows(pose_list)
    print('write to csv!')
    rospy.spin()