#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Author: Zheng Haosi
Date: 2020.5.19
Description: subcribe Gemagic Touch rostopics from another ROS terminal to control the arm.
"""
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from collections import deque

import numpy as np 

def generate_pose():
    random_pose = np.random.randn(3)
    x = 0.3*random_pose[0] + 0.4
    y = 0.4*random_pose[1]
    z = 0.4*random_pose[2] + 0.4
    return [x, y, z]


class MoveItIkDemo:
    def __init__(self, group_name):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander(group_name)
        self.pose_deque = deque(maxlen=100)
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
        rospy.Subscriber('/phantom/pose', PoseStamped, self.poseCallback)
        self._return_to_home()
        

    def _return_to_home(self):
        # 获取终端link的名称，这个在setup assistant中设置过了
        self.end_effector_link = self.arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'world'
        self.arm.set_pose_reference_frame(self.reference_frame)
                
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)
        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('right_home')
        self.arm.go()
        rospy.sleep(1)

    def poseCallback(self, poseMsg):
        pose = poseMsg.pose.position
        orientation = poseMsg.pose.orientation
        self.pose_deque.append([2*pose.x+0.5-.0341, 3*pose.y-0.24, 2*pose.z+0.7-0.2, orientation.x,
        orientation.y, orientation.z, orientation.w])

    def plan_goal(self, goal_pose):
        
        # arm.set_max_acceleration_scaling_factor(0.5)
        # arm.set_max_velocity_scaling_factor(0.5)
 
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        #参考坐标系，前面设置了
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now() #时间戳？
        #末端位置   
        target_pose.pose.position.x = goal_pose[0]
        target_pose.pose.position.y = goal_pose[1]
        target_pose.pose.position.z = goal_pose[2]
        #末端姿态，四元数
        target_pose.pose.orientation.x = goal_pose[3]
        target_pose.pose.orientation.y = goal_pose[4]
        target_pose.pose.orientation.z = goal_pose[5]
        target_pose.pose.orientation.w = goal_pose[6]
        
        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        # 规划运动路径，返回虚影的效果
        traj = self.arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        self.arm.execute(traj)
        # rospy.sleep(1)  #执行完成后休息1s
 
        # 控制机械臂回到初始化位置
        # arm.set_named_target('right_home')
        # arm.go()
    
    def exit(self):
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

 
if __name__ == "__main__":
    IK_agent = MoveItIkDemo(group_name='right_arm')
    while IK_agent.pose_deque:
        rospy.sleep(0.5)
        goal_pose = np.mean(IK_agent.pose_deque, axis=0)
        print goal_pose, len(IK_agent.pose_deque)
        IK_agent.plan_goal(goal_pose)
    
    IK_agent.exit()

