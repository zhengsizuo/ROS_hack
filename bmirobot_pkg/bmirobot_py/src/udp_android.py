#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Use UDP protocol to connect with Android
Authour: zhs
Date: 2020.5.28
"""
import socket
import roslaunch
import json

address = ('127.0.0.1', 10001)  # 服务端地址和端口
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(address)  # 绑定服务端地址和端口
while True:
    data, addr = s.recvfrom(1024)  # 返回数据和接入连接的（客户端）地址
    data = json.loads(data)
    if not data:
        break
    print('[Received]', data)

    if data["start"] == 1:
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        tracking_launch = roslaunch.parent.ROSLaunchParent(
            uuid, ["/home/nvidia//robot_ws/src/ROS_hack/bmirobot_pkg/bmirobot_gazebo/launch/bmirobot_gazebo_moveit.launch"])
        tracking_launch.start()
    if data["grasp"] == 1:
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        tracking_launch = roslaunch.parent.ROSLaunchParent(
            uuid,
            ["/home/nvidia//robot_ws/src/ROS_hack/bmirobot_pkg/bmirobot_py/python_node.launch"])
        tracking_launch.start()

    send = input('Input: ')
    s.sendto(send.encode(), addr)  # UDP 是无状态连接，所以每次连接都需要给出目的地址

s.close()
