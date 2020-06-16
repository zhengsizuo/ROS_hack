#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Use UDP protocol to connect with Android
Authour: zhs
Date: 2020.5.28
"""
import socket
import json
import os
import time
from collections import deque

address = ('', 10003)  # 服务端地址和端口
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

s.bind(address)  # 绑定服务端地址和端口
print('Listening for broadcast at ', s.getsockname())

data_list = deque(maxlen=2)
data_list.append({"start": 0, "grasp": 0})
while True:
    data, addr = s.recvfrom(1024)  # 返回数据和接入连接的（客户端）地址
    data = json.loads(data)
    if not data:
        break
    print('[Received]', data)
    data_list.append(data)
    # last state waiting, current state beginning
    if data_list[-2].values()==[0, 0] and data.values()==[0, 1]:
        # os.system("gnome-terminal -e 'roslaunch bmirobot_gazebo bmirobot_gazebo_moveit.launch'")
        os.system("gnome-terminal -e 'roslaunch bmirobot_hw bmirobot_zhs_right.launch'")
        time.sleep(5)
        os.system("gnome-terminal -e 'roslaunch bmirobot_py bmirobot_rviz_control.launch'")
    if data_list[-2].values()==[0, 1] and data.values()==[1, 1]:
        # os.system("gnome-terminal -e 'roslaunch bmirobot_py bmirobot_rviz_control.launch'")
        time.sleep(5)
        print("Launch the grasping...")
        os.system("gnome-terminal -e 'roslaunch bmirobot_py python_node.launch'")

    # 启动之后才能抓取
    if data_list[-2].values()==[1, 1] and data.values()==[0, 1]:
        os.system("gnome-terminal -e 'rosnode kill move_group_tele_key2'")
        print("Kill the tele key node...")
    # last state beginning, current state closing
    if data_list[-2].values()[1]==1 and data.values()==[0, 0]:
        os.system("gnome-terminal -e 'rosnode kill -a'")
        print("Shutdown all the nodes...")
    # send = input('Input: ')
    # s.sendto(send.encode(), addr)  # UDP 是无状态连接，所以每次连接都需要给出目的地址

s.close()
