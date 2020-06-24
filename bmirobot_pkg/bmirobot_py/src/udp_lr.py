#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Use UDP protocol to connect with Android, control left arm and right arm
Authour: zhs
Date: 2020.6.22
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

right_data = deque(maxlen=2)
right_data.append({"right_grasp": 0, "right_start": 0})
while True:
    data, addr = s.recvfrom(1024)  # 返回数据和接入连接的（客户端）地址
    data = json.loads(data)
    if not data:
        break
    print('[Received]', data)
    right_data.append(data)
    # last state waiting, current state beginning
    if right_data[-2].values()==[0, 0] and data.values()==[0, 1]:
        os.system("gnome-terminal -e 'roslaunch bmirobot_hw bmirobot_zhs_two.launch'")
        time.sleep(5)
        os.system("gnome-terminal -e 'roslaunch bmirobot_py bmirobot_rviz_control.launch'")
    if right_data[-2].values()==[0, 1] and data.values()==[1, 1]:
        # os.system("gnome-terminal -e 'roslaunch bmirobot_py bmirobot_rviz_control.launch'")
        time.sleep(2)
        print("Launch the grasping...")
        os.system("gnome-terminal -e 'roslaunch bmirobot_py python_node.launch'")

    # 启动之后才能抓取
    if right_data[-2].values()==[1, 1] and data.values()==[0, 1]:
        os.system("gnome-terminal -e 'rosnode kill move_group_tele_key2'")
        print("Kill the tele key node...")
    # last state beginning, current state closing
    if right_data[-2].values()[1]==1 and data.values()==[0, 0]:
        os.system("gnome-terminal -e 'rosnode kill -a'")
        print("Shutdown all the nodes...")


s.close()
