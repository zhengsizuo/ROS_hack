#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
UDP client test
Authour: zhs
Date: 2020.6.2
"""
import socket
import json
address = ('192.168.2.107', 10001)  # 服务端地址和端口
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
while True:
    trigger = {'start': 1, 'grasp': 1}
    print('[Sent]', trigger)
    trigger_str = json.dumps(trigger)
    s.sendto(trigger_str, address)
    data, addr = s.recvfrom(1024)  # 返回数据和接入连接的（服务端）地址
    # data = data.decode()
    # print('[Recieved]', data)
    # if trigger == '###':  # 自定义结束字符串
    #     break
s.close()

