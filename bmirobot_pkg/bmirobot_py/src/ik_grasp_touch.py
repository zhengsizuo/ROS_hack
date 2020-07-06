#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Read touch pose topic to control the arm.
Authour: zhs
Date: 2020.6.26
"""
import logging

logger = logging.getLogger("write." + __name__)
logger.setLevel(logging.DEBUG)

handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(levelname)s] %(name)s -> %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)


import rospy
import tf
import numpy as np

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, DisplayRobotState

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray
from omni_msgs.msg import OmniButtonEvent

from util_functions import move_joints, move_hand


class Pose():
    def __init__(self):
        rospy.Subscriber('/phantom/pose', PoseStamped, self.poseCallback, queue_size=1)
        rospy.Subscriber('/phantom/button', OmniButtonEvent, self.buttonCallback)
        self.grasp_pub = rospy.Publisher('/bmirobot/right_group_hand_controller/command', Float64MultiArray,
                                         queue_size=1)
        self.posestamp = PoseStamped()

    def poseCallback(self, poseMsg):
        self.posestamp = poseMsg

    def buttonCallback(self, buttonMsg):
        if buttonMsg.white_button == 1:
            # 按白键，张开
            move_hand(self.grasp_pub, 0.4)
        if buttonMsg.grey_button == 1:
            # 按灰键，闭合
            move_hand(self.grasp_pub, -0.6)


FRAME = "world"
ENDEFFECTOR = "/right_link8"

rospy.wait_for_service('compute_ik')
compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

pub_markers = rospy.Publisher('visualization_marker', Marker, queue_size=1)  # visualize the target points
pub_ik_target = rospy.Publisher('ik_target', PoseStamped, queue_size=1)
pub_dr = rospy.Publisher('display_robot_state', DisplayRobotState, queue_size=1)
joint_pub = rospy.Publisher('/bmirobot/right_group_controller/command', Float64MultiArray, queue_size=1)

rospy.init_node("sample_ik_reachable")
# rospy.rate(10)

tl = tf.TransformListener()

id = 0
initial_state = RobotState()
print(initial_state)
joints_name = ['right_joint1', 'right_joint2', 'right_joint3', 'right_joint4', 'right_joint5', 'right_joint6',
               'right_joint7']
initial_state.joint_state.name = joints_name
initial_state.joint_state.position = [0] * 7


def get_ik(target, group="right_arm"):
    """
    :param target:  a PoseStamped give the desired position of the endeffector.
    """
    service_request = PositionIKRequest()
    service_request.group_name = group
    service_request.robot_state = initial_state
    service_request.ik_link_name = ENDEFFECTOR

    service_request.pose_stamped = target
    service_request.timeout.secs = 0.1
    service_request.avoid_collisions = False

    try:
        resp = compute_ik(ik_request=service_request)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def cube(position, ok=True):
    global id
    id += 1

    sheet = Marker()
    sheet.header.frame_id = FRAME
    sheet.header.stamp = rospy.get_rostime()
    sheet.ns = "sampling"
    sheet.action = Marker.ADD
    sheet.pose.orientation.w = 1.0
    sheet.pose.position = position
    sheet.id = id
    sheet.type = Marker.CUBE
    sheet.scale.x = 0.02
    sheet.scale.y = 0.02
    sheet.scale.z = 0.02
    sheet.color.b = .0
    if ok:
        sheet.color.g = 1.0
        sheet.color.r = .0
    else:
        sheet.color.g = .0
        sheet.color.r = 1.0
    sheet.color.a = 0.2

    return sheet


if __name__ == "__main__":
    logger.info("Sampling space around " + FRAME)
    rs = DisplayRobotState()

    target = PoseStamped()
    target.header.frame_id = FRAME

    tl.waitForTransform("/right_link8", FRAME, rospy.Time(), rospy.Duration(1))
    t = tl.getLatestCommonTime("/right_link8", FRAME)
    position, quaternion = tl.lookupTransform(FRAME, "/right_link8", t)

    qx, qy, qz, qw = quaternion
    target.pose.orientation.x = qx
    target.pose.orientation.y = qy
    target.pose.orientation.z = qz
    target.pose.orientation.w = qw

    p = Pose()
    while not rospy.is_shutdown():
        # 简单的线性空间映射
        pose = p.posestamp.pose.position
        x = 2 * pose.x + 0.341
        y = 2 * pose.y + 0.24 - (0.088 * 2)
        y = (0.24 - y) + 0.24 - 0.15
        z = 2 * pose.z + 0.2 - (-0.065 * 2)
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z * 0.7

        pub_ik_target.publish(target)

        print("Checking %s %s %s" % (x, y, z))

        res = get_ik(target)
        if res.error_code.val != 1:
            print(res.error_code.val)
            pub_markers.publish(cube(target.pose.position, False))  # 没找到逆解，不发布marker
        else:
            pub_markers.publish(cube(target.pose.position, True))
            # initial_state = res.solution
            rs.state = res.solution

            position = rs.state.joint_state.position[9: 16]
            print(", ".join(str(x) for x in position))
            move_joints(joint_pub, position)
            pub_dr.publish(rs)

            #rospy.sleep(0.2)