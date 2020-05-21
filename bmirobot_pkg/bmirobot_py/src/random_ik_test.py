#!/usr/bin/env python

import logging
logger = logging.getLogger("write." + __name__)
logger.setLevel(logging.DEBUG)

handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(levelname)s] %(name)s -> %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

import sys
import math, random

import rospy
import tf
import numpy as np

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, DisplayRobotState

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

FRAME = "world"

rospy.wait_for_service('compute_ik')
compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

pub_markers = rospy.Publisher('visualization_marker', Marker,queue_size=1)
pub_ik_target = rospy.Publisher('ik_target', PoseStamped, queue_size=1)

rospy.init_node("sample_ik_reachable")

tl = tf.TransformListener()

id = 0

def get_ik(target, group = "right_arm"):
    """
    :param target:  a PoseStamped give the desired position of the endeffector.
    """

    
    service_request = PositionIKRequest()
    service_request.group_name = group
    #service_request.robot_state = initial_state
    #service_request.ik_link_name = "pen_link"
    service_request.pose_stamped = target
    service_request.timeout.secs= 0.1
    service_request.avoid_collisions = False

    try:
        resp = compute_ik(ik_request = service_request)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def cube(position, ok = True):

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
    sheet.scale.x = 0.002
    sheet.scale.y = 0.002
    sheet.scale.z = 0.002
    sheet.color.b = .0
    if ok:
        sheet.color.g = 1.0
        sheet.color.r = .0
    else:
        sheet.color.g = .0
        sheet.color.r = 1.0
    sheet.color.a = 0.2

    return sheet

def generate_pose():
    random_pose = np.random.randn(3)
    x = 0.3*random_pose[0] + 0.4
    y = 0.4*random_pose[1]
    z = 0.4*random_pose[2] + 0.4
    return x, y, z

if __name__ == "__main__":

    logger.info("Sampling space around " + FRAME)
    target = PoseStamped()
    target.header.frame_id = FRAME
    

    tl.waitForTransform("/right_link8",  FRAME, rospy.Time(), rospy.Duration(1))
    t = tl.getLatestCommonTime("/right_link8", FRAME)
    position, quaternion = tl.lookupTransform(FRAME, "/right_link8", t)

    qx,qy,qz,qw = quaternion
    target.pose.orientation.x = qx
    target.pose.orientation.y = qy
    target.pose.orientation.z = qz
    target.pose.orientation.w = qw

    while not rospy.is_shutdown():

        # x = (random.random() - 0.5) / 10
        # y = (random.random() - 0.5) / 10
        # z = (random.random() - 0.5) / 20
        x, y, z = generate_pose()
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z

        pub_ik_target.publish(target)

        print("Checking %s %s %s" %(x,y,z))

        res = get_ik(target)
        if res.error_code.val != 1:
            print(res.error_code.val)
            pub_markers.publish(cube(target.pose.position, False))
        else:
            pub_markers.publish(cube(target.pose.position, True))