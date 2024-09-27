#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
import tf


def wait_for_param(param_name, timeout=30):
    rospy.logwarn(f"Waiting for parameter: {param_name}")
    while not rospy.has_param(param_name):
        rospy.sleep(0.1)
    rospy.loginfo(f"Parameter {param_name} found!")
    return rospy.get_param(param_name)


def getPose(p: Pose):
    x = p.position.x
    y = p.position.y
    
    q = (
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w
    )
    
    m = tf.transformations.quaternion_matrix(q)
    _, _, yaw = tf.transformations.euler_from_matrix(m)
    return x, y, yaw