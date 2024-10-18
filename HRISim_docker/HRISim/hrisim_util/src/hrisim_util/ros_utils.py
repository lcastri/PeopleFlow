#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
import tf
import time


class ParameterTimeoutError(Exception):
    pass


def wait_for_param(param_name, timeout=60):
    start_time = time.time()
    while not rospy.has_param(param_name):
        if time.time() - start_time > timeout:
            rospy.logerr(f"Parameter {param_name} NOT found!")
            raise ParameterTimeoutError(f"Timeout exceeded while waiting for parameter: {param_name}")
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