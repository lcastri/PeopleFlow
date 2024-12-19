#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Pose
import tf
import time
import networkx as nx

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


def seconds_to_hhmmss(seconds):
    return time.strftime("%H:%M:%S", time.gmtime(seconds))


def seconds_to_hh(seconds):
    return time.strftime("%H", time.gmtime(seconds))


def load_graph_to_rosparam(graph, param_name):
        
    # Convert the graph into a dictionary format for ROS
    ros_graph = {
        'nodes': {node: data for node, data in graph.nodes(data=True)},
        'edges': [
            {'source': u, 'target': v, 'weight': data.get('weight', 1.0)} 
            for u, v, data in graph.edges(data=True)
        ]
    }
    
    # Save the graph dictionary to a ROS parameter
    rospy.set_param(param_name, ros_graph)
    rospy.loginfo(f"Graph saved to ROS parameter: {param_name}")
    return graph


def get_time_to_wp(g, wp_origin, wp_dest, heuristic, robot_speed=0.5):
    pos = nx.get_node_attributes(g, 'pos')
    
    path = nx.astar_path(g, wp_origin, wp_dest, heuristic=heuristic, weight='weight')
    distanceToWP = 0
    for wp_idx in range(1, len(path)):
        wp_current = pos[path[wp_idx-1]]
        wp_next = pos[path[wp_idx]]
        distanceToWP += math.sqrt((wp_next[0] - wp_current[0])**2 + (wp_next[1] - wp_current[1])**2)
        
    timeToWP = math.ceil(distanceToWP/robot_speed) + 2*len(path)-1
    return timeToWP