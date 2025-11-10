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


def wait_for_service(service_name):
    rospy.logwarn(f"Waiting rosservice {service_name} to be ready...")
    rospy.wait_for_service(service_name)


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
            {'source': u, 'target': v, 'weight': data.get('weight', 1.0), 
             'D_cost': data.get('D_cost', 1.0), 'PD_cost': data.get('PD_cost', 1.0), 'BC_cost': data.get('BC_cost', 1.0)} 
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






class HeuristicCounter:
    """
    A wrapper class for a heuristic function that counts
    how many times the heuristic is called.
    """
    def __init__(self, heuristic_func):
        self.heuristic = heuristic_func
        self.counter = 0

    def __call__(self, u, v):
        """
        This method makes the class instance callable,
        just like a function.
        """
        # Increment the counter every time A* calls it
        self.counter += 1
        
        # Now, return the value from the real heuristic
        return self.heuristic(u, v)

    def reset(self):
        """Resets the counter to zero for a new run."""
        self.counter = 0

    def get_count(self):
        """Returns the current count."""
        return self.counter

def send_goal(p, G, next_dest, nextnext_dest=None, time_threshold=-1, obs=False, first=False):
    pos = nx.get_node_attributes(G, 'pos')
    x, y = pos[next_dest]
    if nextnext_dest is not None:
        x2, y2 = pos[nextnext_dest]
        angle = math.atan2(y2-y, x2-x)
    else:
        angle = 0
    inputs = [x, y, angle, time_threshold]
    
    if not obs:
        p.exec_action('goto', "_".join([str(input) for input in inputs]))
    else:
        if nextnext_dest is not None:
            inputs.append(1 if not first and not rospy.get_param('/hrisim/robot_obs', False) else 0)
        else:
            inputs.append(0)
        p.exec_action('gotoobs', "_".join([str(_input) for _input in inputs]))
    
    
def get_prediction(p):
    p.exec_action('predict', "")

    risk_map_data = rospy.get_param('/hrisim/risk_map')
    tmp_ARCs = risk_map_data['arcs']
    ARCs = [(arc.split("__")[0], arc.split("__")[1]) for arc in tmp_ARCs]
    PDs = risk_map_data['PDs']
    BCs = risk_map_data['BCs']
    tot_inf_time = risk_map_data['tot_inf_time']
    PD_inf_time = risk_map_data['PD_inf_time']
    BC_inf_time = risk_map_data['BC_inf_time']
    mean_inf_time = (sum(PD_inf_time)/len(PD_inf_time) + sum(BC_inf_time)/len(BC_inf_time))
    
    risk_map = {}
    for i, arc in enumerate(ARCs):
        risk_map[arc] = {
            'PD': PDs[i],
            'BC': BCs[i]
        }
    return risk_map, tot_inf_time, mean_inf_time


def shortest_heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2)**2 + (y1 - y2)**2)**0.5