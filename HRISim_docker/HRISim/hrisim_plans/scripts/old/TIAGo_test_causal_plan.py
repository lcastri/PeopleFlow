from itertools import combinations
import math
import os
import pickle
import sys
import copy

import numpy as np
import rospy
try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import pnp_cmd_ros
from pnp_cmd_ros import *
from robot_msgs.msg import BatteryStatus
from std_msgs.msg import String, Int32
from move_base_msgs.msg import MoveBaseAction
import actionlib
import hrisim_util.ros_utils as ros_utils
import hrisim_util.constants as constants
import networkx as nx
from peopleflow_msgs.msg import Time as pT
from std_srvs.srv import Trigger
from robot_srvs.srv import NewTask, NewTaskResponse, FinishTask, FinishTaskResponse


BATTERY_CRITICAL_LEVEL = 20


def send_goal(p, next_dest, nextnext_dest=None):
    pos = nx.get_node_attributes(G, 'pos')
    x, y = pos[next_dest]
    if nextnext_dest is not None:
        x2, y2 = pos[nextnext_dest]
        angle = math.atan2(y2-y, x2-x)
        coords = [x, y, angle]
    else:
        coords = [x, y, 0]
    p.exec_action('goto', "_".join([str(coord) for coord in coords]))
    
    
def get_prediction(p):
    p.exec_action('predict', "")

    risk_map_data = rospy.get_param('/hrisim/risk_map')
    flattened_PDs = risk_map_data['PDs']
    flattened_BACs = risk_map_data['BACs']
    n_waypoints = risk_map_data['n_waypoints']
    n_steps = risk_map_data['n_steps']

    PDs_matrix = np.array(flattened_PDs).reshape(n_waypoints, n_steps)
    BACs_matrix = np.array(flattened_BACs).reshape(n_waypoints, n_steps)

    risk_map = {}
    for i, wp in enumerate(risk_map_data['waypoint_ids']):
        risk_map[wp] = {
            'PD': PDs_matrix[i].tolist(),
            'BAC': BACs_matrix[i].tolist()
        }
    return risk_map


def heuristic(a, b):
    ALPHA = 1
    BETA = 1
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    distance_cost = ((x1 - x2)**2 + (y1 - y2)**2)**0.5
    
    # Include PD cost in the heuristic
    PD_a = RISK_MAP[a]['PD'][0]
    PD_b = RISK_MAP[b]['PD'][0]
    avg_PD_cost = (PD_a + PD_b) / 2
    
    # Balance factors as per importance
    return ALPHA * distance_cost + BETA * avg_PD_cost


def update_G_weights(risk_map, g, robot_speed=0.5):   
    max_PD = max([pd for node in G.nodes() if node != constants.WP.CHARGING_STATION.value for pd in risk_map[node]['PD']])

    ALPHA = 1
    BETA = 2 
    
    # Get the position information from the graph
    pos = nx.get_node_attributes(G, 'pos')
    
    travel_distances = []
    pd_costs = []
    pd_null = []
    for u, v in g.edges():
        # Calculate travel distance between nodes u and v
        (x1, y1) = pos[u]
        (x2, y2) = pos[v]
        travel_distance = ((x1 - x2)**2 + (y1 - y2)**2)**0.5
        travel_distances.append(travel_distance)

        if u != constants.WP.CHARGING_STATION.value and v != constants.WP.CHARGING_STATION.value:
            if u == constants.WP.DOOR_CORRIDOR2.value or v == constants.WP.DOOR_CORRIDOR2.value:
                PD_cost = max_PD
            else:
                # Estimate time it will take to reach the next node (simple time = distance / speed)
                (xr, yr) = pos[ROBOT_CLOSEST_WP]
                u_PD_ttr = ((x1 - xr)**2 + (y1 - yr)**2)**0.5 / robot_speed  # Time in seconds to move from r to u
                v_PD_ttr = ((x2 - xr)**2 + (y2 - yr)**2)**0.5 / robot_speed  # Time in seconds to move from r to v
                
                # Find the closest time step to target_time based on your time steps
                idx_u = math.ceil(u_PD_ttr / 7)  # Assuming 7 seconds per time step
                idx_v = math.ceil(v_PD_ttr / 7)

                # Bound the indices to make sure they are within valid range (for list access)
                idx_u = min(len(risk_map[u]['PD']) - 1, idx_u)
                idx_v = min(len(risk_map[v]['PD']) - 1, idx_v)
                
                # Get PD values at the estimated times for both u and v
                PD_u = risk_map[u]['PD'][idx_u] 
                PD_v = risk_map[v]['PD'][idx_v] 
                
                # Average PD cost (from current and neighbor nodes) 
                PD_cost = (PD_u + PD_v) / 2
        else:
            PD_cost = 0
        
        pd_costs.append(PD_cost)
        
        if not(u != constants.WP.CHARGING_STATION.value and v != constants.WP.CHARGING_STATION.value):
            pd_null.append(len(pd_costs)-1)
            
    # Normalize travel distances and PD costs
    max_travel = max(travel_distances) if travel_distances else 1
    max_pd_cost = max(pd_costs) if pd_costs else 1

    normalized_travel_distances = [d / max_travel for d in travel_distances]
    normalized_pd_costs = [c / max_pd_cost if pd_costs.index(c) not in pd_null else 1 for c in pd_costs]

    # Apply normalization and scaling factors
    for idx, (u, v) in enumerate(g.edges()):
        travel_distance = normalized_travel_distances[idx]
        PD_cost = normalized_pd_costs[idx]
        
        # Combine the normalized travel cost and PD cost
        weight = ALPHA * travel_distance + BETA * PD_cost
        
        # Assign the combined weight to the edge between u and v
        g[u][v]['weight'] = weight
        
    return g


def get_next_goal():    
    if not rospy.get_param('/hri/robot_busy'):                                        
        return TASK_LIST.pop(0), constants.Task.DELIVERY, True
                       
        
def Plan(p):
    while not ros_utils.wait_for_param("/pnp_ros/ready"):
        rospy.sleep(0.1)
        
    global NEXT_GOAL, QUEUE, G, RISK_MAP
    ros_utils.wait_for_param("/peopleflow/timeday")
    rospy.set_param('/hri/robot_busy', False)
    PLAN_ON = True
    rospy.set_param("/peopleflow/robot_plan_on", PLAN_ON)
    
    # Service proxies for NewTask and FinishTask
    rospy.wait_for_service('/hrisim/new_task')
    rospy.wait_for_service('/hrisim/finish_task')
    new_task_service = rospy.ServiceProxy('/hrisim/new_task', NewTask)
    finish_task_service = rospy.ServiceProxy('/hrisim/finish_task', FinishTask)
    
    while PLAN_ON:               
            
        if len(QUEUE) == 0:
            NEXT_GOAL, TASK, PLAN_ON = get_next_goal()
            if NEXT_GOAL is None: continue
            if isinstance(NEXT_GOAL, constants.WP): NEXT_GOAL = NEXT_GOAL.value
            rospy.logerr(f"New goal defined: {NEXT_GOAL}")
            
            RISK_MAP = get_prediction(p)
            G = update_G_weights(RISK_MAP, G)
            ros_utils.load_graph_to_rosparam(G, "/peopleflow/G")
            update_service = rospy.ServiceProxy('/update_graph_visualization', Trigger)
            _ = update_service()

            QUEUE = nx.astar_path(G, ROBOT_CLOSEST_WP, NEXT_GOAL, heuristic=heuristic, weight='weight')
            
            task_id = new_task_service(NEXT_GOAL, QUEUE).task_id
            rospy.loginfo(f"NewTask service called. Assigned Task ID: {task_id}")            
        
        #! Here the goal is taken from the queue
        if not rospy.get_param('/hri/robot_busy') and len(QUEUE) > 0:
            next_sub_goal = QUEUE.pop(0)
            rospy.logwarn(f"Planning next goal: {next_sub_goal}")
            nextnext_sub_goal = QUEUE[0] if len(QUEUE) > 0 else None
            if nextnext_sub_goal is None and TASK is constants.Task.CLEANING: 
                nextnext_sub_goal = TASK_LIST[constants.Task.CLEANING.value][0] if len(TASK_LIST[constants.Task.CLEANING.value]) > 0 else None
            send_goal(p, next_sub_goal, nextnext_sub_goal)
            rospy.set_param('/hrisim/robot_task', TASK.value)
            if len(QUEUE) == 0: 
                finish_task_service(task_id, 1)  # 1 for success
                rospy.loginfo(f"FinishTask service called for task ID {task_id} with success.")    
    
    rospy.set_param("/peopleflow/robot_plan_on", PLAN_ON)

                                          
    
def cb_robot_closest_wp(wp: String):
    global ROBOT_CLOSEST_WP
    ROBOT_CLOSEST_WP = wp.data
        
    
if __name__ == "__main__":  
    ROBOT_CLOSEST_WP = None
    NEXT_GOAL = None
    QUEUE = []
    
    p = PNPCmd()
        
    TASK_LIST = [constants.WP.OFFICE2]
    
    g_path = ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path")
    with open(g_path, 'rb') as f:
        G = pickle.load(f)
        G.remove_node("parking")
        G_original = copy.deepcopy(G)
    
    rospy.Subscriber("/hrisim/robot_closest_wp", String, cb_robot_closest_wp)

    STATIC_CONSUMPTION = rospy.get_param("/robot_battery/static_consumption")
    K = rospy.get_param("/robot_battery/dynamic_consumption")

    p.begin()

    Plan(p)

    p.end()