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
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2)**2 + (y1 - y2)**2)**0.5


def update_G_weights(risk_map, g, robot_speed=0.5):    
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
    if BATTERY_LEVEL == 100 and rospy.get_param('/robot_battery/is_charging'):
        rospy.logwarn("Battery is already full, not planning any tasks.")
        return None, None
    
    elif not rospy.get_param('/robot_battery/is_charging') and not rospy.get_param('/hri/robot_busy'):                                        
        if TOD <= 5:
            return TASK_LIST[constants.Task.DELIVERY.value].pop(0), constants.Task.DELIVERY, True
                    
        elif 6 <= TOD <= 9:
            rospy.logwarn("It's afternoon or quitting time, going to a random shelf for inventory check.")
            return TASK_LIST[constants.Task.INVENTORY.value].pop(0), constants.Task.INVENTORY, True
                    
        elif TOD >= 10:
            if len(TASK_LIST[constants.Task.CLEANING.value]) > 0:
                rospy.logwarn("It's off time, going to clean the shop.")
                return TASK_LIST[constants.Task.CLEANING.value].pop(0), constants.Task.CLEANING, True
            else:
                rospy.logwarn("No cleaning tasks left, shutting down the planning.")
                return None, None, False
                       
            
def check_BAC(risk_map, queue, robot_speed = 0.5):
    for wp in queue:
        if wp == constants.WP.CHARGING_STATION.value: continue
        time_to_reach_wp = ros_utils.get_time_to_wp(G_original, ROBOT_CLOSEST_WP, wp, heuristic, robot_speed)
        time_to_reach_charger = ros_utils.get_time_to_wp(G_original, wp, constants.WP.CHARGING_STATION.value, heuristic, robot_speed)
        battery_consumption = time_to_reach_charger * (STATIC_CONSUMPTION + K * robot_speed)
        
        wp_BAC_idx = math.ceil(time_to_reach_wp / 7)
        wp_BAC_idx = min(len(risk_map[wp]['BAC']) - 1, wp_BAC_idx)
        wp_BAC = risk_map[wp]['BAC'][wp_BAC_idx] - battery_consumption
        if wp_BAC >= BATTERY_CRITICAL_LEVEL:
            rospy.logwarn(f"{wp} ttwp: {wp_BAC_idx} BWP: {risk_map[wp]['BAC'][wp_BAC_idx]} BTC: {battery_consumption}.")
            rospy.logwarn(f"{wp} BAC: {wp_BAC}. Critical? False")
        else:
            rospy.logerr(f"{wp} ttwp: {wp_BAC_idx} BWP: {risk_map[wp]['BAC'][wp_BAC_idx]} BTC: {battery_consumption}.")
            rospy.logerr(f"{wp} BAC: {wp_BAC}. Critical? True")
            return True
        
    return False
        
def Plan(p):
    while not ros_utils.wait_for_param("/pnp_ros/ready"):
        rospy.sleep(0.1)
        
    global NEXT_GOAL, QUEUE, GO_TO_CHARGER, TASK_ON, G
    ros_utils.wait_for_param("/peopleflow/timeday")
    rospy.set_param('/hri/robot_busy', False)
    PLAN_ON = True
    TASK_ON = 0
    rospy.set_param("/peopleflow/robot_plan_on", PLAN_ON)
    
    # Service proxies for NewTask and FinishTask
    rospy.wait_for_service('/hrisim/new_task')
    rospy.wait_for_service('/hrisim/finish_task')
    new_task_service = rospy.ServiceProxy('/hrisim/new_task', NewTask)
    finish_task_service = rospy.ServiceProxy('/hrisim/finish_task', FinishTask)
    
    while PLAN_ON:               
        if GO_TO_CHARGER:
            NEXT_GOAL = constants.WP.CHARGING_STATION
            TASK = constants.Task.CHARGING
            PLAN_ON = True
            QUEUE = nx.astar_path(G, ROBOT_CLOSEST_WP, NEXT_GOAL.value, heuristic=heuristic, weight='weight')
            while QUEUE:
                current_wp = QUEUE.pop(0)
                next_wp = QUEUE[0] if QUEUE else None
                send_goal(p, current_wp, next_wp)
            GO_TO_CHARGER = False
            rospy.set_param('/robot_battery/is_charging', True)
            rospy.logwarn("Battery charging..")
            TASK_ON = 0
            finish_task_service(task_id, -1)  # -1 for failure
            rospy.loginfo(f"FinishTask service called for task ID {task_id} with success.")  
            NEXT_GOAL = None
            TASK = constants.Task.CHARGING
            
        elif not rospy.get_param('/robot_battery/is_charging') and not GO_TO_CHARGER and TASK_ON == 0:
            if TOD is None: continue
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
            GO_TO_CHARGER = check_BAC(RISK_MAP, QUEUE)
            if GO_TO_CHARGER: continue
            
            task_id = new_task_service(NEXT_GOAL, QUEUE).task_id
            rospy.loginfo(f"NewTask service called. Assigned Task ID: {task_id}")            
        
        #! Here the goal is taken from the queue
        if not rospy.get_param('/hri/robot_busy') and len(QUEUE) > 0:
            TASK_ON = 1
            next_sub_goal = QUEUE.pop(0)
            rospy.logwarn(f"Planning next goal: {next_sub_goal}")
            nextnext_sub_goal = QUEUE[0] if len(QUEUE) > 0 else None
            if nextnext_sub_goal is None and TASK is constants.Task.CLEANING: 
                nextnext_sub_goal = TASK_LIST[constants.Task.CLEANING.value][0] if len(TASK_LIST[constants.Task.CLEANING.value]) > 0 else None
            send_goal(p, next_sub_goal, nextnext_sub_goal)
            rospy.set_param('/hrisim/robot_task', TASK.value)
            if len(QUEUE) == 0: 
                TASK_ON = 0
                finish_task_service(task_id, 1)  # 1 for success
                rospy.loginfo(f"FinishTask service called for task ID {task_id} with success.")    
    
    rospy.set_param("/peopleflow/robot_plan_on", PLAN_ON)

                                   
def cb_battery(msg):
    global BATTERY_LEVEL, QUEUE, NEXT_GOAL, GO_TO_CHARGER
    
    BATTERY_LEVEL = float(msg.level.data)
    if not GO_TO_CHARGER and not rospy.get_param('/robot_battery/is_charging') and BATTERY_LEVEL <= BATTERY_CRITICAL_LEVEL:
        rospy.logwarn("Cancelling all goals..")
        client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        client.wait_for_server()
        client.cancel_all_goals()
        p.action_cmd('goto', "", 'interrupt')
        while rospy.get_param('/hri/robot_busy'): rospy.sleep(0.1)
        NEXT_GOAL = None
        QUEUE = []
        GO_TO_CHARGER = True
        
    elif BATTERY_LEVEL == 100 and rospy.get_param('/robot_battery/is_charging'):
        rospy.set_param('/robot_battery/is_charging', False)
        rospy.logwarn("Battery is already full, not planning any tasks.")
        
    
def cb_robot_closest_wp(wp: String):
    global ROBOT_CLOSEST_WP, task_pub
    ROBOT_CLOSEST_WP = wp.data
    
    task_pub.publish(TASK_ON)

def cb_time(t: pT):
    global TOD
    TOD = int(ros_utils.seconds_to_hh(t.elapsed))
    
    
if __name__ == "__main__":  
    BATTERY_LEVEL = None
    ROBOT_CLOSEST_WP = None
    NEXT_GOAL = None
    GO_TO_CHARGER = False
    QUEUE = []
    TASK_ON = 0
    TOD = None
    
    p = PNPCmd()
        
    TLISTPATH = '/root/ros_ws/src/HRISim/hrisim_plans/hardcoded/task_list.pkl'
    with open(TLISTPATH, 'rb') as f:
        TASK_LIST = pickle.load(f)
    
    g_path = ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path")
    with open(g_path, 'rb') as f:
        G = pickle.load(f)
        G.remove_node("parking")
        G_original = copy.deepcopy(G)
    task_pub = rospy.Publisher('/hrisim/robot_task_status', Int32, queue_size=10)
    
    rospy.Subscriber("/hrisim/robot_battery", BatteryStatus, cb_battery)
    rospy.Subscriber("/hrisim/robot_closest_wp", String, cb_robot_closest_wp)
    rospy.Subscriber("/peopleflow/time", pT, cb_time)

    STATIC_CONSUMPTION = rospy.get_param("/robot_battery/static_consumption")
    K = rospy.get_param("/robot_battery/dynamic_consumption")

    p.begin()

    Plan(p)

    p.end()