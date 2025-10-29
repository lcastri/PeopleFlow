import math
import os
import pickle
import sys
import copy
import json
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
from robot_srvs.srv import NewTask, NewTaskResponse, FinishTask, FinishTaskResponse, SetBattery, SetBatteryResponse
import functools
from std_srvs.srv import Empty  # Import the Empty service
import argparse
import heapq
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

BATTERY_CRITICAL_LEVEL = 20

def send_goal(p, next_dest, nextnext_dest=None, time_threshold=-1, first=False):
    pos = nx.get_node_attributes(G, 'pos')
    x, y = pos[next_dest]
    if nextnext_dest is not None:
        x2, y2 = pos[nextnext_dest]
        angle = math.atan2(y2-y, x2-x)
        inputs = [x, y, angle, time_threshold, 1 if not first and not rospy.get_param('/hrisim/robot_obs', False) else 0]
    else:
        inputs = [x, y, 0, time_threshold, 0]
    p.exec_action('gotoobs', "_".join([str(_input) for _input in inputs]))
    
    
def get_prediction(p):
    p.exec_action('predict', "")

    risk_map_data = rospy.get_param('/hrisim/risk_map')
    tmp_ARCs = risk_map_data['arcs']
    ARCs = [(arc.split("__")[0], arc.split("__")[1]) for arc in tmp_ARCs]
    PDs = risk_map_data['PDs']
    BCs = risk_map_data['BCs']

    risk_map = {}
    for i, arc in enumerate(ARCs):
        risk_map[arc] = {
            'PD': PDs[i],
            'BC': BCs[i]
        }
    return risk_map


def shortest_heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2)**2 + (y1 - y2)**2)**0.5


def causal_heuristic(a, b, max_d_cost, max_pd_cost, max_bc_cost, R_B, current_path):
    
    def _extract_info(a, b, variable):
        if (a, b) in RISK_MAP:
            cost = RISK_MAP[(a, b)][variable]
        elif (b, a) in RISK_MAP:
            cost = RISK_MAP[(b, a)][variable]
        else:
            cost = 0
        return cost
    
    pos = nx.get_node_attributes(G, 'pos')

    # Get coordinates
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]

    # Calculate normalized distance cost
    distance_cost = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    normalized_d_cost = distance_cost / max_d_cost if max_d_cost > 0 else 0

    # Calculate PD cost
    PD_cost = _extract_info(a, b, 'PD')
    normalized_PD_cost = PD_cost / max_pd_cost if max_pd_cost > 0 else 0

    # Calculate BC cost
    BC_cost = _extract_info(a, b, 'BC')
    normalized_BC_cost = BC_cost / max_bc_cost if max_bc_cost > 0 else 0

    # Compute total estimated battery consumption for the path
    total_battery_cost = sum(_extract_info(x, y, 'BC') for x, y in zip(current_path, current_path[1:]))

    # Enforce battery constraint
    if R_B - total_battery_cost < BATTERY_CRITICAL_LEVEL:
        return float('inf')

    # Combine weighted costs
    return K_D * normalized_d_cost + K_PD * normalized_PD_cost + K_BC * normalized_BC_cost


def astar_with_causal_heuristic(G, start, goal, max_d_cost, max_pd_cost, max_bc_cost, R_B):
    """
    Custom A* search that keeps track of paths and applies the causal heuristic correctly.
    """
    # Priority queue: (total cost, current node, path taken so far)
    pq = [(0, start, [start])]

    while pq:
        cost, node, path = heapq.heappop(pq)

        # If we reached the goal, return the path
        if node == goal:
            return path

        # Expand neighbors
        for neighbor in G.neighbors(node):
            if neighbor in path:  # Avoid cycles
                continue

            # Compute heuristic with the current path
            heuristic_func = functools.partial(
                causal_heuristic, 
                max_d_cost=max_d_cost, 
                max_pd_cost=max_pd_cost, 
                max_bc_cost=max_bc_cost, 
                R_B=R_B, 
                current_path=path + [neighbor]  # Update path
            )

            # Get edge weight
            edge_weight = G[node][neighbor].get('weight', 1)

            # Compute heuristic value
            h_value = heuristic_func(node, neighbor)

            # If the heuristic returns infinity (battery constraint), discard the path
            if h_value == float('inf'):
                continue

            # Total cost (g(n) + h(n))
            total_cost = cost + edge_weight + h_value

            # Add to priority queue
            heapq.heappush(pq, (total_cost, neighbor, path + [neighbor]))

    return None


def compute_max_values(G, risk_map):
    pos = nx.get_node_attributes(G, 'pos')
    travel_distances = []

    # Calculate all edge travel distances
    for u, v in G.edges():
        (x1, y1) = pos[u]
        (x2, y2) = pos[v]
        travel_distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
        travel_distances.append(travel_distance)


    max_d_cost = max(travel_distances) if travel_distances else 1
    max_pd_cost = max([risk_map[arc]['PD'] for arc in risk_map.keys()])
    max_bc_cost = max([risk_map[arc]['BC'] for arc in risk_map.keys()])
    rospy.logwarn(f"max_d_cost: {max_d_cost}")
    rospy.logwarn(f"max_pd_cost: {max_pd_cost}") 
    rospy.logwarn(f"max_bc_cost: {max_bc_cost}") 

    return max_d_cost, max_pd_cost, max_bc_cost


def update_G_weights(g, max_d_cost, max_pd_cost, max_bc_cost):
    
    def _extract_info(a, b, variable):
        if (a, b) in RISK_MAP:
            cost = RISK_MAP[(a, b)][variable]
        elif (b, a) in RISK_MAP:
            cost = RISK_MAP[(b, a)][variable]
        else:
            cost = 0
        return cost  
        
    # Get the position information from the graph
    pos = nx.get_node_attributes(g, 'pos')
    
    d_costs = []
    pd_costs = []
    bc_costs = []
    
    for u, v in g.edges():
        # Calculate travel distance between nodes u and v
        (x1, y1) = pos[u]
        (x2, y2) = pos[v]
        d_cost = ((x1 - x2)**2 + (y1 - y2)**2)**0.5
        d_costs.append(d_cost)

        if u != constants.WP.CHARGING_STATION.value and v != constants.WP.CHARGING_STATION.value and ((u, v) in RISK_MAP or (v, u) in RISK_MAP):
            PD_cost = _extract_info(u, v, 'PD')
            BC_cost = _extract_info(u, v, 'BC')
        else:
            PD_cost = max_pd_cost
            BC_cost = max_bc_cost
        
        pd_costs.append(PD_cost)
        bc_costs.append(BC_cost)
            
    normalized_d_costs = [d / max_d_cost for d in d_costs]
    normalized_pd_costs = [c / max_pd_cost for c in pd_costs]
    normalized_bc_costs = [c / max_bc_cost for c in bc_costs]

    # Apply normalization and scaling factors
    for idx, (u, v) in enumerate(g.edges()):
        d_cost = normalized_d_costs[idx]
        PD_cost = normalized_pd_costs[idx]
        BC_cost = normalized_bc_costs[idx]
                
        # Assign the combined weight to the edge between u and v
        g[u][v]['weight'] = K_D * d_cost + K_PD * PD_cost + K_BC * BC_cost
        
    return g


def get_next_goal():
    global TASK_LIST

    if not rospy.get_param('/robot_battery/is_charging') and not rospy.get_param('/hrisim/robot_busy'):
        
        tod = rospy.get_param('/peopleflow/timeday')                                   
        if len(TASK_LIST[tod]) > 0:
            if tod in [constants.TOD.H1.value, constants.TOD.H2.value, constants.TOD.H3.value, 
                    constants.TOD.H4.value, constants.TOD.H5.value, constants.TOD.H7.value, 
                    constants.TOD.H8.value, constants.TOD.H9.value, constants.TOD.H10.value]:
                return TASK_LIST[tod][0], constants.Task.DELIVERY, True
                        
            elif rospy.get_param('/peopleflow/timeday') in [constants.TOD.H6.value]:
                return TASK_LIST[tod][0], constants.Task.DELIVERY, True
                        
            elif rospy.get_param('/peopleflow/timeday') in [constants.TOD.OFF.value]:
                if len(TASK_LIST[tod]) > 0:
                    rospy.logwarn("It's off time, going to clean the shop.")
                    return TASK_LIST[tod][0], constants.Task.CLEANING, True
        else:
            rospy.logwarn("No tasks left, shutting down the planning.")
            return None, None, False
                       
            
# def check_BAC(risk_map, queue, heuristic, robot_speed = 0.5):
#     for wp in queue:
#         if wp == constants.WP.CHARGING_STATION.value and wp not in risk_map: continue
#         time_to_reach_wp = ros_utils.get_time_to_wp(G_original, ROBOT_CLOSEST_WP, wp, heuristic, robot_speed)
#         # time_to_reach_charger = ros_utils.get_time_to_wp(G_original, wp, constants.WP.CHARGING_STATION.value, shortest_heuristic, robot_speed)
#         battery_consumption = time_to_reach_wp * (STATIC_CONSUMPTION + K * robot_speed)
        
#         wp_BAC_idx = math.ceil(time_to_reach_wp / PRED_STEP)
#         wp_BAC_idx = min(len(risk_map[wp]['BAC']) - 1, wp_BAC_idx)
#         wp_BAC = risk_map[wp]['BAC'][wp_BAC_idx] - battery_consumption
#         if wp_BAC >= BATTERY_CRITICAL_LEVEL:
#             rospy.logwarn(f"{wp} ttwp: {wp_BAC_idx} BWP: {risk_map[wp]['BAC'][wp_BAC_idx]} BTC: {battery_consumption}.")
#             rospy.logwarn(f"{wp} BAC: {wp_BAC}. Critical? False")
#         else:
#             rospy.logerr(f"{wp} ttwp: {wp_BAC_idx} BWP: {risk_map[wp]['BAC'][wp_BAC_idx]} BTC: {battery_consumption}.")
#             rospy.logerr(f"{wp} BAC: {wp_BAC}. Critical? True")
#             return True
        
#     return False

def set_battery(b):
    try:
        response = set_battery_level(b)
        if response.success:
            rospy.loginfo("Battery successfully set to 100%.")
        else:
            rospy.logwarn("Failed to set battery level.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call to set_battery_level failed: {e}")
        
        
def set_robot_pos(dest):
    global ROBOT_CLOSEST_WP
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        pos = nx.get_node_attributes(G, 'pos')
        x, y = pos[dest]
        yaw = 0
        
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state_msg = ModelState()
        state_msg.model_name = 'tiago'

        # Set the robot's position and orientation from input
        state_msg.pose.position.x = float(x)
        state_msg.pose.position.y = float(y)
        state_msg.pose.position.z = 0
        state_msg.pose.orientation.z = math.sin(float(yaw)/2)
        state_msg.pose.orientation.w = math.cos(float(yaw)/2)
        state_msg.reference_frame = 'world'
        set_model_state(state_msg)

        # Publish the pose to /initialpose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.pose = state_msg.pose

        # Optional: Set the covariance to indicate confidence in this estimate
        pose_msg.pose.covariance[0] = 0.25  # x covariance
        pose_msg.pose.covariance[7] = 0.25  # y covariance
        pose_msg.pose.covariance[35] = 0.0685  # yaw covariance (approx. 5 degrees)

        initial_pose_pub.publish(pose_msg)
        ROBOT_CLOSEST_WP = dest
        rospy.sleep(2)
        rospy.logwarn(f"Robot position and orientation set: {dest}!")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)   
        
def Plan(p):
    while not ros_utils.wait_for_param("/pnp_ros/ready"):
        rospy.sleep(0.1)
        
    global NEXT_GOAL, QUEUE, GO_TO_CHARGER, G, RISK_MAP, TASK_LIST, set_battery_level, dynobs_remove_service, dynobs_timer_service
    # rospy.logwarn("Waiting rosservice /hrisim/new_task to be ready...")
    # rospy.wait_for_service('/hrisim/new_task')
    # rospy.logwarn("Waiting rosservice /hrisim/finish_task to be ready...")
    # rospy.wait_for_service('/hrisim/finish_task')
    # rospy.logwarn("Waiting rosservice /hrisim/shutdown to be ready...")
    # rospy.wait_for_service('/hrisim/shutdown')
    # rospy.logwarn("Waiting rosservice /hrisim/set_battery_level to be ready...")
    # rospy.wait_for_service('/hrisim/set_battery_level')
    # rospy.logwarn("Waiting rosservice /hrisim/obstacles/remove to be ready...")
    # rospy.wait_for_service('/hrisim/obstacles/remove')
    # rospy.logwarn("Waiting rosservice /hrisim/obstacles/timer/off to be ready...")
    # rospy.wait_for_service('/hrisim/obstacles/timer/off')
    # rospy.logwarn("Waiting rosservice /hrisim/riskMap/predict to be ready...")
    # rospy.wait_for_service('/hrisim/riskMap/predict')
    # rospy.logwarn("Waiting rosservice /update_graph_visualization to be ready...")
    # rospy.wait_for_service('/update_graph_visualization')
    ros_utils.wait_for_service('/hrisim/new_task')
    ros_utils.wait_for_service('/hrisim/finish_task')
    ros_utils.wait_for_service('/hrisim/shutdown')
    ros_utils.wait_for_service('/hrisim/set_battery_level')
    ros_utils.wait_for_service('/hrisim/obstacles/remove')
    ros_utils.wait_for_service('/hrisim/obstacles/timer/off')
    ros_utils.wait_for_service('/hrisim/riskMap/predict')
    ros_utils.wait_for_service('/update_graph_visualization')

    new_task_service = rospy.ServiceProxy('/hrisim/new_task', NewTask)
    finish_task_service = rospy.ServiceProxy('/hrisim/finish_task', FinishTask)
    shutdown_service = rospy.ServiceProxy('/hrisim/shutdown', Empty)
    set_battery_level = rospy.ServiceProxy('/hrisim/set_battery_level', SetBattery)
    dynobs_remove_service = rospy.ServiceProxy('/hrisim/obstacles/remove', Empty)
    dynobs_timer_service = rospy.ServiceProxy('/hrisim/obstacles/timer/off', Empty) 
    update_service = rospy.ServiceProxy('/update_graph_visualization', Empty)
   
    ros_utils.wait_for_param("/hrisim/prediction_ready")
    
    rospy.logwarn("Waiting PeopleFlow timeday to be ready...")
    while ros_utils.wait_for_param("/peopleflow/timeday") != INIT_TIME: 
        rospy.sleep(0.1)    
    set_battery(INIT_BATTERY)
    rospy.set_param('/hrisim/tasks/total', len(TASK_LIST[ros_utils.wait_for_param("/peopleflow/timeday")]))
    rospy.set_param('/hrisim/robot_busy', False)
    no_prediction = False
    PLAN_ON = True
    TASK_ON = False
    GO_TO_CHARGER = False
    
    rospy.logwarn("Ready for the plan!")
    while PLAN_ON:               
        if GO_TO_CHARGER:
            if TASK_ON:
                rospy.logerr(f"Task {task_id} fail for critical battery")
                finish_task_service(task_id, constants.TaskResult.CRITICAL_BATTERY.value)
                TASK_ON = False
                rospy.logwarn("Cancelling all goals..")
                client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
                client.wait_for_server()
                client.cancel_all_goals()
                p.action_cmd('goto', "", 'interrupt')
                while rospy.get_param('/hrisim/robot_busy'): rospy.sleep(0.1)
        
                set_robot_pos(NEXT_GOAL)
                QUEUE = []
            rospy.set_param('/robot_battery/is_charging', True)
            set_battery(100)
            GO_TO_CHARGER = False
            
        elif not rospy.get_param('/robot_battery/is_charging') and not GO_TO_CHARGER and len(QUEUE) == 0:
            NEXT_GOAL, TASK, PLAN_ON = get_next_goal()
            if NEXT_GOAL is None: continue
            if isinstance(NEXT_GOAL, constants.WP): NEXT_GOAL = NEXT_GOAL.value
            rospy.logerr(f"New goal defined: {NEXT_GOAL}")
            
            if not no_prediction:
                RISK_MAP = get_prediction(p)
            if rospy.get_param('/peopleflow/timeday') == constants.TOD.OFF.value and not no_prediction:
                no_prediction = True
            
            # Update weights
            max_d_cost, max_pd_cost, max_bc_cost = compute_max_values(G, RISK_MAP)
            G = update_G_weights(G, max_d_cost, max_pd_cost, max_bc_cost)
            ros_utils.load_graph_to_rosparam(G, "/peopleflow/G")
            update_service()

            QUEUE = astar_with_causal_heuristic(G, ROBOT_CLOSEST_WP, NEXT_GOAL, max_d_cost, max_pd_cost, max_bc_cost, BATTERY_LEVEL)
            if QUEUE is None:
                rospy.logwarn("No path found, going to charger")
                QUEUE = []
                GO_TO_CHARGER = True
                continue
            else:
                rospy.logwarn(f"Path found: {QUEUE}")
            if GO_TO_CHARGER: continue
            
            TASK_LIST[rospy.get_param('/peopleflow/timeday')].pop(0)
            task_id = new_task_service(NEXT_GOAL, QUEUE).task_id
            TASK_ON = True
            firstgoal = QUEUE[0]
        
        #! Here the goal is taken from the queue
        if not rospy.get_param('/hrisim/robot_busy') and len(QUEUE) > 0:
            next_sub_goal = QUEUE.pop(0)
            rospy.logwarn(f"Planning next goal: {next_sub_goal}")
            nextnext_sub_goal = QUEUE[0] if len(QUEUE) > 0 else None
            if nextnext_sub_goal is None and TASK is constants.Task.CLEANING:
                tod = rospy.get_param('/peopleflow/timeday')                                   
                nextnext_sub_goal = TASK_LIST[tod][0] if len(TASK_LIST[tod]) > 0 else None
            
            send_goal(p, next_sub_goal, nextnext_sub_goal, TIME_THRESHOLD, next_sub_goal == firstgoal)
            GOAL_STATUS = rospy.get_param('/hrisim/goal_status')
            if GOAL_STATUS == -1:
                rospy.logerr("Goal failed!")
                finish_task_service(task_id, constants.TaskResult.FAILURE.value)
                TASK_ON = False
                set_robot_pos(NEXT_GOAL)
                QUEUE = []
                continue
            rospy.set_param('/hrisim/goal_status', 0)
            
            if len(QUEUE) == 0: 
                finish_task_service(task_id, constants.TaskResult.SUCCESS.value)
                TASK_ON = False
                
    shutdown_service()
                                   
def cb_battery(msg):
    global BATTERY_LEVEL, GO_TO_CHARGER
    BATTERY_LEVEL = float(msg.level.data)
    if not GO_TO_CHARGER and not rospy.get_param('/robot_battery/is_charging') and BATTERY_LEVEL <= 20:        
        GO_TO_CHARGER = True
        
    elif BATTERY_LEVEL == 100 and rospy.get_param('/robot_battery/is_charging'):
        rospy.set_param('/robot_battery/is_charging', False)
        rospy.logwarn("Battery is already full, not planning any tasks.")
        
    
def cb_robot_closest_wp(wp: String):
    global ROBOT_CLOSEST_WP, task_pub
    ROBOT_CLOSEST_WP = wp.data
    
    
def cb_odom(odom: Odometry):
    v = abs(odom.twist.twist.linear.x)
    if (rospy.get_param('/hrisim/robot_obs', False) and v >= 0.5):
        dynobs_remove_service()
        rospy.set_param('/hrisim/robot_obs', False)
        dynobs_timer_service()
    
    
if __name__ == "__main__":  
    BATTERY_LEVEL = None
    ROBOT_CLOSEST_WP = None
    NEXT_GOAL = None
    GO_TO_CHARGER = False
    QUEUE = []
    rospy.set_param('/hrisim/robot_obs', False)

    
    PRED_STEP = 5
    K_D = 7.5
    K_PD = 10
    K_BC = 10
    global TASK_LIST

    p = PNPCmd()
        
    TLISTPATH = '/root/ros_ws/src/HRISim/hrisim_plans/hardcoded/task_list.json'
    with open(TLISTPATH, 'r') as f:
        TASK_LIST = json.load(f)
    
    g_path = ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path")
    with open(g_path, 'rb') as f:
        G = pickle.load(f)
        G.remove_node("parking")
    ros_utils.load_graph_to_rosparam(G, "/peopleflow/G")
    rospy.Subscriber("/hrisim/robot_battery", BatteryStatus, cb_battery)
    rospy.Subscriber("/hrisim/robot_closest_wp", String, cb_robot_closest_wp)
    rospy.Subscriber("/mobile_base_controller/odom", Odometry, cb_odom)
    initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)

    # STATIC_CONSUMPTION = rospy.get_param("/robot_battery/static_consumption")
    # K = rospy.get_param("/robot_battery/dynamic_consumption")
    TIME_THRESHOLD = ros_utils.wait_for_param("/hrisim/abort_time_threshold")

    parser = argparse.ArgumentParser(description='Initialize robot parameters.')
    parser.add_argument('--init_time', type=str, required=True, help='Initial time to start the plan.')
    parser.add_argument('--init_battery', type=float, required=True, help='Initial battery level.')

    args = parser.parse_args()
    INIT_TIME = args.init_time
    INIT_BATTERY = args.init_battery
    
    p.begin()

    Plan(p)

    p.end()