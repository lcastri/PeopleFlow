import math
import os
import pickle
import sys
import json
import rospy
try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import pnp_cmd_ros
from pnp_cmd_ros import *
from robot_msgs.msg import BatteryStatus
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction
import actionlib
import hrisim_util.ros_utils as ros_utils
import hrisim_util.constants as constants
import networkx as nx
from robot_srvs.srv import NewTask, FinishTask, SetBattery, VisualisePath
from std_srvs.srv import Empty
import functools
import argparse
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import time

BATTERY_CRITICAL_LEVEL = 20


def extract_info(a, b, variable):
    #! Trick to remove
    magic_arc = (constants.WP.CORRIDOR_CANTEEN_1.value, constants.WP.DOOR_CANTEEN.value) if (constants.WP.CORRIDOR_CANTEEN_1.value, constants.WP.DOOR_CANTEEN.value) in RISK_MAP else (constants.WP.DOOR_CANTEEN.value, constants.WP.CORRIDOR_CANTEEN_1.value)
    if (a, b) in RISK_MAP:
        if a in [constants.WP.CORRIDOR_CANTEEN_2.value, constants.WP.CORRIDOR_CANTEEN_3.value, constants.WP.CORRIDOR_CANTEEN_4.value] or \
           b in [constants.WP.CORRIDOR_CANTEEN_2.value, constants.WP.CORRIDOR_CANTEEN_3.value, constants.WP.CORRIDOR_CANTEEN_4.value]:
            cost = RISK_MAP[magic_arc][variable]
        else:
            cost = RISK_MAP[(a, b)][variable]
    elif (b, a) in RISK_MAP:
        if a in [constants.WP.CORRIDOR_CANTEEN_2.value, constants.WP.CORRIDOR_CANTEEN_3.value, constants.WP.CORRIDOR_CANTEEN_4.value] or \
           b in [constants.WP.CORRIDOR_CANTEEN_2.value, constants.WP.CORRIDOR_CANTEEN_3.value, constants.WP.CORRIDOR_CANTEEN_4.value]:
            cost = RISK_MAP[magic_arc][variable]
        else:
            cost = RISK_MAP[(b, a)][variable]
    else:
        cost = 0
    return cost


def causal_heuristic(a, b, max_d_cost, max_pd_cost, max_bc_cost, k_d, k_pd, k_bc):
    """
    The A* heuristic function.
    Now dynamically receives k_d, k_pd, and k_bc as arguments.
    """    
    pos = nx.get_node_attributes(G, 'pos')

    # Get coordinates
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]

    # Calculate D cost
    D_cost = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    D_cost = D_cost / max_d_cost if max_d_cost > 0 else 0 #! Normalised

    # Calculate PD cost
    PD_cost = extract_info(a, b, 'PD')
    PD_cost = PD_cost / max_pd_cost if max_pd_cost > 0 else 0 #! Normalised

    # Calculate BC cost
    BC_cost = extract_info(a, b, 'BC')
    BC_cost = BC_cost / max_bc_cost if max_bc_cost > 0 else 0 #! Normalised

    # Combine weighted costs using the passed-in K values
    return k_d * D_cost + k_pd * PD_cost + k_bc * BC_cost


def compute_max_values(G, risk_map):
    pos = nx.get_node_attributes(G, 'pos')
    travel_distances = []
    arc_list = []
    # Calculate all edge travel distances
    for u, v in G.edges():
        (x1, y1) = pos[u]
        (x2, y2) = pos[v]
        travel_distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
        travel_distances.append(travel_distance)
        arc_list.append((u, v))


    if travel_distances:
        max_d_index = max(range(len(travel_distances)), key=travel_distances.__getitem__)
        max_d_cost = travel_distances[max_d_index]
    else:
        max_d_cost = 1
        max_d_index = None

    if risk_map:
        max_pd_arc = max(risk_map, key=lambda arc: risk_map[arc]['PD'])
        max_pd_cost = risk_map[max_pd_arc]['PD']
        
        max_bc_arc = max(risk_map, key=lambda arc: risk_map[arc]['BC'])
        max_bc_cost = risk_map[max_bc_arc]['BC']
    else:
        max_pd_cost = 1
        max_pd_arc = None
        max_bc_cost = 1
        max_bc_arc = None

    # --- Log the results with arc info ---
    rospy.logwarn(f"Max costs:"
                    f"\n- D: {max_d_cost} (arc: {arc_list[max_d_index]}),"
                    f"\n- PD: {max_pd_cost} (arc: {max_pd_arc}),"
                    f"\n- BC: {max_bc_cost} (arc: {max_bc_arc})")

    return max_d_cost, max_pd_cost, max_bc_cost


def update_G_weights(g, max_d_cost, max_pd_cost, max_bc_cost, k_d, k_pd, k_bc):
    """
    Updates the graph edge weights.
    Now dynamically receives k_d, k_pd, and k_bc as arguments.
    """        
    # Get the position information from the graph
    pos = nx.get_node_attributes(g, 'pos')
    
    D_costs = []
    PD_costs = []
    BC_costs = []
    
    for u, v in g.edges():
        # Calculate travel distance between nodes u and v
        (x1, y1) = pos[u]
        (x2, y2) = pos[v]
        D_cost = ((x1 - x2)**2 + (y1 - y2)**2)**0.5
        D_costs.append(D_cost)

        if u != constants.WP.CHARGING_STATION.value and v != constants.WP.CHARGING_STATION.value and ((u, v) in RISK_MAP or (v, u) in RISK_MAP):
            PD_cost = extract_info(u, v, 'PD')
            BC_cost = extract_info(u, v, 'BC')
        else:
            PD_cost = max_pd_cost
            BC_cost = max_bc_cost
        
        PD_costs.append(PD_cost)
        BC_costs.append(BC_cost)
    
    #! Normalised   
    D_costs = [d / max_d_cost for d in D_costs]
    PD_costs = [pd / max_pd_cost for pd in PD_costs]
    BC_costs = [bc / max_bc_cost for bc in BC_costs]

    for idx, (u, v) in enumerate(g.edges()):
        D_cost = D_costs[idx]
        PD_cost = PD_costs[idx]
        BC_cost = BC_costs[idx]
                
        # Assign the combined weight to the edge between u and v
        g[u][v]['weight'] = k_d * D_cost + k_pd * PD_cost + k_bc * BC_cost
        g[u][v]['D_cost'] = k_d * D_cost
        g[u][v]['PD_cost'] = k_pd * PD_cost
        g[u][v]['BC_cost'] = k_bc * BC_cost
        
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
        
    global NEXT_GOAL, QUEUE, GO_TO_CHARGER, G, RISK_MAP, TASK_LIST, set_battery_level
    ros_utils.wait_for_param('/hrisim/weights/k_d')
    ros_utils.wait_for_param('/hrisim/weights/k_pd')
    ros_utils.wait_for_param('/hrisim/weights/k_bc')
    ros_utils.wait_for_service('/hrisim/new_task')
    ros_utils.wait_for_service('/hrisim/finish_task')
    ros_utils.wait_for_service('/hrisim/shutdown')
    ros_utils.wait_for_service('/hrisim/set_battery_level')
    ros_utils.wait_for_service('/hrisim/riskMap/predict')
    ros_utils.wait_for_service('/graph/weights/update')
    ros_utils.wait_for_service('/graph/path/show')

    new_task_service = rospy.ServiceProxy('/hrisim/new_task', NewTask)
    finish_task_service = rospy.ServiceProxy('/hrisim/finish_task', FinishTask)
    shutdown_service = rospy.ServiceProxy('/hrisim/shutdown', Empty)
    set_battery_level = rospy.ServiceProxy('/hrisim/set_battery_level', SetBattery)
    graph_weight_update = rospy.ServiceProxy('/graph/weights/update', Empty)
    graph_path_show = rospy.ServiceProxy('/graph/path/show', VisualisePath)
   
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
                RISK_MAP, tot_inf_time, mean_inf_time = ros_utils.get_prediction(p)
            else:
                tot_inf_time = 0.0
                mean_inf_time = 0.0
            if rospy.get_param('/peopleflow/timeday') == constants.TOD.OFF.value and not no_prediction:
                no_prediction = True

            # Update weights
            K_D = rospy.get_param('/hrisim/weights/k_d')
            K_PD = rospy.get_param('/hrisim/weights/k_pd')
            K_BC = rospy.get_param('/hrisim/weights/k_bc')
            rospy.logwarn(f"[Planner] Using weights: K_D={K_D}, K_PD={K_PD}, K_BC={K_BC}")
            max_d_cost, max_pd_cost, max_bc_cost = compute_max_values(G, RISK_MAP)
            # Pass the fetched K-values to the update function
            G = update_G_weights(G, max_d_cost, max_pd_cost, max_bc_cost, K_D, K_PD, K_BC)
            ros_utils.load_graph_to_rosparam(G, "/peopleflow/G")
            graph_weight_update()
            
            causal_heuristic_predefined = functools.partial(
                causal_heuristic, 
                max_d_cost=max_d_cost, 
                max_pd_cost=max_pd_cost, 
                max_bc_cost=max_bc_cost,
                k_d=K_D,
                k_pd=K_PD,
                k_bc=K_BC
            )

            heuristic_wrapper = ros_utils.HeuristicCounter(causal_heuristic_predefined)
            heuristic_wrapper.reset()
            evaluations = 0
            # Step 1: Run A* to find the best path based on distance, people density, and battery cost
            try:
                start_time = time.perf_counter()
                QUEUE = nx.astar_path(G, ROBOT_CLOSEST_WP, NEXT_GOAL, heuristic=heuristic_wrapper, weight='weight')
                end_time = time.perf_counter()
                planning_time = end_time - start_time
                evaluations = heuristic_wrapper.get_count()            
            except nx.NetworkXNoPath:
                raise ValueError("No valid path found by A*!")
                
            # Step 2: Check the battery consumption of the chosen path
            if QUEUE:
                total_battery_cost = sum(RISK_MAP.get((a, b), {}).get('BC', 0) for a, b in zip(QUEUE, QUEUE[1:]))

                # Step 3: Enforce the battery constraint AFTER path selection
                if BATTERY_LEVEL - total_battery_cost < BATTERY_CRITICAL_LEVEL:
                    rospy.logwarn("Path violates battery safety constraint! Going to charger")
                    QUEUE = []
                    GO_TO_CHARGER = True
                    continue
                else:
                    rospy.logwarn(f"Path found: {QUEUE}")
            
            graph_path_show(','.join(QUEUE))
            TASK_LIST[rospy.get_param('/peopleflow/timeday')].pop(0)
            task_id = new_task_service(NEXT_GOAL, QUEUE, tot_inf_time, mean_inf_time, planning_time, evaluations).task_id
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
            
            ros_utils.send_goal(p, G, next_sub_goal, nextnext_sub_goal, TIME_THRESHOLD, obs = False, first = next_sub_goal == firstgoal)
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
       
    
if __name__ == "__main__":  
    BATTERY_LEVEL = None
    ROBOT_CLOSEST_WP = None
    NEXT_GOAL = None
    GO_TO_CHARGER = False
    QUEUE = []
    rospy.set_param('/hrisim/robot_obs', False)

    
    PRED_STEP = 5
    global TASK_LIST

    p = PNPCmd()
        
    TLISTPATH = '/home/hrisim/ros_ws/src/HRISim/hrisim_plans/hardcoded/task_list.json'
    with open(TLISTPATH, 'r') as f:
        TASK_LIST = json.load(f)
    
    g_path = ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path")
    with open(g_path, 'rb') as f:
        G = pickle.load(f)
        G.remove_node("parking")
    ros_utils.load_graph_to_rosparam(G, "/peopleflow/G")
    rospy.Subscriber("/hrisim/robot_battery", BatteryStatus, cb_battery)
    rospy.Subscriber("/hrisim/robot_closest_wp", String, cb_robot_closest_wp)
    initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)

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