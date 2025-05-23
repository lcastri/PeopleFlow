import math
import os
import pickle
import random
import sys
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
from robot_srvs.srv import NewTask, FinishTask, VisualisePath
from std_srvs.srv import Empty  # Import the Empty service
from nav_msgs.msg import Odometry


WORKING_TOP_TARGETS = [constants.WP.TARGET_1.value, constants.WP.TARGET_2.value, constants.WP.TARGET_3.value]
WORKING_BOTTOM_TARGETS = [constants.WP.TARGET_4.value, constants.WP.TARGET_5.value, constants.WP.TARGET_6.value]
LUNCH_TARGETS = [constants.WP.ENTRANCE.value, constants.WP.TARGET_7.value]


# def send_goal(p, next_dest, nextnext_dest=None):
#     pos = nx.get_node_attributes(G, 'pos')
#     x, y = pos[next_dest]
#     if nextnext_dest is not None:
#         x2, y2 = pos[nextnext_dest]
#         angle = math.atan2(y2-y, x2-x)
#         coords = [x, y, angle]
#     else:
#         coords = [x, y, 0]
#     p.exec_action('goto', "_".join([str(coord) for coord in coords]))
def send_goal(p, next_dest, nextnext_dest=None, first=False):
    global OBS_SPAWNED_TIME
    pos = nx.get_node_attributes(G, 'pos')
    x, y = pos[next_dest]
    if nextnext_dest is not None:
        x2, y2 = pos[nextnext_dest]
        angle = math.atan2(y2-y, x2-x)
        inputs = [x, y, angle, TIME_THRESHOLD, 1 if not first and not rospy.get_param('/hrisim/robot_obs', False) else 0]
    else:
        inputs = [x, y, 0, TIME_THRESHOLD, 0]
    p.exec_action('gotoobs', "_".join([str(input) for input in inputs]))
    
    
def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def get_next_goal():
    global ROBOT_CLOSEST_WP, CLEANING_PATH
    
    if not rospy.get_param('/robot_battery/is_charging') and not rospy.get_param('/hrisim/robot_busy'):     
        if rospy.get_param('/peopleflow/timeday') in [constants.TOD.H1.value, constants.TOD.H2.value, constants.TOD.H3.value, 
                                                      constants.TOD.H4.value, constants.TOD.H5.value, constants.TOD.H7.value, 
                                                      constants.TOD.H8.value, constants.TOD.H9.value, constants.TOD.H10.value]:
            if ROBOT_CLOSEST_WP in WORKING_TOP_TARGETS:
                return random.choice(WORKING_BOTTOM_TARGETS), constants.Task.DELIVERY, True
            else:
                return random.choice(WORKING_TOP_TARGETS), constants.Task.DELIVERY, True
                    
        elif rospy.get_param('/peopleflow/timeday') in [constants.TOD.H6.value]:
            if ROBOT_CLOSEST_WP == constants.WP.ENTRANCE.value:
                return constants.WP.TARGET_7.value, constants.Task.DELIVERY, True
            else:
                return constants.WP.ENTRANCE.value, constants.Task.DELIVERY, True
            
        elif rospy.get_param('/peopleflow/timeday') in [constants.TOD.OFF.value]:
            if len(CLEANING_PATH) > 0:
                rospy.logwarn("It's off time, going to clean the shop.")
                return CLEANING_PATH.pop(0), constants.Task.CLEANING, True
            else:
                rospy.logwarn("No cleaning tasks left, shutting down the planning.")
                return None, None, False
        
        
def Plan(p):
    while not ros_utils.wait_for_param("/pnp_ros/ready"):
        rospy.sleep(0.1)
        
    global NEXT_GOAL, QUEUE, GO_TO_CHARGER, dynobs_remove_service, dynobs_timer_service
    
    ros_utils.wait_for_service('/hrisim/new_task')
    ros_utils.wait_for_service('/hrisim/finish_task')
    ros_utils.wait_for_service('/hrisim/shutdown')
    ros_utils.wait_for_service('/hrisim/obstacles/remove')
    ros_utils.wait_for_service('/hrisim/obstacles/timer/off')
    ros_utils.wait_for_service('/graph/path/show')

    new_task_service = rospy.ServiceProxy('/hrisim/new_task', NewTask)
    finish_task_service = rospy.ServiceProxy('/hrisim/finish_task', FinishTask)
    dynobs_remove_service = rospy.ServiceProxy('/hrisim/obstacles/remove', Empty)
    dynobs_timer_service = rospy.ServiceProxy('/hrisim/obstacles/timer/off', Empty)
    graph_path_show = rospy.ServiceProxy('/graph/path/show', VisualisePath)
    
    
    ros_utils.wait_for_param("/peopleflow/timeday")
    rospy.set_param('/hrisim/robot_busy', False)
    PLAN_ON = True
    rospy.set_param("/peopleflow/robot_plan_on", PLAN_ON)
    
    while PLAN_ON:
        rospy.logerr("Planning..")
        if GO_TO_CHARGER:
            NEXT_GOAL = constants.WP.CHARGING_STATION
            PLAN_ON = True
            QUEUE = nx.astar_path(G, ROBOT_CLOSEST_WP, NEXT_GOAL.value, heuristic=heuristic, weight='weight')
            while QUEUE:
                current_wp = QUEUE.pop(0)
                next_wp = QUEUE[0] if QUEUE else None
                send_goal(p, current_wp, next_wp)
            GO_TO_CHARGER = False
            TASK = constants.Task.CHARGING
            finish_task_service(task_id, constants.TaskResult.FAILURE.value)  # 1 for success
            rospy.set_param('/robot_battery/is_charging', True)
            rospy.logwarn("Battery charging..")
            
        elif not rospy.get_param('/robot_battery/is_charging') and not GO_TO_CHARGER and len(QUEUE) == 0:
            NEXT_GOAL, TASK, PLAN_ON = get_next_goal()
            if NEXT_GOAL is None: continue
            QUEUE = nx.astar_path(G, ROBOT_CLOSEST_WP, NEXT_GOAL, heuristic=heuristic, weight='weight')
            firstgoal = QUEUE[0]

            rospy.logwarn(f"{QUEUE}")
            graph_path_show(','.join(QUEUE))

            task_id = new_task_service(NEXT_GOAL, QUEUE).task_id

        
        #! Here the goal is taken from the queue
        if not rospy.get_param('/hrisim/robot_busy') and len(QUEUE) > 0:
            next_sub_goal = QUEUE.pop(0)
            rospy.logwarn(f"Planning next goal: {next_sub_goal}")
            nextnext_sub_goal = QUEUE[0] if len(QUEUE) > 0 else None
            if nextnext_sub_goal is None and TASK is constants.Task.CLEANING: 
                nextnext_sub_goal = CLEANING_PATH[0] if len(CLEANING_PATH) > 0 else None
                
            send_goal(p, next_sub_goal, nextnext_sub_goal, next_sub_goal == firstgoal)
            
            # Publish +1 when reaching the final goal
            if len(QUEUE) == 0:
                finish_task_service(task_id, constants.TaskResult.SUCCESS.value)  # 1 for success

        
    rospy.set_param("/peopleflow/robot_plan_on", PLAN_ON)

                                   
def cb_battery(msg):
    global BATTERY_LEVEL, QUEUE, NEXT_GOAL, GO_TO_CHARGER
    BATTERY_LEVEL = float(msg.level.data)
    if not GO_TO_CHARGER and not rospy.get_param('/robot_battery/is_charging') and BATTERY_LEVEL <= 20:
        rospy.logwarn("Cancelling all goals..")
        client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        client.wait_for_server()
        client.cancel_all_goals()
        p.action_cmd('goto', "", 'interrupt')
        while rospy.get_param('/hrisim/robot_busy'): rospy.sleep(0.1)
        NEXT_GOAL = None
        QUEUE = []
        GO_TO_CHARGER = True
        
    elif BATTERY_LEVEL == 100 and rospy.get_param('/robot_battery/is_charging'):
        rospy.set_param('/robot_battery/is_charging', False)
        rospy.logwarn("Battery is already full, not planning any tasks.")
        
    
def cb_robot_closest_wp(wp: String):
    global ROBOT_CLOSEST_WP
    ROBOT_CLOSEST_WP = wp.data
    
    
def cb_odom(odom: Odometry):
    global OBS_SPAWNED_TIME
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
    
    p = PNPCmd()
    
    g_path = ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path")
    with open(g_path, 'rb') as f:
        G = pickle.load(f)
        G.remove_node("parking")
    CLEANING_PATH = nx.approximation.traveling_salesman_problem(G, cycle=False)
    rospy.Subscriber("/hrisim/robot_battery", BatteryStatus, cb_battery)
    rospy.Subscriber("/hrisim/robot_closest_wp", String, cb_robot_closest_wp)
    rospy.Subscriber("/mobile_base_controller/odom", Odometry, cb_odom)

    TIME_THRESHOLD = ros_utils.wait_for_param("/hrisim/abort_time_threshold")

    p.begin()

    Plan(p)

    p.end()