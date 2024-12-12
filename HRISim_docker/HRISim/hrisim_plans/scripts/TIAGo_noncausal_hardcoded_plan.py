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
from std_msgs.msg import String, Int32
from move_base_msgs.msg import MoveBaseAction
import actionlib
import hrisim_util.ros_utils as ros_utils
import hrisim_util.constants as constants
import networkx as nx


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
    
    
def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def get_next_goal():
    global BATTERY_LEVEL
    
    if BATTERY_LEVEL == 100 and rospy.get_param('/robot_battery/is_charging'):
        rospy.logwarn("Battery is already full, not planning any tasks.")
        return None, None
    
    elif not rospy.get_param('/robot_battery/is_charging') and not rospy.get_param('/hri/robot_busy'):                                        
        if rospy.get_param('/peopleflow/timeday') <= 5:
            return TASK_LIST[constants.Task.DELIVERY].pop(0), constants.Task.DELIVERY, True
                    
        elif 6 <= rospy.get_param('/peopleflow/timeday') <= 9:
            rospy.logwarn("It's afternoon or quitting time, going to a random shelf for inventory check.")
            return TASK_LIST[constants.Task.INVENTORY].pop(0), constants.Task.INVENTORY, True
                    
        elif rospy.get_param('/peopleflow/timeday') >= 10:
            if len(TASK_LIST[constants.Task.CLEANING]) > 0:
                rospy.logwarn("It's off time, going to clean the shop.")
                return TASK_LIST[constants.Task.CLEANING].pop(0), constants.Task.CLEANING, True
            else:
                rospy.logwarn("No cleaning tasks left, shutting down the planning.")
                return None, None, False
        
        
def Plan(p):
    while not ros_utils.wait_for_param("/pnp_ros/ready"):
        rospy.sleep(0.1)
        
    global wp, NEXT_GOAL, QUEUE, GO_TO_CHARGER, task_pub
    ros_utils.wait_for_param("/peopleflow/timeday")
    rospy.set_param('/hri/robot_busy', False)
    PLAN_ON = True
    rospy.set_param("/peopleflow/robot_plan_on", PLAN_ON)
    
    while PLAN_ON:
        rospy.logerr("Planning..")
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
            
        elif not rospy.get_param('/robot_battery/is_charging') and not GO_TO_CHARGER and len(QUEUE) == 0:
            NEXT_GOAL, TASK, PLAN_ON = get_next_goal()
            if NEXT_GOAL is None: continue
            if isinstance(NEXT_GOAL, constants.WP): NEXT_GOAL = NEXT_GOAL.value
            QUEUE = nx.astar_path(G, ROBOT_CLOSEST_WP, NEXT_GOAL, heuristic=heuristic, weight='weight')
            rospy.logwarn(f"{QUEUE}")
        
        #! Here the goal is taken from the queue
        if not rospy.get_param('/hri/robot_busy') and len(QUEUE) > 0:
            next_sub_goal = QUEUE.pop(0)
            rospy.logwarn(f"Planning next goal: {next_sub_goal}")
            nextnext_sub_goal = QUEUE[0] if len(QUEUE) > 0 else None
            if nextnext_sub_goal is None and TASK is constants.Task.CLEANING: 
                nextnext_sub_goal = TASK_LIST[constants.Task.CLEANING][0] if len(TASK_LIST[constants.Task.CLEANING]) > 0 else None
            send_goal(p, next_sub_goal, nextnext_sub_goal)
            rospy.set_param('/hrisim/robot_task', TASK.value)
            
            # Publish +1 when reaching the final goal
            if len(QUEUE) == 0: task_pub.publish(constants.TaskResult.SUCCESS.value)
        
    rospy.set_param("/peopleflow/robot_plan_on", PLAN_ON)

                                   
def cb_battery(msg):
    global BATTERY_LEVEL, QUEUE, NEXT_GOAL, GO_TO_CHARGER, task_pub
    BATTERY_LEVEL = float(msg.level.data)
    if not GO_TO_CHARGER and not rospy.get_param('/robot_battery/is_charging') and BATTERY_LEVEL <= 20:
        rospy.logwarn("Cancelling all goals..")
        client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        client.wait_for_server()
        client.cancel_all_goals()
        p.action_cmd('goto', "", 'interrupt')
        while rospy.get_param('/hri/robot_busy'): rospy.sleep(0.1)
        NEXT_GOAL = None
        QUEUE = []
        GO_TO_CHARGER = True
        task_pub.publish(constants.TaskResult.FAILURE.value)
        
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
    
    p = PNPCmd()
    
    TLISTPATH = '/home/lcastri/git/PeopleFlow/HRISim_docker/HRISim/hrisim_plans/helper/task_list.pkl'
    with open(TLISTPATH, 'rb') as f:
        TASK_LIST = pickle.load(f)
    
    g_path = ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path")
    with open(g_path, 'rb') as f:
        G = pickle.load(f)
        G.remove_node("parking")
    rospy.Subscriber("/hrisim/robot_battery", BatteryStatus, cb_battery)
    rospy.Subscriber("/hrisim/robot_closest_wp", String, cb_robot_closest_wp)
    task_pub = rospy.Publisher('/hrisim/robot_task_status', Int32, queue_size=10)

    p.begin()

    Plan(p)

    p.end()