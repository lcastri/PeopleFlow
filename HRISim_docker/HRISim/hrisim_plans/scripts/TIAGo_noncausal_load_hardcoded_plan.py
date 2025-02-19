import math
import os
import pickle
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
from std_msgs.msg import String, Bool
from move_base_msgs.msg import MoveBaseAction
import actionlib
import hrisim_util.ros_utils as ros_utils
import hrisim_util.constants as constants
import networkx as nx
from peopleflow_msgs.msg import Time as pT
from robot_srvs.srv import NewTask, NewTaskResponse, FinishTask, FinishTaskResponse
from std_srvs.srv import Empty  # Import the Empty service
from dynamic_reconfigure.client import Client

WORKING_TOP_TARGETS = [constants.WP.TARGET_1.value, constants.WP.TARGET_2.value, constants.WP.TARGET_3.value]
WORKING_BOTTOM_TARGETS = [constants.WP.TARGET_4.value, constants.WP.TARGET_5.value, constants.WP.TARGET_6.value]
LUNCH_TARGETS = [constants.WP.ENTRANCE.value, constants.WP.TARGET_7.value]


def send_goal(p, next_dest, nextnext_dest=None):
    pos = nx.get_node_attributes(G, 'pos')
    x, y = pos[next_dest]
    if nextnext_dest is not None:
        x2, y2 = pos[nextnext_dest]
        angle = math.atan2(y2-y, x2-x)
        coords = [x, y, angle, TIME_THRESHOLD]
    else:
        coords = [x, y, 0, TIME_THRESHOLD]
    p.exec_action('goto', "_".join([str(coord) for coord in coords]))
    
    
def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def get_next_goal():    
    if not rospy.get_param('/robot_battery/is_charging') and not rospy.get_param('/hrisim/robot_busy'):                                        
        if rospy.get_param('/peopleflow/timeday') in [constants.TOD.H1.value, constants.TOD.H2.value, constants.TOD.H3.value, 
                                                      constants.TOD.H4.value, constants.TOD.H5.value, constants.TOD.H7.value, 
                                                      constants.TOD.H8.value, constants.TOD.H9.value, constants.TOD.H10.value]:
            dest, loaded = TASK_LIST[constants.Task.DELIVERY.value].pop(0)
            return dest, loaded, constants.Task.DELIVERY, True
            # return TASK_LIST[constants.Task.DELIVERY.value].pop(0), constants.Task.DELIVERY, True
                    
        elif rospy.get_param('/peopleflow/timeday') in [constants.TOD.H6.value]:
            dest, loaded = TASK_LIST['LUNCH'].pop(0)
            return dest, loaded, constants.Task.DELIVERY, True
                    
        elif rospy.get_param('/peopleflow/timeday') in [constants.TOD.OFF.value]:
            if len(TASK_LIST[constants.Task.CLEANING.value]) > 0:
                rospy.logwarn("It's off time, going to clean the shop.")
                dest, loaded = TASK_LIST[constants.Task.CLEANING.value].pop(0)
                return dest, loaded, constants.Task.CLEANING, True
            else:
                rospy.logwarn("No cleaning tasks left, shutting down the planning.")
                return None, False, None, False
            
            
def set_Load(load):
    client = Client("/move_base/TebLocalPlannerROS")

    if load:
        rospy.set_param("/hrisim/robot_load", True)
        client.update_configuration({"max_vel_x": 0.5})
        # client.update_configuration({"max_vel_x": 0.25})

    else:
        rospy.set_param("/hrisim/robot_load", False)
        client.update_configuration({"max_vel_x": 0.75})
        # client.update_configuration({"max_vel_x": 0.75})
    
    rospy.loginfo("Setting load: " + str(load))
    pub_load.publish(load)

        
def Plan(p):
    while not ros_utils.wait_for_param("/pnp_ros/ready"):
        rospy.sleep(0.1)
        
    global NEXT_GOAL, QUEUE, GO_TO_CHARGER
    ros_utils.wait_for_param("/peopleflow/timeday")
    rospy.set_param('/hrisim/robot_busy', False)
    rospy.set_param("/hrisim/robot_load", False)
    PLAN_ON = True
    rospy.set_param("/peopleflow/robot_plan_on", PLAN_ON)
    
    # Service proxies for NewTask and FinishTask
    rospy.wait_for_service('/hrisim/new_task')
    rospy.wait_for_service('/hrisim/finish_task')
    new_task_service = rospy.ServiceProxy('/hrisim/new_task', NewTask)
    finish_task_service = rospy.ServiceProxy('/hrisim/finish_task', FinishTask)
    shutdown_service = rospy.ServiceProxy('/hrisim/shutdown', Empty)
    
    while PLAN_ON:
        rospy.logerr("Planning..")
        if GO_TO_CHARGER:
            finish_task_service(task_id, constants.TaskResult.CRITICAL_BATTERY.value)
            set_Load(False)
            
            NEXT_GOAL = constants.WP.CHARGING_STATION
            PLAN_ON = True
            QUEUE = nx.astar_path(G, ROBOT_CLOSEST_WP, NEXT_GOAL.value, heuristic=heuristic, weight='weight')
            while QUEUE:
                current_wp = QUEUE.pop(0)
                next_wp = QUEUE[0] if QUEUE else None
                send_goal(p, current_wp, next_wp)
            GO_TO_CHARGER = False
            rospy.set_param('/robot_battery/is_charging', True)
            rospy.logwarn("Battery charging..")
            NEXT_GOAL = None
            TASK = constants.Task.CHARGING
            
            
        elif not rospy.get_param('/robot_battery/is_charging') and not GO_TO_CHARGER and len(QUEUE) == 0:
            NEXT_GOAL, LOADED, TASK, PLAN_ON = get_next_goal()
            if NEXT_GOAL is None: continue
            QUEUE = nx.astar_path(G, ROBOT_CLOSEST_WP, NEXT_GOAL, heuristic=heuristic, weight='weight')
            rospy.logwarn(f"{QUEUE}")
            set_Load(LOADED)
            task_id = new_task_service(NEXT_GOAL, QUEUE).task_id

        
        #! Here the goal is taken from the queue
        if not rospy.get_param('/hrisim/robot_busy') and len(QUEUE) > 0:
            next_sub_goal = QUEUE.pop(0)
            rospy.logwarn(f"Planning next goal: {next_sub_goal}")
            nextnext_sub_goal = QUEUE[0] if len(QUEUE) > 0 else None
            if nextnext_sub_goal is None and TASK is constants.Task.CLEANING:
                nextnext_sub_goal = TASK_LIST[constants.Task.CLEANING.value][0][0] if len(TASK_LIST[constants.Task.CLEANING.value]) > 0 else None
            send_goal(p, next_sub_goal, nextnext_sub_goal)
            GOAL_STATUS = rospy.get_param('/hrisim/goal_status')
            if GOAL_STATUS == -1:
                rospy.logerr("Goal failed!")
                QUEUE = []
                finish_task_service(task_id, constants.TaskResult.FAILURE.value)
                set_Load(False)
                continue
            rospy.set_param('/hrisim/goal_status', 0)
            
            if len(QUEUE) == 0: 
                finish_task_service(task_id, constants.TaskResult.SUCCESS.value)
                set_Load(False)
                
    rospy.set_param("/peopleflow/robot_plan_on", PLAN_ON)
    shutdown_service()             

                                   
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
    
    
if __name__ == "__main__":  
    BATTERY_LEVEL = None
    ROBOT_CLOSEST_WP = None
    NEXT_GOAL = None
    GO_TO_CHARGER = False
    QUEUE = []
    
    p = PNPCmd()
    
    TLISTPATH = '/root/ros_ws/src/HRISim/hrisim_plans/hardcoded/task_list.pkl'
    with open(TLISTPATH, 'rb') as f:
        TASK_LIST = pickle.load(f)
    
    g_path = ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path")
    with open(g_path, 'rb') as f:
        G = pickle.load(f)
        G.remove_node("parking")
    rospy.Subscriber("/hrisim/robot_battery", BatteryStatus, cb_battery)
    rospy.Subscriber("/hrisim/robot_closest_wp", String, cb_robot_closest_wp)
    pub_load = rospy.Publisher('/hrisim/robot_load', Bool, queue_size=10)
    
    TIME_THRESHOLD = ros_utils.wait_for_param("/hrisim/abort_time_threshold")
    
    p.begin()

    Plan(p)

    p.end()