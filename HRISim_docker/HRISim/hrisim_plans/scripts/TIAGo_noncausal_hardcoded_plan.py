import math
import os
import pickle
import json
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
from peopleflow_msgs.msg import Time as pT
from robot_srvs.srv import NewTask, NewTaskResponse, FinishTask, FinishTaskResponse, SetBattery, SetBatteryResponse
from std_srvs.srv import Empty  # Import the Empty service
import argparse

from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


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
    
    
def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def get_next_goal():
    global TASK_LIST

    if not rospy.get_param('/robot_battery/is_charging') and not rospy.get_param('/hrisim/robot_busy'):
        
        tod = rospy.get_param('/peopleflow/timeday')                                   
        if len(TASK_LIST[tod]) > 0:
            if tod in [constants.TOD.H1.value, constants.TOD.H2.value, constants.TOD.H3.value, 
                    constants.TOD.H4.value, constants.TOD.H5.value, constants.TOD.H7.value, 
                    constants.TOD.H8.value, constants.TOD.H9.value, constants.TOD.H10.value]:
                return TASK_LIST[tod].pop(0), constants.Task.DELIVERY, True
                        
            elif rospy.get_param('/peopleflow/timeday') in [constants.TOD.H6.value]:
                return TASK_LIST[tod].pop(0), constants.Task.DELIVERY, True
                        
            elif rospy.get_param('/peopleflow/timeday') in [constants.TOD.OFF.value]:
                if len(TASK_LIST[tod]) > 0:
                    rospy.logwarn("It's off time, going to clean the shop.")
                    return TASK_LIST[tod].pop(0), constants.Task.CLEANING, True
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
    rospy.logwarn("Waiting PNP ROS to be ready...")
    while not ros_utils.wait_for_param("/pnp_ros/ready"):
        rospy.sleep(0.1)
        
    global NEXT_GOAL, QUEUE, GO_TO_CHARGER, set_battery_level, dynobs_remove_service, dynobs_timer_service
    rospy.logwarn("Waiting rosservice /hrisim/new_task to be ready...")
    rospy.wait_for_service('/hrisim/new_task')
    rospy.logwarn("Waiting rosservice /hrisim/finish_task to be ready...")
    rospy.wait_for_service('/hrisim/finish_task')
    rospy.logwarn("Waiting rosservice /hrisim/shutdown to be ready...")
    rospy.wait_for_service('/hrisim/shutdown')
    rospy.logwarn("Waiting rosservice /hrisim/set_battery_level to be ready...")
    rospy.wait_for_service('/hrisim/set_battery_level')
    rospy.logwarn("Waiting rosservice /hrisim/obstacles/remove to be ready...")
    rospy.wait_for_service('/hrisim/obstacles/remove')
    rospy.logwarn("Waiting rosservice /hrisim/obstacles/timer/off to be ready...")
    rospy.wait_for_service('/hrisim/obstacles/timer/off')

    new_task_service = rospy.ServiceProxy('/hrisim/new_task', NewTask)
    finish_task_service = rospy.ServiceProxy('/hrisim/finish_task', FinishTask)
    shutdown_service = rospy.ServiceProxy('/hrisim/shutdown', Empty)
    set_battery_level = rospy.ServiceProxy('/hrisim/set_battery_level', SetBattery)
    dynobs_remove_service = rospy.ServiceProxy('/hrisim/obstacles/remove', Empty)
    dynobs_timer_service = rospy.ServiceProxy('/hrisim/obstacles/timer/off', Empty)
        
    rospy.logwarn("Waiting PeopleFlow timeday to be ready...")
    while ros_utils.wait_for_param("/peopleflow/timeday") != INIT_TIME: 
        rospy.sleep(0.1)    
    set_battery(INIT_BATTERY)
    rospy.set_param('/hrisim/tasks/total', len(TASK_LIST[ros_utils.wait_for_param("/peopleflow/timeday")]))
    rospy.set_param('/hrisim/robot_busy', False)
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
            QUEUE = nx.astar_path(G, ROBOT_CLOSEST_WP, NEXT_GOAL, heuristic=heuristic, weight='weight')
            firstgoal = QUEUE[0]
            rospy.logwarn(f"{QUEUE}")
            
            task_id = new_task_service(NEXT_GOAL, QUEUE).task_id
            TASK_ON = True
        
        #! Here the goal is taken from the queue
        if not rospy.get_param('/hrisim/robot_busy') and len(QUEUE) > 0:
            next_sub_goal = QUEUE.pop(0)
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
    global ROBOT_CLOSEST_WP
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

    global TASK_LIST
    
    p = PNPCmd()
    
    TLISTPATH = '/root/ros_ws/src/HRISim/hrisim_plans/hardcoded/task_list.json'
    with open(TLISTPATH, 'r') as f:
        TASK_LIST = json.load(f)
    
    g_path = ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path")
    with open(g_path, 'rb') as f:
        G = pickle.load(f)
        G.remove_node("parking")
    rospy.Subscriber("/hrisim/robot_battery", BatteryStatus, cb_battery)
    rospy.Subscriber("/hrisim/robot_closest_wp", String, cb_robot_closest_wp)
    rospy.Subscriber("/mobile_base_controller/odom", Odometry, cb_odom)
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