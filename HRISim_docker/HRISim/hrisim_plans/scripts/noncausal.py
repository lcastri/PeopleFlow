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
from std_msgs.msg import String
import hrisim_util.ros_utils as ros_utils
import networkx as nx
from robot_srvs.srv import PathRequest


def send_goal(p, next_dest, nextnext_dest=None):
    pos = nx.get_node_attributes(G, 'pos')
    x, y = pos[next_dest]
    if nextnext_dest is not None:
        x2, y2 = pos[nextnext_dest]
        angle = math.atan2(y2-y, x2-x)
        coords = [x, y, angle, TASK_TIMEOUT]
    else:
        coords = [x, y, 0, TASK_TIMEOUT]
    p.exec_action('goto', "_".join([str(coord) for coord in coords]))
    
        
def Plan(p):
    while not ros_utils.wait_for_param("/pnp_ros/ready"): rospy.sleep(0.1)
        
    global DESTINATION, QUEUE
    
    ros_utils.wait_for_service('/hrisim/path')
    path_request = rospy.ServiceProxy('/hrisim/path', PathRequest)

    rospy.set_param('/hrisim/robot_busy', False)
    PLAN_ON = True
    while PLAN_ON:
        rospy.logerr("Planning..")
                   
        if len(QUEUE) == 0:
            QUEUE = path_request(ROBOT_CLOSEST_WP, DESTINATION).queue
            QUEUE = QUEUE.split(',')
            rospy.logwarn(f"{QUEUE}")
            p.exec_action('speak', f'Navigating_to_{QUEUE[-1]}')

        #! Here the goal is taken from the queue
        if not rospy.get_param('/hrisim/robot_busy') and len(QUEUE) > 0:
            next_sub_goal = QUEUE.pop(0)
            rospy.logwarn(f"Planning next goal: {next_sub_goal}")
            nextnext_sub_goal = QUEUE[0] if len(QUEUE) > 0 else None
                
            send_goal(p, next_sub_goal, nextnext_sub_goal)
            GOAL_STATUS = rospy.get_param('/hrisim/goal_status')
            if GOAL_STATUS == -1:
                rospy.logerr("Goal failed!")
                QUEUE = []
                p.exec_action('speak', 'Task_failed!')
                PLAN_ON = False
                continue
            rospy.set_param('/hrisim/goal_status', 0)
            
            if len(QUEUE) == 0: 
                p.exec_action('speak', 'Task_completed_successfully!')
                PLAN_ON = False
    
def cb_robot_closest_wp(wp: String):
    global ROBOT_CLOSEST_WP
    ROBOT_CLOSEST_WP = wp.data
    
   
if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python script.py DESTINATION SCENARIO")
        sys.exit(1)
    DESTINATION = sys.argv[1]
    SCENARIO = sys.argv[2]
    
    g_path = ros_utils.wait_for_param("/Astar_navigation/graph_path")
    with open(g_path, 'rb') as f:
        G = pickle.load(f)
    
    ROBOT_CLOSEST_WP = None
    QUEUE = []
    TASK_TIMEOUT = ros_utils.wait_for_param("/hrisim/abort_timeout")
    
    p = PNPCmd()
    rospy.set_param('/hrisim/time_slot', SCENARIO)
    rospy.Subscriber("/hrisim/robot_closest_wp", String, cb_robot_closest_wp)

    p.begin()

    Plan(p)

    p.end()