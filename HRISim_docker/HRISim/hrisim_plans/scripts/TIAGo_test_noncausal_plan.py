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
    if not rospy.get_param('/hri/robot_busy'):                                        
        return TASK_LIST.pop(0), constants.Task.DELIVERY, True

        
def Plan(p):
    while not ros_utils.wait_for_param("/pnp_ros/ready"):
        rospy.sleep(0.1)
        
    global NEXT_GOAL, QUEUE
    ros_utils.wait_for_param("/peopleflow/timeday")
    rospy.set_param('/hri/robot_busy', False)
    PLAN_ON = True
    rospy.set_param("/peopleflow/robot_plan_on", PLAN_ON)
    
    while PLAN_ON:
        rospy.logerr("Planning..")
        if len(QUEUE) == 0:
            rospy.sleep(1)
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
                nextnext_sub_goal = TASK_LIST[constants.Task.CLEANING.value][0] if len(TASK_LIST[constants.Task.CLEANING.value]) > 0 else None
            send_goal(p, next_sub_goal, nextnext_sub_goal)
            rospy.set_param('/hrisim/robot_task', TASK.value)
            if len(QUEUE) == 0: TASK_ON = 0
                    
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
    rospy.Subscriber("/hrisim/robot_closest_wp", String, cb_robot_closest_wp)


    p.begin()

    Plan(p)

    p.end()