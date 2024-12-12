#!/usr/bin/env python

import csv
from enum import Enum
import math
import os
import sys
import threading
import pandas as pd
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from pedsim_msgs.msg import AgentStates
from peopleflow_msgs.msg import WPPeopleCounters, Time as pT
from tiago_battery.msg import BatteryStatus
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
import hrisim_util.ros_utils as ros_utils
from robot_msgs.msg import BatteryAtChargers
from shapely.geometry import Polygon, Point
import xml.etree.ElementTree as ET
import json
import time
from utils import *
from concurrent.futures import ThreadPoolExecutor


NODE_NAME = 'hrisim_postprocess'
NODE_RATE = 10 #Hz
GOAL_REACHED_THRES = 0.2
NOGOAL = -1000

AREAS = {
    'shelves_12': Polygon([(-18.839, 9), (-10.5, 9), (-10.5, 3), (-18.839, 3)]),
    'shelves_34': Polygon([(-18.839, 2.5), (-10.5, 2.5), (-10.5, -3), (-18.839, -3)]),
    'shelves_56': Polygon([(-18.839, -3.5), (-10.5, -3.5), (-10.5, -10.5), (-18.839, -10.5)]),
    'shelf_top_corr': Polygon([(-10.5, 10.5), (-7.2, 10.5), (-7.1, 2), (-10.5, 2)]),
    'shelf_centre_corr': Polygon([(-10.5, 2), (-7.1, 2), (-7.1, -4), (-10.5, -4)]),
    'shelf_bottom_corr': Polygon([(-10.5, -4), (-7.1, -4), (-7.1, -10.5), (-10.5, -10.5)]),
    'entrance': Polygon([(10.6, 10), (10.6, 6.87), (2, 6.87), (2, 10)]),
    'corridor_0': Polygon([(2, 10), (2, 6.87), (-1.55, 6.87), (-1.55, 10)]),
    'corridor_1': Polygon([(-1.54, 10), (-1.54, 5), (-7.14, 5), (-7.19, 10)]),
    'corridor_2': Polygon([(-7.14, 5), (-7.1, -1.93), (-1.54, -1.93), (-1.54, 5)]),
    'corridor_3': Polygon([(-7.1, -1.93), (-1.54, -1.93), (-1.54, -5.05), (1.95, -5.05), (1.95, -10.5), (-7.1, -10.5)]),
    'office_1': Polygon([(-1.54, 6.9), (1.95, 6.9), (1.95, 4), (-1.54, 4)]),   
    'office_2': Polygon([(-1.54, 4), (1.95, 4), (1.95, 1), (-1.54, 1)]),   
    'toilet_1': Polygon([(-1.54, 1), (1.95, 1), (1.95, -1.92), (-1.54, -1.92)]),   
    'toilet_2': Polygon([(-1.54, -1.92), (1.95, -1.92), (1.95, -5), (-1.54, -5)]),   
    'kitchen_1': Polygon([(10.6, 6.9), (1.95, 6.9), (1.95, 2), (5.8, 2), (5.8, 4.8), (10.6, 4.8)]),         
    'kitchen_2': Polygon([(1.95, 2), (5.8, 2), (5.8, -2.4), (1.95, -2.4)]),         
    'kitchen_3': Polygon([(1.95, -2.4), (5.8, -2.4), (5.8, -10.5), (1.95, -10.5)]),         
    'tables_23': Polygon([(5.8, 4), (10.6, 4), (10.6, 0), (5.8, 0)]),         
    'tables_45': Polygon([(5.8, -0.35), (10.6, -0.35), (10.6, -4.35), (5.8, -4.35)]),         
    'tables_6': Polygon([(5.8, -4.9), (10.6, -4.9), (10.6, -10.5), (5.8, -10.5)])
}

class Robot():
    def __init__(self, x, y, gx, gy) -> None:
        self.x = x
        self.y = y
        self.yaw = 0
        self.v = 0
        self.gx = gx
        self.gy = gy
        self.task = -1
        self.battery_level = 0
        self.is_charging = 0
        self.closest_wp = ''
        self.task_result = 0
    
    @property
    def goalReached(self):
        return math.sqrt((self.x - self.gx)**2 + (self.y - self.gy)**2) <= GOAL_REACHED_THRES
    
    @property
    def taskOn(self):
        return self.gx != NOGOAL and self.gy != NOGOAL
            
        
        
class Agent():
    def __init__(self) -> None:
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 0      


class DataManager():
    """
    Class handling data
    """
    
    def __init__(self, x = 0, y = 0, gx = NOGOAL, gy = NOGOAL):
        """
        Class constructor. Init publishers and subscribers
        """
    
        self.last_clock_time = time.time()
        self.rostime = 0
        self.robot = Robot(x, y, gx, gy)
        self.agents = {}
        
        self.WPs = {}
        self.BACs = {}
        self.PDs = {}

        self.peopleAtWork = 0
        self.timeOfDay = ''        
        self.hhmmss = ''
        self.elapsed = 0

        # subscribers
        rospy.Subscriber("/clock", Clock, self.cb_clock)
        rospy.Subscriber("/robot_pose", PoseWithCovarianceStamped, self.cb_robot_pose)
        rospy.Subscriber("/mobile_base_controller/odom", Odometry, self.cb_odom)
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.cb_robot_goal)
        rospy.Subscriber("/hrisim/robot_task", String, self.cb_robot_task)
        rospy.Subscriber("/peopleflow/counter", WPPeopleCounters, self.cb_people_counter)
        rospy.Subscriber("/peopleflow/time", pT, self.cb_time)
        rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, self.cb_agents)
        rospy.Subscriber("/hrisim/robot_battery", BatteryStatus, self.cb_robot_battery)
        rospy.Subscriber("/hrisim/robot_bac", BatteryAtChargers, self.cb_robot_bac)
        rospy.Subscriber("/hrisim/robot_closest_wp", String, self.cb_robot_closest_wp)
                   
            
    def cb_clock(self, clock: Clock):
        self.rostime = clock.clock.to_sec()
        
        
    def cb_robot_pose(self, pose: PoseWithCovarianceStamped):
        self.robot.x, self.robot.y, self.robot.yaw = ros_utils.getPose(pose.pose.pose)
        
        
    def cb_odom(self, odom: Odometry):
        self.robot.v = abs(odom.twist.twist.linear.x)
        
        
    def cb_robot_goal(self, goal: MoveBaseActionGoal):
        if self.robot.taskOn: 
            if self.robot.goalReached:
                self.robot.task_result = TaskResult.SUCCESS.value
            else:
                self.robot.task_result = TaskResult.FAILURE.value
        else:
            self.robot.task_result = TaskResult.WIP.value
            
        self.robot.gx = goal.goal.target_pose.pose.position.x
        self.robot.gy = goal.goal.target_pose.pose.position.y
    
        
    def cb_robot_task(self, task: String):
        self.robot.task = TASKS[task.data] if task.data != 'none' else -1
        
        
    def cb_people_counter(self, wps: WPPeopleCounters):
        self.peopleAtWork = wps.numberOfWorkingPeople
        for wp in wps.counters:
            self.WPs[wp.WP_id.data] = wp.numberOfPeople
            self.PDs[wp.WP_id.data] = wp.numberOfPeople/AREAS[WP_AREA[wp.WP_id.data]].area
            
            
    def cb_time(self, t: pT):
        self.timeOfDay = int(t.elapsed // 3600)
        # self.timeOfDay = TODS[t.time_of_the_day.data] if t.time_of_the_day.data != 'None' else 'none'
        self.hhmmss = t.hhmmss.data
        self.elapsed = t.elapsed
        
        
    def cb_agents(self, people: AgentStates):        
        for person in people.agent_states:
            if person.id not in self.agents:
                self.agents[person.id] = Agent()
            self.agents[person.id].x, self.agents[person.id].y, self.agents[person.id].yaw = ros_utils.getPose(person.pose)
            self.agents[person.id].v = person.twist.linear.x      


    def cb_robot_battery(self, b: BatteryStatus):
        self.robot.battery_level = b.level.data
        self.robot.is_charging = b.is_charging.data
               
        
    def cb_robot_bac(self, bacs: BatteryAtChargers):
        for bac in bacs.BACs:
            self.BACs[bac.WP_id.data] = bac.BAC.data
                    
        
    def cb_robot_closest_wp(self, wp: String):
        self.robot.closest_wp = WPS[wp.data]
        
def shutdown_callback(data_rows, filename, csv_path, goal_path, data):   
    rospy.logwarn("Shutting down and saving data.")
    
    if data.robot.taskOn and not data.robot.goalReached:
        goal_data = {'x': data.robot.x, 'y': data.robot.y,
                     'gx': data.robot.gx, 'gy': data.robot.gy}

        # Save the goal data into a JSON file
        with open(goal_path, 'w') as json_file:
            json.dump(goal_data, json_file)
        rospy.loginfo(f"Saved goal coordinates to {goal_path}")
    
    save_data_to_csv(data_rows, filename, csv_path)
           

def save_data_to_csv(data_rows, filename, csv_path):   
    rospy.logwarn("Saving data into CSV files..")
    
    # Generating Task Result Column
    df = pd.DataFrame(data_rows)
        
    df.to_csv(f"{csv_path}/{filename}.csv", index=False)
    del df
    rospy.logwarn(f"Saved {filename}.csv")
    

def readScenario():
    # Load and parse the XML file
    tree = ET.parse(SCENARIO)
    root = tree.getroot()
    
    wps = {}
    for waypoint in root.findall('waypoint'):
        waypoint_id = waypoint.get('id')
        x = float(waypoint.get('x'))
        y = float(waypoint.get('y'))
        r = float(waypoint.get('r'))
        wps[waypoint_id] = {'x': x, 'y': y, 'r': r}
    
    return wps


def value2key(my_dict, val):
    key = next(k for k, v in my_dict.items() if v == val)
    return key


if __name__ == '__main__':
    
    # Init node
    rospy.init_node(NODE_NAME)

    # Set node rate
    rate = rospy.Rate(NODE_RATE)
    
    BAGNAME = rospy.get_param('~bagname')
    NODE_PATH = rospy.get_param('~node_path')
    scn = rospy.get_param('~scenario')
    SCENARIO = os.path.join(NODE_PATH, 'scenarios', f'{scn}.xml')
    CSV_PATH = os.path.join(NODE_PATH, 'csv', 'original', BAGNAME)
    os.makedirs(CSV_PATH, exist_ok=True)
    LOADGOAL = rospy.get_param('~load_goal')
    NEXT_SAVE_TIME = int(rospy.get_param('~next_save_time'))  # Save every hour based on elapsed time
    STARTING_TIME = NEXT_SAVE_TIME - 3600
    GOAL_PATH = os.path.join(NODE_PATH, 'csv', "goal.json")
    WPS_COORD = readScenario()
    if LOADGOAL:
        if os.path.exists(GOAL_PATH):
            with open(GOAL_PATH, 'r') as json_file:
                G = json.load(json_file)
            rospy.logwarn(f"Loaded goal coordinates from {GOAL_PATH}")
        else:
            G = {'x': 0, 'y': 0, 'gx': NOGOAL, 'gy': NOGOAL}
    else:
        G = {'x': 0, 'y': 0, 'gx': NOGOAL, 'gy': NOGOAL}
    
    # Map waypoints to clusters
    WP_AREA = {}
    for wp, coords in WPS_COORD.items():
        point = Point(coords['x'], coords['y'])
        for cluster_name, cluster_polygon in AREAS.items():
            if cluster_polygon.contains(point):
                WP_AREA[wp] = cluster_name
                break

    data_handler = DataManager(x = G['x'], y = G['y'], gx = G['gx'], gy = G['gy'])
    recording = False
    data_rows = []  # List to store collected data for each segment
    
    # Register the shutdown callback
    rospy.on_shutdown(lambda: shutdown_callback(data_rows, f"{BAGNAME}_{int((NEXT_SAVE_TIME // 3600))}h", CSV_PATH, GOAL_PATH, data_handler))

    # executor = ThreadPoolExecutor(max_workers=1)

    while not rospy.is_shutdown():
        if data_handler.timeOfDay == '': continue
        rospy.logerr(f"Elapsed time: {data_handler.elapsed}/{NEXT_SAVE_TIME}")
        
        if not recording and data_handler.elapsed >= STARTING_TIME:
            rospy.logwarn(f"Started recording")
            recording = True
            
        if data_handler.elapsed >= NEXT_SAVE_TIME:
            # Copy current data to a buffer for saving
            # data_to_save = data_rows[:]
            # data_rows = []  # Clear data_rows immediately for new data
    
            # Save the current data to a CSV file
            # t = threading.Thread(target=save_data_to_csv, args=(data_to_save, f"{BAGNAME}_{int(next_save_time // 3600)}h.csv", CSV_PATH))
            # t.start()
            # executor.submit(save_data_to_csv, data_to_save, f"{BAGNAME}_{int(next_save_time // 3600)}h", CSV_PATH)
            # save_data_to_csv(data_to_save, f"{BAGNAME}_{int(next_save_time // 3600)}h", CSV_PATH)
            rospy.signal_shutdown("Time of day changed or rosbag finished.")


            # Prepare for the next hour
            # next_save_time += 3600  # Set next save time

        if recording:

            # Collect data for the current time step
            data_row = {
                'ros_time': data_handler.rostime,
                'pf_elapsed_time': data_handler.elapsed,
                'TOD': data_handler.timeOfDay,
                'T': data_handler.robot.task,
                'R_X': data_handler.robot.x,
                'R_Y': data_handler.robot.y,
                'R_YAW': data_handler.robot.yaw,
                'R_V': data_handler.robot.v,
                'G_X': data_handler.robot.gx,
                'G_Y': data_handler.robot.gy,
                'T_R': data_handler.robot.task_result if len(data_rows) > 0 else 0,
                'R_B': data_handler.robot.battery_level,
                'B_S': 1 if data_handler.robot.is_charging else 0,
            }
                            
            data_handler.robot.task_result = TaskResult.WIP.value
                
            # Collect agents' data
            for agent_id, agent in data_handler.agents.items():
                data_row[f'a{agent_id}_X'] = agent.x
                data_row[f'a{agent_id}_Y'] = agent.y
                data_row[f'a{agent_id}_YAW'] = agent.yaw
                data_row[f'a{agent_id}_V'] = agent.v

            # Collect WP' data
            for wp_id in data_handler.WPs.keys():
                data_row[f'{wp_id}_NP'] = data_handler.WPs[wp_id]
            for wp_id in data_handler.PDs.keys():
                data_row[f'{wp_id}_PD'] = data_handler.PDs[wp_id]
            for wp_id in data_handler.BACs.keys():
                data_row[f'{wp_id}_BAC'] = data_handler.BACs[wp_id]

            # Append the row to the list
            data_rows.append(data_row)

        rate.sleep()