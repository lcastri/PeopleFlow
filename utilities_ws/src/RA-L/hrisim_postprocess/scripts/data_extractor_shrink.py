#!/usr/bin/env python

from enum import Enum
import math
import os
import sys
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
import subprocess
import threading
import time

NODE_NAME = 'hrisim_postprocess'
NODE_RATE = 10 #Hz
GOAL_REACHED_THRES = 0.2
NOGOAL = -1000
STOP = False

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

class TaskResult(Enum):
    SUCCESS = 1
    FAILURE = -1
    WIP = 0

class Task(Enum):
    DELIVERY = "delivery"
    INVENTORY = "inventory"
    CLEANING = "cleaning"
    CHARGING = "charging"

TASKS = {
    Task.DELIVERY.value: 0,
    Task.INVENTORY.value: 1,
    Task.CLEANING.value: 2,
    Task.CHARGING.value: 3
}

class TOD(Enum):
    STARTING = "starting"
    MORNING = "morning"
    LUNCH = "lunch"
    AFTERNOON = "afternoon"
    QUITTING = "quitting"
    OFF = "off"

TODS = {
    TOD.STARTING.value: 0,
    TOD.MORNING.value: 1,
    TOD.LUNCH.value: 2,
    TOD.AFTERNOON.value: 3,
    TOD.QUITTING.value: 4,
    TOD.OFF.value: 5
}

class WP(Enum):
  PARKING = "parking"
  DOOR_ENTRANCE = "door_entrance"
  DOOR_ENTRANCE_CANTEEN = "door_entrance-canteen"
  CORRIDOR_ENTRANCE = "corridor_entrance"
  DOOR_CORRIDOR1 = "door_corridor1"
  DOOR_CORRIDOR2 = "door_corridor2"
  DOOR_CORRIDOR3 = "door_corridor3"
  SHELF12 = "shelf12"
  SHELF23 = "shelf23"
  SHELF34 = "shelf34"
  SHELF45 = "shelf45"
  SHELF56 = "shelf56"
  DOOR_OFFICE1 = "door_office1"
  DOOR_OFFICE2 = "door_office2"
  DOOR_TOILET1 = "door_toilet1"
  DOOR_TOILET2 = "door_toilet2"
  DELIVERY_POINT = "delivery_point"
  CORRIDOR0 = "corridor0"
  CORRIDOR1 = "corridor1"
  CORRIDOR2 = "corridor2"
  CORRIDOR3 = "corridor3"
  CORRIDOR4 = "corridor4"
  CORRIDOR5 = "corridor5"
  ENTRANCE = "entrance"
  OFFICE1 = "office1"
  OFFICE2 = "office2"
  TOILET1 = "toilet1"
  TOILET2 = "toilet2"
  TABLE2 = "table2"
  TABLE3 = "table3"
  TABLE4 = "table4"
  TABLE5 = "table5"
  TABLE6 = "table6"
  CORR_CANTEEN_1 = "corr_canteen_1"
  CORR_CANTEEN_2 = "corr_canteen_2"
  CORR_CANTEEN_3 = "corr_canteen_3"
  CORR_CANTEEN_4 = "corr_canteen_4"
  CORR_CANTEEN_5 = "corr_canteen_5"
  CORR_CANTEEN_6 = "corr_canteen_6"
  KITCHEN_1 = "kitchen1"
  KITCHEN_2 = "kitchen2"
  KITCHEN_3 = "kitchen3"
  CORRIDOR_CANTEEN = "corridor_canteen"
  SHELF1 = "shelf1"
  SHELF2 = "shelf2"
  SHELF3 = "shelf3"
  SHELF4 = "shelf4"
  SHELF5 = "shelf5"
  SHELF6 = "shelf6"
  CHARGING_STATION = "charging_station"

WPS = {
    WP.PARKING.value: 0,
    WP.DOOR_ENTRANCE.value: 1,
    WP.DOOR_ENTRANCE_CANTEEN.value: 2,
    WP.CORRIDOR_ENTRANCE.value: 3,
    WP.DOOR_CORRIDOR1.value: 4,
    WP.DOOR_CORRIDOR2.value: 5,
    WP.DOOR_CORRIDOR3.value: 6,
    WP.SHELF12.value: 7,
    WP.SHELF23.value: 8,
    WP.SHELF34.value: 9,
    WP.SHELF45.value: 10,
    WP.SHELF56.value: 11,
    WP.DOOR_OFFICE1.value: 12,
    WP.DOOR_OFFICE2.value: 13,
    WP.DOOR_TOILET1.value: 14,
    WP.DOOR_TOILET2.value: 15,
    WP.DELIVERY_POINT.value: 16,
    WP.CORRIDOR0.value: 17,
    WP.CORRIDOR1.value: 18,
    WP.CORRIDOR2.value: 19,
    WP.CORRIDOR3.value: 20,
    WP.CORRIDOR4.value: 21,
    WP.CORRIDOR5.value: 22,
    WP.ENTRANCE.value: 23,
    WP.OFFICE1.value: 24,
    WP.OFFICE2.value: 25,
    WP.TOILET1.value: 26,
    WP.TOILET2.value: 27,
    WP.TABLE2.value: 28,
    WP.TABLE3.value: 29,
    WP.TABLE4.value: 30,
    WP.TABLE5.value: 31,
    WP.TABLE6.value: 32,
    WP.CORR_CANTEEN_1.value: 33,
    WP.CORR_CANTEEN_2.value: 34,
    WP.CORR_CANTEEN_3.value: 35,
    WP.CORR_CANTEEN_4.value: 36,
    WP.CORR_CANTEEN_5.value: 37,
    WP.CORR_CANTEEN_6.value: 38,
    WP.KITCHEN_1.value: 39,
    WP.KITCHEN_2.value: 40,
    WP.KITCHEN_3.value: 41,
    WP.CORRIDOR_CANTEEN.value: 42,
    WP.SHELF1.value: 43,
    WP.SHELF2.value: 44,
    WP.SHELF3.value: 45,
    WP.SHELF4.value: 46,
    WP.SHELF5.value: 47,
    WP.SHELF6.value: 48,
    WP.CHARGING_STATION.value: 49,
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
        self.last_clock_time = time.time()
        
        
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
        self.timeOfDay = TODS[t.time_of_the_day.data] if t.time_of_the_day.data != 'None' else 'none'
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
        
        
def isFinished(data_handler):
    while not rospy.is_shutdown():

        # rospy.logerr(f"now: {time.time()} - last: {data_handler.last_clock_time} = {time.time() - data_handler.last_clock_time}")
        if time.time() - data_handler.last_clock_time > 5:
            rospy.logerr("Threshooooold")
            rospy.signal_shutdown("Time of day changed or rosbag finished.")
            

def save_data_to_csv(data_rows, filename, csv_path, wps):   
    rospy.logwarn("Saving data into CSV files..")
    
    # Generating Task Result Column
    df = pd.DataFrame(data_rows)
        
    df.to_csv(f"{csv_path}/{filename}.csv", index=False)
    del df
    rospy.logwarn(f"Saved {filename}.csv")

    # Save WP-specific DataFrames
    general_columns_name = ['pf_elapsed_time', 'TOD', 'R_V', 'R_T', 'R_B']
    for wp_id in wps.keys():
        wp_df = pd.DataFrame(
            [{key: row[key] for key in row if key in (general_columns_name + [f"{wp_id}_NP", f"{wp_id}_PD", f"{wp_id}_BAC"])} for row in data_rows]
        )
        
        # Rename the WP-specific columns
        wp_df = wp_df.rename(columns={f"{wp_id}_NP": "NP", f"{wp_id}_PD": "PD", f"{wp_id}_BAC": "BAC"})
        
        # Add the constant column "wp" with the value wp_id
        wp_df["WP"] = WPS[wp_id]
        
        wp_filename = f"{filename}_{wp_id}.csv"
        wp_df.to_csv(f"{csv_path}/{wp_filename}", index=False)
        rospy.logwarn(f"Saved {wp_filename}")


def shutdown_callback(data_rows, bagname, csv_path, data, savegoal = True):
    rospy.logwarn("Shutting down and saving data.")
    filename = f"{bagname}_{TIMEOFTHEDAY}"
    
    if savegoal and data.robot.taskOn and not data.robot.goalReached:
        goal_data = {'x': data.robot.x, 'y': data.robot.y,
                     'gx': data.robot.gx, 'gy': data.robot.gy}
        json_filename = os.path.join(csv_path, "goal.json")

        # Save the goal data into a JSON file
        with open(json_filename, 'w') as json_file:
            json.dump(goal_data, json_file)
        rospy.loginfo(f"Saved goal coordinates to {json_filename}")
    
    save_data_to_csv(data_rows, filename, csv_path, data.WPs)
    

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
    CSV_PATH = os.path.join(NODE_PATH, 'csv')
    SCENARIO = os.path.join(NODE_PATH, 'scenarios', f'{scn}.xml')
    TIMEOFTHEDAY = rospy.get_param('~time_of_the_day')
    LOADGOAL = rospy.get_param('~load_goal')
    WPS_COORD = readScenario()
    rospy.logwarn(f"Running data_extractor_shrink.py on TimeOfTheDay: {TIMEOFTHEDAY}")
    if LOADGOAL:
        goal_file = os.path.join(CSV_PATH, "goal.json")
        if os.path.exists(goal_file):
            with open(goal_file, 'r') as json_file:
                G = json.load(json_file)
            rospy.logwarn(f"Loaded goal coordinates from {goal_file}")
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
    
    # Start the clock monitor in a separate thread
    thread = threading.Thread(target=isFinished, args=(data_handler,))
    thread.daemon = True  # Daemonize the thread to shut down with the program
    thread.start()

    # Initialize variables to track timeOfDay
    recording = False
    data_rows = []  # List to store collected data for each segment
    
    # Register the shutdown callback
    rospy.on_shutdown(lambda: shutdown_callback(data_rows, BAGNAME, CSV_PATH, data_handler))
    
    while not rospy.is_shutdown():

        if data_handler.timeOfDay == '':
            continue

        # Start recording when timeOfDay matches the specified time
        if not recording and value2key(TODS, data_handler.timeOfDay) == TIMEOFTHEDAY:
            rospy.logwarn(f"Started recording at {TIMEOFTHEDAY}")
            recording = True

        # Stop recording and trigger shutdown when timeOfDay changes
        if recording and (value2key(TODS, data_handler.timeOfDay) != TIMEOFTHEDAY):
            if data_handler.timeOfDay == 'none':
                rospy.logwarn("Rosbag ended!, triggering shutdown")
            else:
                rospy.logwarn(f"Time of day changed from {TIMEOFTHEDAY} to {value2key(TODS, data_handler.timeOfDay)}, triggering shutdown")
            rospy.signal_shutdown("Time of day changed or rosbag finished.")


        if recording:
            # Collect data for the current time step
            data_row = {
                'ros_time': data_handler.rostime,
                'pf_elapsed_time': data_handler.elapsed,
                'TOD': data_handler.timeOfDay,
                'R_X': data_handler.robot.x,
                'R_Y': data_handler.robot.y,
                'R_YAW': data_handler.robot.yaw,
                'R_V': data_handler.robot.v,
                'G_X': data_handler.robot.gx,
                'G_Y': data_handler.robot.gy,
                'R_T': data_handler.robot.task_result if len(data_rows) > 0 else 0,
                'R_B': data_handler.robot.battery_level,
                'is_charging': 1 if data_handler.robot.is_charging else 0,
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