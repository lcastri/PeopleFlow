#!/usr/bin/env python

import math
import os
import pandas as pd
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from pedsim_msgs.msg import AgentStates
from peopleflow_msgs.msg import WPPeopleCounters, Time as pT
from tiago_battery.msg import BatteryStatus
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Int32
from rosgraph_msgs.msg import Clock
import hrisim_util.ros_utils as ros_utils
from robot_msgs.msg import BatteryAtChargers, TasksInfo, TaskInfo
from shapely.geometry import Point
import xml.etree.ElementTree as ET
import json
import time
from utils import *


NODE_NAME = 'hrisim_postprocess'
NODE_RATE = 10 #Hz
GOAL_REACHED_THRES = 0.2
NOGOAL = -1000


class Robot():
    def __init__(self, x, y, gx, gy) -> None:
        self.x = x
        self.y = y
        self.yaw = 0
        self.v = 0
        self.gx = gx
        self.gy = gy
        self.battery_level = 0
        self.is_charging = 0
        self.closest_wp = ''
        self.clearing_distance = 0
        self.H_collision = 0
        self.task = -1
            
        
class Agent():
    def __init__(self) -> None:
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 0 
           
class Task():
    def __init__(self, id, starting, path, destination) -> None:
        self.id = id
        self.starting = starting
        self.path = path
        self.destination = destination
        self.ending = 0
        self.result = 0      


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
        self.tasks = {}
        self.n_tasks = 0
        self.n_success = 0
        self.n_failure = 0
        
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
        rospy.Subscriber("/peopleflow/counter", WPPeopleCounters, self.cb_people_counter)
        rospy.Subscriber("/peopleflow/time", pT, self.cb_time)
        rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, self.cb_agents)
        rospy.Subscriber("/hrisim/robot_battery", BatteryStatus, self.cb_robot_battery)
        rospy.Subscriber("/hrisim/robot_bac", BatteryAtChargers, self.cb_robot_bac)
        rospy.Subscriber("/hrisim/robot_closest_wp", String, self.cb_robot_closest_wp)
        rospy.Subscriber("/hrisim/robot_clearing_distance", Float32, self.cb_robot_clearing_distance)
        rospy.Subscriber("/hrisim/robot_human_collision", Int32, self.cb_robot_human_collision)
        rospy.Subscriber("/hrisim/robot_tasks_info", TasksInfo, self.cb_robot_tasks)  
                   
            
    def cb_clock(self, clock: Clock):
        self.rostime = clock.clock.to_sec()
        
        
    def cb_robot_pose(self, pose: PoseWithCovarianceStamped):
        self.robot.x, self.robot.y, self.robot.yaw = ros_utils.getPose(pose.pose.pose)
        
        
    def cb_odom(self, odom: Odometry):
        self.robot.v = abs(odom.twist.twist.linear.x)
        
        
    def cb_robot_goal(self, goal: MoveBaseActionGoal):            
        self.robot.gx = goal.goal.target_pose.pose.position.x
        self.robot.gy = goal.goal.target_pose.pose.position.y
        
        
    def cb_people_counter(self, wps: WPPeopleCounters):
        self.peopleAtWork = wps.numberOfWorkingPeople
        for wp in wps.counters:
            self.WPs[wp.WP_id.data] = wp.numberOfPeople
            self.PDs[wp.WP_id.data] = wp.numberOfPeople/WPS_INFO[wp.WP_id.data]['A']
            
            
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

        
    def cb_robot_clearing_distance(self, msg: Float32):
        self.robot.clearing_distance = msg.data
           
           
    def cb_robot_human_collision(self, msg: Int32):
        self.robot.H_collision = msg.data
        
    
    def cb_robot_tasks(self, msg: TasksInfo):
        tasks = msg.Tasks
        for task in tasks:
            if task.task_id not in self.tasks:
                self.tasks[task.task_id] = Task(task.task_id, task.start_time.to_sec(), task.path, task.final_destination)
            self.tasks[task.task_id].result = task.result
            self.tasks[task.task_id].ending = task.end_time.to_sec()
            
        self.robot.task = tasks[-1].task_id if len(tasks) else -1
        
        self.n_tasks = msg.num_tasks
        self.n_success = msg.num_success
        self.n_failure = msg.num_failure
        
        
        
def shutdown_callback(data_rows, filename, csv_path, goal_path, data):   
    rospy.logwarn("Shutting down and saving data.")
    
    goal_data = {'x': data.robot.x, 'y': data.robot.y,
                 'gx': data.robot.gx, 'gy': data.robot.gy}
    
    tasks = {id: {'start': task.starting, 
                  'end': task.ending, 
                  'result': task.result,
                  'path': task.path,
                  'final_destination': task.destination} for id, task in data.tasks.items()}
    tasks['n_tasks'] = data.n_tasks
    tasks['n_success'] = data.n_success
    tasks['n_failure'] = data.n_failure

    # Save the goal data into a JSON file
    with open(goal_path, 'w') as json_file:
        json.dump(goal_data, json_file)
    rospy.loginfo(f"Saved goal coordinates to {goal_path}")
    
    with open(os.path.join(csv_path, 'tasks.json'), 'w') as json_file:
        json.dump(tasks, json_file)
    rospy.loginfo(f"Saved tasks {csv_path}")
    
    save_data_to_csv(data_rows, filename, csv_path)
           

def save_data_to_csv(data_rows, filename, csv_path):   
    rospy.logwarn("Saving data into CSV files..")
    
    # Generating Task Result Column
    df = pd.DataFrame(data_rows)
        
    df.to_csv(f"{csv_path}/{filename}_{TIMEOFTHEDAY}.csv", index=False)
    del df
    rospy.logwarn(f"Saved {filename}_{TIMEOFTHEDAY}.csv")
    

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
        wps[waypoint_id]['A'] = math.pi * r**2
    
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
    CSV_PATH = os.path.join(NODE_PATH, 'csv', 'HH','original', BAGNAME)
    os.makedirs(CSV_PATH, exist_ok=True)
    LOADGOAL = rospy.get_param('~load_goal')
    TIMEOFTHEDAY = rospy.get_param('~time_of_the_day')
    GOAL_PATH = os.path.join(NODE_PATH, 'csv', "goal.json")
    
    # Map waypoints
    WPS_INFO = readScenario()
            
    if LOADGOAL:
        if os.path.exists(GOAL_PATH):
            with open(GOAL_PATH, 'r') as json_file:
                G = json.load(json_file)
            rospy.logwarn(f"Loaded goal coordinates from {GOAL_PATH}")
        else:
            G = {'x': 0, 'y': 0, 'gx': NOGOAL, 'gy': NOGOAL}
    else:
        G = {'x': 0, 'y': 0, 'gx': NOGOAL, 'gy': NOGOAL}
    
    data_handler = DataManager(x = G['x'], y = G['y'], gx = G['gx'], gy = G['gy'])
    
    # Initialize variables to track timeOfDay
    recording = False
    data_rows = []  # List to store collected data for each segment
    
    # Register the shutdown callback
    rospy.on_shutdown(lambda: shutdown_callback(data_rows, BAGNAME, CSV_PATH, GOAL_PATH, data_handler))
    
    while not rospy.is_shutdown():

        if data_handler.timeOfDay == '': continue

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
                'R_B': data_handler.robot.battery_level,
                'B_S': 1 if data_handler.robot.is_charging else 0,
                'R_CD': data_handler.robot.clearing_distance,
                'R_HC': data_handler.robot.H_collision,
                'T': data_handler.robot.task,
            }
            
            data_handler.robot.H_collision = 0
                                            
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