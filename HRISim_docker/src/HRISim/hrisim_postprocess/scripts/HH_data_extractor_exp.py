#!/usr/bin/env python

import math
import os
import pandas as pd
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from pedsim_msgs.msg import AgentStates
from peopleflow_msgs.msg import WPPeopleCounters, Time as pT
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Int32, Bool
from rosgraph_msgs.msg import Clock
import hrisim_util.ros_utils as ros_utils
from robot_msgs.msg import TasksInfo, BatteryStatus
import xml.etree.ElementTree as ET
import json
from utils import *
from std_srvs.srv import SetBool, SetBoolRequest


NODE_NAME = 'hrisim_postprocess'
NODE_RATE = 10 #Hz
NOGOAL = -1000


class Robot():
    def __init__(self) -> None:
        self.x = NOGOAL
        self.y = NOGOAL
        self.yaw = NOGOAL
        self.v = 0
        self.gx = NOGOAL
        self.gy = NOGOAL
        self.battery_level = NOGOAL
        self.is_charging = 0
        self.closest_wp = ''
        self.clearing_distance = 0
        self.H_collision = 0
        self.task = -1
        self.obs = 0
    
        
class Agent():
    def __init__(self) -> None:
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.v = 0 
        
        
class Task():
    def __init__(self, id, starting, path, destination, 
                 tot_query_inf_time=0, mean_query_inf_time=0, planning_time=0, evaluations=0) -> None:
        self.id = id
        self.starting = starting
        self.path = path
        self.destination = destination
        self.tot_query_inf_time = tot_query_inf_time
        self.mean_query_inf_time = mean_query_inf_time
        self.planning_time = planning_time
        self.evaluations = evaluations
        self.ending = 0
        self.result = 0      

class DataManager():
    """
    Class handling data
    """
    
    def __init__(self):
        """
        Class constructor. Init publishers and subscribers
        """
    
        self.rostime = 0
        self.robot = Robot()
        self.agents = {}
        self.tasks = {}
        self.n_tasks = 0
        self.n_success = 0
        self.n_failure = 0
        
        self.WPs = {}
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
        rospy.Subscriber("/hrisim/robot_closest_wp", String, self.cb_robot_closest_wp)
        rospy.Subscriber("/hrisim/robot_clearing_distance", Float32, self.cb_robot_clearing_distance)
        rospy.Subscriber("/hrisim/robot_human_collision", Int32, self.cb_robot_human_collision)
        rospy.Subscriber("/hrisim/robot_tasks_info", TasksInfo, self.cb_robot_tasks)  
        rospy.Subscriber("/hrisim/robot_obs", Bool, self.cb_robot_obs)            
            
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
            self.PDs[wp.WP_id.data] = math.log1p(wp.numberOfPeople) / math.log1p(WPS_INFO[wp.WP_id.data]['A'])
            
            
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
                # self.tasks[task.task_id] = Task(task.task_id, task.start_time.to_sec(), task.path, task.final_destination,
                #                                 task.tot_inf_time, task.mean_inf_time, 0, task.evaluations) # VersionA
                # self.tasks[task.task_id] = Task(task.task_id, task.start_time.to_sec(), task.path, task.final_destination,
                #                                 task.tot_inf_time, task.mean_inf_time, task.planning_time, task.evaluations) # VersionB
                self.tasks[task.task_id] = Task(task.task_id, task.start_time.to_sec(), task.path, task.final_destination) # Base/Full
            self.tasks[task.task_id].result = task.result
            self.tasks[task.task_id].ending = task.end_time.to_sec()
            
        self.robot.task = tasks[-1].task_id if len(tasks) and not self.robot.is_charging else -1
        
        self.n_tasks = msg.num_tasks
        self.n_success = msg.num_success
        self.n_failure = msg.num_failure
    
    def cb_robot_obs(self, msg: Bool):
        self.robot.obs = 1 if msg.data else 0
        

def save_batch(rows, csv_path, bagname):
    """
    Appends a list of data rows to a CSV file.
    Handles the header automatically.
    """
    if not rows:
        rospy.loginfo("No data in batch to save.")
        return  # Nothing to save

    filepath = f"{csv_path}/{bagname}.csv"
    
    # Check if the file already exists to decide on writing the header
    file_exists = os.path.exists(filepath)
    
    rospy.logwarn(f"Saving batch of {len(rows)} rows to {filepath}...")
    
    try:
        df = pd.DataFrame(rows)
        # Append ('a') to the file.
        # Write header only if the file does not exist.
        df.to_csv(filepath, mode='a', header=not file_exists, index=False)
        rospy.loginfo("...Batch saved successfully.")
    except Exception as e:
        rospy.logerr(f"Failed to save data batch: {e}")
            

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
    TIMEOFTHEDAY = BAGNAME.split('-')[-1]
    rospy.loginfo(f"Processing {BAGNAME}")
    NODE_PATH = rospy.get_param('~node_path')
    scn = rospy.get_param('~scenario')
    SCENARIO = os.path.join(NODE_PATH, 'scenarios', f'{scn}.xml')
    CSV_PATH = os.path.join(NODE_PATH, 'csv', '-'.join(BAGNAME.split('-')[:-1]))
    os.makedirs(CSV_PATH, exist_ok=True)
    
    # Map waypoints
    WPS_INFO = readScenario()
    
    data_handler = DataManager()
    
    data_rows = []  # List to store collected data for each segment
    
    BATCH_SIZE = 600 

    rospy.logwarn("Waiting for rosbag pause/resume service...")
    
    service_name = "/rosbag_play_data/pause_playback"
    
    try:
        # Wait for the service to be available
        rospy.wait_for_service(service_name, timeout=30.0)
    except rospy.ROSException:
        rospy.logerr(f"Service '{service_name}' not found.")
        rospy.logerr("Is the rosbag node running and named 'rosbag_play_data'?")
        rospy.signal_shutdown("Service not found")

    try:
        # Create a persistent service proxy
        pause_service = rospy.ServiceProxy(service_name, SetBool)
        
        rospy.logwarn("Service connected. Resuming playback to start...")
        
        # --- 1. RESUME the bag to start playback ---
        pause_service(SetBoolRequest(data=False))
        
        # --- Main loop wrapped in try ---
        while not rospy.is_shutdown():
            
            if data_handler.timeOfDay == '' or (value2key(TODS, data_handler.timeOfDay) != TIMEOFTHEDAY): 
                rate.sleep()
                continue
            
            # Collect data for the current time step
            data_row = {
                'ros_time': data_handler.rostime,
                'pf_elapsed_time': data_handler.elapsed,
                'TOD': data_handler.timeOfDay,
                'R_X': data_handler.robot.x,
                'R_Y': data_handler.robot.y,
                'R_V': data_handler.robot.v,
                'G_X': data_handler.robot.gx,
                'G_Y': data_handler.robot.gy,
                'R_B': data_handler.robot.battery_level,
                'B_S': 1 if data_handler.robot.is_charging else 0,
                'R_CD': data_handler.robot.clearing_distance,
                'R_HC': data_handler.robot.H_collision,
                'T': data_handler.robot.task,
                'OBS': data_handler.robot.obs,
            }
            
            data_handler.robot.H_collision = 0
                                                
            # Collect agents' data
            for agent_id, agent in data_handler.agents.items():
                data_row[f'a{agent_id}_X'] = agent.x
                data_row[f'a{agent_id}_Y'] = agent.y

            # Collect WP' data
            for wp_id in data_handler.WPs.keys():
                data_row[f'{wp_id}_NP'] = data_handler.WPs[wp_id]
            for wp_id in data_handler.PDs.keys():
                data_row[f'{wp_id}_PD'] = data_handler.PDs[wp_id]

            # Append the row to the batch list
            data_rows.append(data_row)

            # --- NEW: Batch saving logic ---
            if len(data_rows) >= BATCH_SIZE:
                
                # 1. PAUSE the bag playback
                rospy.logwarn("Batch full. Pausing rosbag...")
                pause_service(SetBoolRequest(data=True))
                
                # 2. SAVE the batch (this can take time)
                save_batch(data_rows, CSV_PATH, BAGNAME)
                data_rows = [] # Clear the list for the next batch
                
                # 3. RESUME the bag playback
                rospy.logwarn("...Resuming rosbag.")
                pause_service(SetBoolRequest(data=False))

            rate.sleep()

    except rospy.ROSInterruptException:
        # This catches Ctrl+C
        rospy.logwarn("ROSInterruptException caught. Shutting down.")
    except rospy.ServiceException as e:
        # This catches the error when the service call fails
        # which happens when the rosbag finishes and shuts down.
        rospy.logwarn(f"Rosbag service failed: {e}")
        rospy.logwarn("This likely means the bag has finished. Saving final batch.")
    
    finally:
        rospy.logwarn("Saving final data and tasks...")
        
        # --- Save any remaining rows in the list ---
        save_batch(data_rows, CSV_PATH, BAGNAME)
        
        # --- Save the tasks JSON (this logic is from your old shutdown_callback) ---
        tasks = {id: {'start': task.starting, 
                      'end': task.ending, 
                      'result': task.result,
                      'path': task.path,
                      'final_destination': task.destination,
                      'evaluations': task.evaluations,
                      'planning_time': task.planning_time,
                      'mean_query_inf_time': task.mean_query_inf_time,
                      'tot_query_inf_time': task.tot_query_inf_time} for id, task in data_handler.tasks.items()}
        
        tasks['n_tasks'] = data_handler.n_tasks
        tasks['n_success'] = data_handler.n_success
        tasks['n_failure'] = data_handler.n_failure
        
        with open(os.path.join(CSV_PATH, f'tasks-{TIMEOFTHEDAY}.json'), 'w') as json_file:
            json.dump(tasks, json_file)
        rospy.loginfo(f"Saved tasks {CSV_PATH}")
        
        rospy.logwarn("Shutdown complete.")