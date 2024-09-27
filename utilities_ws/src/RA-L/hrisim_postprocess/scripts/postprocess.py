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

class Task(Enum):
    DELIVERY = "delivery"
    INVENTORY = "inventory"
    CLEANING = "cleaning"
    CHARGING = "charging"
    
class TaskResult(Enum):
    SUCCESS = 1
    FAILURE = -1
    WIP = 0

TASKS = {
    Task.DELIVERY.value: 0,
    Task.INVENTORY.value: 1,
    Task.CLEANING.value: 2,
    Task.CHARGING.value: 3
}

class Robot():
    def __init__(self, gx, gy) -> None:
        self.x = 0
        self.y = 0
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
    
    def __init__(self, gx = NOGOAL, gy = NOGOAL):
        """
        Class constructor. Init publishers and subscribers
        """
        self.rostime = 0
        self.robot = Robot(gx, gy)
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
        if self.robot.taskOn and self.robot.goalReached:
            self.robot.task_result = TaskResult.SUCCESS.value
        else:
            self.robot.task_result = TaskResult.FAILURE.value
            
        self.robot.gx = goal.goal.target_pose.pose.position.x
        self.robot.gy = goal.goal.target_pose.pose.position.y
    
        
    def cb_robot_task(self, task: String):
        self.robot.task = TASKS[task.data] if task.data != 'none' else -1
        
        
    def cb_people_counter(self, wps: WPPeopleCounters):
        self.peopleAtWork = wps.numberOfWorkingPeople
        self.timeOfDay = wps.counters[0].time.data
        for wp in wps.counters:
            self.WPs[wp.WP_id.data] = wp.numberOfPeople
            self.PDs[wp.WP_id.data] = wp.numberOfPeople/AREAS[WP_AREA[wp.WP_id.data]].area
            
            
    def cb_time(self, t: pT):
        self.timeOfDay = t.time_of_the_day.data
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
        self.robot.closest_wp = wp.data
        

def save_data_to_csv(data_rows, filename, csv_path, wps, general_columns_name):
    df = pd.DataFrame(data_rows)
    df.to_csv(f"{csv_path}/{filename}.csv", index=False)
    rospy.logwarn(f"Saved {filename}.csv")

    # Save WP-specific DataFrames
    for wp_id, np in wps.items():
        wp_df = pd.DataFrame(
            [{key: row[key] for key in row if key in (general_columns_name + [f"{wp_id}_np", f"{wp_id}_pd", f"{wp_id}_bac"])} for row in data_rows]
        )
        wp_filename = f"{filename}_{wp_id}.csv"
        wp_df.to_csv(f"{csv_path}/{wp_filename}", index=False)
        rospy.logwarn(f"Saved {wp_filename}")


def shutdown_callback(data_rows, bagname, current_time_of_day, csv_path, data, general_columns_name):
    rospy.logwarn("Shutting down and saving data.")
    filename = f"{bagname}_{current_time_of_day}"
    
    if data.robot.taskOn and not data.robot.goalReached:
        goal_data = {'x': data.robot.gx, 'y': data.robot.gy}
        json_filename = os.path.join(csv_path, "goal.json")

        # Save the goal data into a JSON file
        with open(json_filename, 'w') as json_file:
            json.dump(goal_data, json_file)
        rospy.loginfo(f"Saved goal coordinates to {json_filename}")
    
    save_data_to_csv(data_rows, filename, csv_path, data.WPs, general_columns_name)



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
        


if __name__ == '__main__':
    
    BAGNAME = sys.argv[1]
    CSV_PATH = os.path.join(sys.argv[2], 'csv')
    SCENARIO = os.path.join(sys.argv[2], 'scenarios', f'{sys.argv[3]}.xml')
    TIMEOFTHEDAY = sys.argv[4]
    LOADGOAL = True if sys.argv[5] == 'true' else False
    WPS = readScenario()
    if LOADGOAL:
        goal_file = os.path.join(CSV_PATH, "goal.json")
        if os.path.exists(goal_file):
            with open(goal_file, 'r') as json_file:
                G = json.load(json_file)
            rospy.loginfo(f"Loaded goal coordinates from {goal_file}")
    else:
        G = {'x': NOGOAL, 'y': NOGOAL}
    
    # Map waypoints to clusters
    WP_AREA = {}
    for wp, coords in WPS.items():
        point = Point(coords['x'], coords['y'])
        for cluster_name, cluster_polygon in AREAS.items():
            if cluster_polygon.contains(point):
                WP_AREA[wp] = cluster_name
                break

    # Init node
    rospy.init_node(NODE_NAME)

    # Set node rate
    rate = rospy.Rate(NODE_RATE)

    data_handler = DataManager(gx = G['x'], gy = G['y'])

    # Initialize variables to track timeOfDay
    current_time_of_day = None
    recording = False
    data_rows = []  # List to store collected data for each segment

    # Register shutdown callback
    rospy.on_shutdown(lambda: shutdown_callback(
        data_rows, BAGNAME, current_time_of_day, CSV_PATH, data_handler, GENERAL_COLUMNS_NAME))

    while not rospy.is_shutdown():
        if data_handler.timeOfDay == '':
            continue

        # Start recording when timeOfDay matches the specified time
        if not recording and data_handler.timeOfDay == TIMEOFTHEDAY:
            current_time_of_day = data_handler.timeOfDay
            rospy.logwarn(f"Started recording at {TIMEOFTHEDAY}")
            recording = True

        # Stop recording and trigger shutdown when timeOfDay changes
        if recording and data_handler.timeOfDay != TIMEOFTHEDAY:
            rospy.logwarn(f"Time of day changed from {TIMEOFTHEDAY}, triggering shutdown")
            rospy.signal_shutdown("Time of day changed")

        if recording:
            # Collect data for the current time step
            data_row = {
                'ros_time': data_handler.rostime,
                'pf_elapsed_time': data_handler.elapsed,
                'time_of_day': data_handler.timeOfDay,
                'hhmmss': data_handler.hhmmss,
                'r_wp': data_handler.robot.closest_wp,
                'r_x': data_handler.robot.x,
                'r_y': data_handler.robot.y,
                'r_yaw': data_handler.robot.yaw,
                'r_v': data_handler.robot.v,
                'g_x': data_handler.robot.gx,
                'g_y': data_handler.robot.gy,
                'r_task': data_handler.robot.task,
                'r_T': data_handler.robot.task_result,
                'r_battery': data_handler.robot.battery_level,
                'is_charging': 1 if data_handler.robot.is_charging else 0,
                'people_at_work': data_handler.peopleAtWork,
            }
            
            data_handler.robot.task_result = TaskResult.WIP.value

            # Collect agents' data
            for agent_id, agent in data_handler.agents.items():
                data_row[f'a{agent_id}_x'] = agent.x
                data_row[f'a{agent_id}_y'] = agent.y
                data_row[f'a{agent_id}_yaw'] = agent.yaw
                data_row[f'a{agent_id}_v'] = agent.v

            GENERAL_COLUMNS_NAME = list(data_row.keys())

            # Collect WP' data
            for wp_id in data_handler.WPs.keys():
                data_row[f'{wp_id}_np'] = data_handler.WPs[wp_id]
            for wp_id in data_handler.PDs.keys():
                data_row[f'{wp_id}_pd'] = data_handler.PDs[wp_id]
            for wp_id in data_handler.BACs.keys():
                data_row[f'{wp_id}_bac'] = data_handler.BACs[wp_id]

            # Append the row to the list
            data_rows.append(data_row)

        rate.sleep()