#!/usr/bin/env python

import math
import os
import pandas as pd
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from hrisim_people_counter.msg import WPPeopleCounters
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Int32, Bool
import hrisim_util.ros_utils as ros_utils
import xml.etree.ElementTree as ET
import time
from utils import *
from rosgraph_msgs.msg import Clock


NODE_NAME = 'robot_postprocess'
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
              

class DataManager():
    """
    Class handling data
    """
    
    def __init__(self):
        """
        Class constructor. Init publishers and subscribers
        """
    
        self.last_clock_time = time.time()
        self.rostime = 0
        self.robot = Robot()
        
        self.WPs = {}
        self.PDs = {}

        # subscribers
        rospy.Subscriber("/clock", Clock, self.cb_clock)
        rospy.Subscriber("/robot_pose", PoseWithCovarianceStamped, self.cb_robot_pose)
        rospy.Subscriber("/mobile_base_controller/odom", Odometry, self.cb_odom)
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.cb_robot_goal)
        rospy.Subscriber("/hrisim/people_counter", WPPeopleCounters, self.cb_people_counter)
        rospy.Subscriber("/power/battery_level", Float32, self.cb_robot_battery)
        rospy.Subscriber("/hrisim/robot_closest_wp", String, self.cb_robot_closest_wp)


    def cb_clock(self, clock: Clock):
        self.rostime = clock.clock.to_sec()
        self.last_clock_time = time.time()
        
        
    def cb_robot_pose(self, pose: PoseWithCovarianceStamped):
        self.robot.x, self.robot.y, self.robot.yaw = ros_utils.getPose(pose.pose.pose)
        
        
    def cb_odom(self, odom: Odometry):
        self.robot.v = abs(odom.twist.twist.linear.x)
        
        
    def cb_robot_goal(self, goal: MoveBaseActionGoal):            
        self.robot.gx = goal.goal.target_pose.pose.position.x
        self.robot.gy = goal.goal.target_pose.pose.position.y
        
        
    def cb_people_counter(self, wps: WPPeopleCounters):
        for wp in wps.counters:
            self.WPs[wp.WP_id.data] = wp.numberOfPeople
            self.PDs[wp.WP_id.data] = math.log1p(wp.numberOfPeople) / math.log1p(WPS_INFO[wp.WP_id.data]['A'])
            # self.PDs[wp.WP_id.data] = wp.numberOfPeople/WPS_INFO[wp.WP_id.data]['A']
            

    def cb_robot_battery(self, b: Float32):
        self.robot.battery_level = b.data
                    
        
    def cb_robot_closest_wp(self, wp: String):
        self.robot.closest_wp = WPS[wp.data]
           
        
def shutdown_callback(data_rows, filename, csv_path):   
    rospy.logwarn("Shutting down and saving data.")
    
    # Generating Task Result Column
    df = pd.DataFrame(data_rows)
        
    df.to_csv(f"{csv_path}/{filename}.csv", index=False)
    del df
    rospy.logwarn(f"Saved {filename}.csv")
    

def readScenario():
    # Load and parse the XML file
    tree = ET.parse(MAP)
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
    map = rospy.get_param('~map')
    rospy.loginfo(f"Processing {BAGNAME}")
    MAP = os.path.join(NODE_PATH, 'scenarios', f'{map}.xml')
    CSV_PATH = os.path.join(NODE_PATH, 'csv')
    os.makedirs(CSV_PATH, exist_ok=True)
    
    # Map waypoints
    WPS_INFO = readScenario()
    
    DM = DataManager()
    
    data_rows = []  # List to store collected data for each segment
    
    # Register the shutdown callback
    rospy.on_shutdown(lambda: shutdown_callback(data_rows, BAGNAME, CSV_PATH))
    
    while not rospy.is_shutdown():

                
        # Collect data for the current time step
        data_row = {
            'ros_time': DM.rostime,
            'TOD': TODS[BAGNAME],
            'R_X': DM.robot.x,
            'R_Y': DM.robot.y,
            'R_V': DM.robot.v,
            'G_X': DM.robot.gx,
            'G_Y': DM.robot.gy,
            'R_B': DM.robot.battery_level,
        }

        # Collect WP' data
        for wp_id in DM.WPs.keys():
            data_row[f'{wp_id}_NP'] = DM.WPs[wp_id]
        for wp_id in DM.PDs.keys():
            data_row[f'{wp_id}_PD'] = DM.PDs[wp_id]

        # Append the row to the list
        data_rows.append(data_row)

        rate.sleep()