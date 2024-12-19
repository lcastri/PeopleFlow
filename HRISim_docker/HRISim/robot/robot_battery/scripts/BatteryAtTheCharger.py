#!/usr/bin/env python

import math
import pickle
import rospy
from robot_msgs.msg import BatteryStatus, BatteryAtCharger as msgBAC, BatteryAtChargers as msgBACs
from std_msgs.msg import Header, String
import networkx as nx
import hrisim_util.ros_utils as ros_utils

def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2)**2 + (y1 - y2)**2)**0.5

class BatteryAtCharger():
    def __init__(self):
        self.bac_pub = rospy.Publisher('/hrisim/robot_bac', msgBACs, queue_size=10)
        self.robot_closest_wp = None
        self.battery_consumption_per_time = STATIC_CONSUMPTION + DYNAMIC_CONSUMPTION * ROBOT_MAX_VEL
        rospy.Subscriber("/hrisim/robot_closest_wp", String, self.cb_robot_closest_wp)
        rospy.Subscriber("/hrisim/robot_battery", BatteryStatus, self.cb_robot_battery)
           
    def cb_robot_battery(self, b: BatteryStatus):
        msg = msgBACs()
        msg.header = Header()
        
        battery_level = b.level.data
        for wp in WPS:
            battery_to_wp = ros_utils.get_time_to_wp(G, self.robot_closest_wp, wp, heuristic, robot_speed=ROBOT_MAX_VEL) * self.battery_consumption_per_time
            battery_to_charger = ros_utils.get_time_to_wp(G, wp, 'charging_station', heuristic, robot_speed=ROBOT_MAX_VEL) * self.battery_consumption_per_time
                       
            bac = msgBAC()
            bac.BAC.data = battery_level - battery_to_wp - battery_to_charger
            bac.WP_id.data = wp
            msg.BACs.append(bac)
        
        self.bac_pub.publish(msg)
        
    def cb_robot_closest_wp(self, wp: String):
        self.robot_closest_wp = wp.data        
       
if __name__ == '__main__':
    rospy.init_node('robot_bac')
    rate = rospy.Rate(1)  # 1 Hz
    SCENARIO = str(ros_utils.wait_for_param("/peopleflow_manager/scenario"))
    G_PATH = str(ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path"))
    STATIC_CONSUMPTION = float(ros_utils.wait_for_param("/robot_battery/static_consumption"))
    DYNAMIC_CONSUMPTION = float(ros_utils.wait_for_param("/robot_battery/dynamic_consumption"))
    ROBOT_MAX_VEL = float(ros_utils.wait_for_param("/move_base/TebLocalPlannerROS/max_vel_x"))
    WPS = ros_utils.wait_for_param("/peopleflow/wps")
    with open(G_PATH, 'rb') as f:
        G = pickle.load(f)
    
    BAC = BatteryAtCharger()
    
    rospy.spin()