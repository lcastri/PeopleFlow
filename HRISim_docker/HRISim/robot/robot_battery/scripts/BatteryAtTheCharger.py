#!/usr/bin/env python

import math
import pickle
import random
import rospy
from robot_msgs.msg import BatteryStatus, BatteryAtCharger as msgBAC, BatteryAtChargers as msgBACs
from std_msgs.msg import Header
import networkx as nx
import hrisim_util.ros_utils as ros_utils


def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def get_TTC():
    ttc = {}
    for wp in WPS:
        path = nx.astar_path(G, wp, "charging_station", heuristic=heuristic, weight='weight')
        distanceToCharger = 0
        for wp_idx in range(1, len(path)):
            wp_current = path[wp_idx-1]
            wp_next = path[wp_idx]
            distanceToCharger += math.sqrt((WPS[wp_next]['x'] - WPS[wp_current]['x'])**2 + (WPS[wp_next]['y'] - WPS[wp_current]['y'])**2)
        
        timeToCharger = math.ceil(distanceToCharger/ROBOT_MAX_VEL)
        ttc[wp] = timeToCharger
    return ttc


class BatteryAtCharger():
    def __init__(self):
        rospy.Subscriber("/hrisim/robot_battery", BatteryStatus, self.cb_robot_battery)
        self.bac_pub = rospy.Publisher('/hrisim/robot_bac', msgBACs, queue_size=10)
        self.ttc = get_TTC()
           
    def cb_robot_battery(self, b: BatteryStatus):
        msg = msgBACs()
        msg.header = Header()
        
        battery_level = b.level.data
        for wp in WPS:
            bac = msgBAC()
            bac.BAC.data = battery_level - self.ttc[wp] * (STATIC_CONSUMPTION + DYNAMIC_CONSUMPTION * ROBOT_MAX_VEL)
            bac.WP_id.data = wp
            msg.BACs.append(bac)
        
        self.bac_pub.publish(msg)    
        
       
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