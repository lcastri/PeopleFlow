#!/usr/bin/env python

import pickle
import rospy
from robot_msgs.msg import BatteryStatus, ExpectedLifetime as msgELT, ExpectedLifetimes as msgELTs
from std_msgs.msg import Header, String
import networkx as nx
import hrisim_util.ros_utils as ros_utils
import hrisim_util.constants as constants

def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2)**2 + (y1 - y2)**2)**0.5

class ExpectedLifetime():
    def __init__(self):
        self.elt_pub = rospy.Publisher('/hrisim/robot_elt', msgELTs, queue_size=10)
        
        self.noload_battery_consumption_per_time = KS_NOLOAD + KD_NOLOAD * ROBOT_MAX_VEL
        self.noload_battery_lookup = self.precompute_battery_consumptions()
        
        self.load_battery_consumption_per_time = KS_LOAD + KD_LOAD * ROBOT_MAX_VEL
        self.load_battery_lookup = self.precompute_battery_consumptions()
        rospy.Subscriber("/hrisim/robot_closest_wp", String, self.cb_robot_closest_wp)
        rospy.Subscriber("/hrisim/robot_battery", BatteryStatus, self.cb_robot_battery)
        
        
    def precompute_battery_consumptions(self, load=False):
        """Precompute battery consumption from all waypoints to chargers."""
        battery_lookup = {}
        
        for start_wp in WPS:
            battery_lookup[start_wp] = {}
            for end_wp in WPS:
                time_to_wp = ros_utils.get_time_to_wp(G, start_wp, end_wp, heuristic, robot_speed=ROBOT_MAX_VEL)
                if load:
                    consumption = time_to_wp * self.load_battery_consumption_per_time
                else:
                    consumption = time_to_wp * self.noload_battery_consumption_per_time
                battery_lookup[start_wp][end_wp] = consumption
        
        return battery_lookup
            
            
    def cb_robot_battery(self, b: BatteryStatus):
        msg = msgELTs()
        msg.header = Header()
        
        battery_level = b.level.data

        for wp in WPS:
            
            battery_to_charger = self.noload_battery_lookup[wp][constants.WP.CHARGING_STATION.value]

            elt = msgELT()
            elt.ELT.data = battery_level - battery_to_charger
            elt.WP_id.data = wp
            msg.ELTs.append(elt)

        self.elt_pub.publish(msg)
        
        
    def cb_robot_closest_wp(self, wp: String):
        self.robot_closest_wp = wp.data        


if __name__ == '__main__':
    rospy.init_node('robot_elt')
    rate = rospy.Rate(1)  # 1 Hz
    
    SCENARIO = str(ros_utils.wait_for_param("/peopleflow_manager/scenario"))
    G_PATH = str(ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path"))
    KS_NOLOAD = float(ros_utils.wait_for_param("/robot_battery/noload_static_consumption"))
    KD_NOLOAD = float(ros_utils.wait_for_param("/robot_battery/noload_dynamic_consumption"))
    KS_LOAD = float(ros_utils.wait_for_param("/robot_battery/load_static_consumption"))
    KD_LOAD = float(ros_utils.wait_for_param("/robot_battery/nload_dynamic_consumption"))
    ROBOT_MAX_VEL = float(ros_utils.wait_for_param("/move_base/TebLocalPlannerROS/max_vel_x"))
    WPS = ros_utils.wait_for_param("/peopleflow/wps")
    
    with open(G_PATH, 'rb') as f:
        G = pickle.load(f)

    ELT = ExpectedLifetime()
    
    rospy.spin()