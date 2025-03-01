#!/usr/bin/env python

import pickle

import numpy as np
import rospy
from robot_msgs.msg import BatteryStatus, ExpectedLifetime as msgELT, ExpectedLifetimes as msgELTs
from std_msgs.msg import Header
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
        self.robot_obs = bool(ros_utils.wait_for_param('/hrisim/robot_obs'))
        self.num_wps = len(WPS)

        self.elt_pub = rospy.Publisher('/hrisim/robot_elt', msgELTs, queue_size=10)
        
        self.noload_battery_lookup = self.precompute_battery_consumptions()
        self.load_battery_lookup = self.precompute_battery_consumptions(load=True)
        rospy.Subscriber("/hrisim/robot_battery", BatteryStatus, self.cb_robot_battery)
        
           
    def precompute_battery_consumptions(self, load=False):
        """Precompute battery consumption from waypoints to charger as a NumPy array."""
        battery_consumption_per_time = (KS_LOAD + KD_LOAD * ROBOT_MAX_VEL) if load else (KS_NOLOAD + KD_NOLOAD * ROBOT_MAX_VEL)

        charger_wp = constants.WP.CHARGING_STATION.value
        battery_lookup = np.zeros(self.num_wps, dtype=np.float32)

        for i, start_wp in enumerate(WPS):
            time_to_wp = ros_utils.get_time_to_wp(G, start_wp, charger_wp, heuristic, ROBOT_MAX_VEL)
            battery_lookup[i] = time_to_wp * battery_consumption_per_time

        return battery_lookup
            
            
    def cb_robot_battery(self, b: BatteryStatus):
        """Fast lookup from precomputed NumPy tables instead of recomputing battery consumption."""
        msg = msgELTs()
        msg.header = Header()

        battery_level = b.level.data
        battery_lookup = self.load_battery_lookup if self.robot_obs else self.noload_battery_lookup

        # Vectorized subtraction for fast computation
        elt_values = battery_level - battery_lookup

        for i, wp in enumerate(WPS):
            elt = msgELT()
            elt.ELT.data = elt_values[i]
            elt.WP_id.data = wp
            msg.ELTs.append(elt)

        self.elt_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('robot_elt')
    rate = rospy.Rate(10)  # 1 Hz
    
    SCENARIO = str(ros_utils.wait_for_param("/peopleflow_manager/scenario"))
    G_PATH = str(ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path"))
    KS_NOLOAD = float(ros_utils.wait_for_param("/robot_battery/noload_static_consumption"))
    KD_NOLOAD = float(ros_utils.wait_for_param("/robot_battery/noload_dynamic_consumption"))
    KS_LOAD = float(ros_utils.wait_for_param("/robot_battery/load_static_consumption"))
    KD_LOAD = float(ros_utils.wait_for_param("/robot_battery/load_dynamic_consumption"))
    ROBOT_MAX_VEL = float(ros_utils.wait_for_param("/move_base/TebLocalPlannerROS/max_vel_x"))
    WPS = ros_utils.wait_for_param("/peopleflow/wps")
    
    with open(G_PATH, 'rb') as f:
        G = pickle.load(f)

    ELT = ExpectedLifetime()
    
    while not rospy.is_shutdown():
        ELT.robot_obs = bool(ros_utils.wait_for_param('/hrisim/robot_load'))