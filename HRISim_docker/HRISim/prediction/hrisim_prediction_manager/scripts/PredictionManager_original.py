#!/usr/bin/env python

import math
import os
import pickle
import time
import numpy as np
import pandas as pd
import rospy
import hrisim_util.ros_utils as ros_utils
import hrisim_util.constants as constants
from hrisim_prediction_srvs.srv import GetRiskMap, GetRiskMapResponse
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from peopleflow_msgs.msg import WPPeopleCounters, Time as pT
from robot_msgs.msg import BatteryStatus, BatteryAtChargers
from std_msgs.msg import String
from causalflow.basics.constants import *
from causalflow.causal_reasoning.CausalInferenceEngine import CausalInferenceEngine
from collections import deque
import networkx as nx

class Robot():
    def __init__(self) -> None:
        self.x = None
        self.y = None
        self.yaw = 0
        self.v = 0
        self.battery_level = 0
        self.is_charging = 0
        self.closest_wp = ''
        self.task_result = 0
        

def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
       

class PredictionManager:
    def __init__(self):
        """
        Class constructor. Init publishers and subscribers
        """
        self.robot = Robot()

        self.WPs = {}
        self.BACs = {}
        self.PDs = {}

        self.TOD = ''        
        self.hhmmss = ''
        self.elapsed = 0

        # subscribers
        rospy.Subscriber("/robot_pose", PoseWithCovarianceStamped, self.cb_robot_pose)
        rospy.Subscriber("/mobile_base_controller/odom", Odometry, self.cb_odom)
        rospy.Subscriber("/peopleflow/counter", WPPeopleCounters, self.cb_people_counter)
        rospy.Subscriber("/peopleflow/time", pT, self.cb_time)
        rospy.Subscriber("/hrisim/robot_battery", BatteryStatus, self.cb_robot_battery)
        rospy.Subscriber("/hrisim/robot_bac", BatteryAtChargers, self.cb_robot_bac)
        rospy.Subscriber("/hrisim/robot_closest_wp", String, self.cb_robot_closest_wp)
        
        self.CIE = CausalInferenceEngine.load(CIEDIR)
        self.DAG = self.CIE.DAG['complete']
        self.MAX_LAG = self.DAG.max_lag
        
        dag = self.CIE.remove_intVarParents(self.DAG, 'R_V')
        self.calculation_order = list(nx.topological_sort(self.CIE.DAG2NX(dag)))
            
        self.observations = deque(maxlen=self.MAX_LAG + 1)  # Store up to MAX_LAG + 1 steps (current and previous)
        self.service = None
        
        rospy.set_param('/hrisim/prediction_ready', True)
             
                                      
    def cb_robot_pose(self, pose: PoseWithCovarianceStamped):
        self.robot.x, self.robot.y, self.robot.yaw = ros_utils.getPose(pose.pose.pose)
        
        
    def cb_odom(self, odom: Odometry):
        self.robot.v = abs(odom.twist.twist.linear.x)
        
        
    def cb_robot_closest_wp(self, wp: String):
        self.robot.closest_wp = wp.data
        
        
    def cb_robot_battery(self, b: BatteryStatus):
        self.robot.battery_level = b.level.data
        self.robot.is_charging = b.is_charging.data
        
        
    def cb_robot_bac(self, bacs: BatteryAtChargers):
        for bac in bacs.BACs:
            self.BACs[bac.WP_id.data] = bac.BAC.data
    
    
    def cb_people_counter(self, wps: WPPeopleCounters):
        self.peopleAtWork = wps.numberOfWorkingPeople
        for wp in wps.counters:
            self.WPs[wp.WP_id.data] = wp.numberOfPeople
            self.PDs[wp.WP_id.data] = wp.numberOfPeople/WPS_COORD[wp.WP_id.data]['A']
            
            
    def cb_time(self, t: pT):
        self.TOD = int(ros_utils.seconds_to_hh(t.elapsed))
        self.hhmmss = t.hhmmss.data
        self.elapsed = t.elapsed
            
            
    def get_treatment_len(self):        
        # Calculate the prediction horizon based on the time needed to reach the furthest waypoint
        travelled_distances = []
        for wp in WPS_COORD.keys():
            path = nx.astar_path(G, self.robot.closest_wp, wp, heuristic=heuristic, weight='weight')
            travelled_distance = 0
            for wp_idx in range(1, len(path)):
                wp_current = path[wp_idx-1]
                wp_next = path[wp_idx]
                travelled_distance += math.sqrt((WPS_COORD[wp_next]['x'] - WPS_COORD[wp_current]['x'])**2 + (WPS_COORD[wp_next]['y'] - WPS_COORD[wp_current]['y'])**2)
            travelled_distances.append(travelled_distance)
        return math.ceil((max(travelled_distances)/ROBOT_MAX_VEL)/PREDICTION_STEP)
            
            
    def collect_data(self):
        """
        Collects the current state of all data and logs or processes it.
        """
        # Check that self.PDs has exactly the same keys as WPS_COORD
        if (set(self.PDs.keys()) != set(WPS_COORD.keys())) or (set(self.BACs.keys()) != set(WPS_COORD.keys())):
            rospy.logerr("Mismatch between PDs/BACs keys and WPS_COORD keys")
            return
        
        # Create a dictionary for the current data
        current_data = {
            "TOD": self.TOD,
            "R_V": self.robot.v,
            "R_B": self.robot.battery_level,
            "B_S": 1 if self.robot.is_charging else 0,
        }
        for wp in WPS_COORD.keys():
            current_data[f"PD_{wp}"] = self.PDs[wp]
            current_data[f"BAC_{wp}"] = self.BACs[wp]
        # for wp in self.PDs.keys():
        #     current_data[f"PD_{wp}"] = self.PDs[wp] #! version without BAC

        # Add current data to the sliding window
        self.observations.append(current_data)
        if self.service is None: 
            self.service = rospy.Service('/get_risk_map', GetRiskMap, PM.handle_get_risk_map)
            
            
    def elapsed2TOD(self, t):
        d = 0
        for time in SCHEDULE:
            d += SCHEDULE[time]['duration']
            if t > d:
                continue
            else:
                return constants.TODS[SCHEDULE[time]['name']]
        
        
    def handle_get_risk_map(self, req):
        treatment_len = self.get_treatment_len()
        rospy.logwarn(f"Treatment length: {treatment_len}")
        rospy.logwarn(f"Treatment seconds: {treatment_len*PREDICTION_STEP}")
        
        # Convert the observations deque to a pandas DataFrame
        data = pd.DataFrame(list(self.observations))
               
        # Init output
        flattened_PDs = []
        flattened_BACs = []
        
        for i, wp in enumerate(WPS_COORD.keys()):
            # For each waypoint, pass the corresponding data to the causal inference engine
            # wp_obs = data[["TOD", "R_V", "R_B", "B_S", f"PD_{wp}"]].values #! version without BAC
            wp_obs = data[["TOD", "R_V", "R_B", "B_S", f"PD_{wp}", f"BAC_{wp}"]].values
            # wp_obs_df = pd.DataFrame(wp_obs, columns=["TOD", "R_V", "R_B", "B_S", "PD"]) #! version without BAC
            wp_obs_df = pd.DataFrame(wp_obs, columns=["TOD", "R_V", "R_B", "B_S", "PD", "BAC"])
            wp_obs_df["WP"] = constants.WPS[wp]
            
            # Init prior knowledge
            prior_knowledge = {f: np.full(treatment_len, wp_obs_df[f].values[-1]) for f in ['B_S', 'WP']}
            prior_knowledge['TOD'] = [self.elapsed2TOD(self.elapsed + i * PREDICTION_STEP) for i in range(treatment_len)]
            if i > 0: prior_knowledge['R_B'] = prediction_df['R_B'].values
            
            # start_time_cie = time.time()
            res = self.CIE.whatIf('R_V', 
                                  ROBOT_MAX_VEL * np.ones(treatment_len), 
                                  wp_obs_df.values,
                                  prior_knowledge,
                                  self.calculation_order
                                 )
            # end_time_cie = time.time()
            # rospy.logwarn(f"Time elapsed CIE: {end_time_cie - start_time_cie}")

            # prediction_df = pd.DataFrame(res, columns=["TOD", "R_V", "R_B", "B_S", "PD", "WP"]) #! version without BAC
            prediction_df = pd.DataFrame(res, columns=["TOD", "R_V", "R_B", "B_S", "PD", "BAC", "WP"])
            if wp == constants.WP.CHARGING_STATION.value: prediction_df["BAC"] = prediction_df["R_B"]
            flattened_PDs.extend(prediction_df['PD'].values)
            # flattened_BACs.extend(prediction_df['R_B'].values) #! version without BACs
            flattened_BACs.extend(prediction_df['BAC'].values)
        
        return GetRiskMapResponse(list(self.PDs.keys()), 
                                  treatment_len, 
                                  len(self.PDs.keys()),
                                  flattened_PDs,
                                  flattened_BACs)
        

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('prediction_manager')
    
    CIEDIR = os.path.join(rospy.get_param("~CIE"), "cie.pkl")
    PREDICTION_STEP = rospy.get_param("~pred_step")
    ROBOT_MAX_VEL = float(ros_utils.wait_for_param("/move_base/TebLocalPlannerROS/max_vel_x"))
    g_path = ros_utils.wait_for_param("/peopleflow_pedsim_bridge/g_path")
    with open(g_path, 'rb') as f:
        G = pickle.load(f)

    SCHEDULE = ros_utils.wait_for_param("/peopleflow/schedule")
    WPS_COORD = ros_utils.wait_for_param("/peopleflow/wps")
    for wp in WPS_COORD:
        WPS_COORD[wp]['A'] = math.pi * WPS_COORD[wp]['r']**2
    
    PM = PredictionManager()
    
    rate = rospy.Rate(1 / PREDICTION_STEP)
        
    rospy.loginfo("Starting periodic data collection.")
    while not rospy.is_shutdown():
        PM.collect_data()
        rate.sleep()