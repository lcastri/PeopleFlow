#!/usr/bin/env python

import math
import pickle
import numpy as np
import rospy
import hrisim_util.ros_utils as ros_utils
import hrisim_util.constants as constants
from hrisim_prediction_srvs.srv import GetRiskMap, GetRiskMapResponse
from hrisim_people_counter.msg import WPPeopleCounters
from std_msgs.msg import String
import networkx as nx
import pyAgrum
import pyAgrum.causal as pyc
        

def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def get_info(D, auditDict, var):
    if auditDict[var]['method'] == 'NoDiscretization':
        edges = auditDict[var]['values']
        midpoints = auditDict[var]['values']
        return 'NoDiscretization', edges, midpoints
    elif 'param' in auditDict[var] and isinstance(auditDict[var]['param'], list):
        edges = auditDict[var]['param']
        midpoints = [(edges[i] + edges[i+1]) / 2.0 for i in range(len(edges)-1)]
        return auditDict[var]['param'], edges, midpoints
    quantiles = np.linspace(0, 100, auditDict[var]['param'] + 1 if 'param' in auditDict[var] else auditDict[var]['nbBins'] + 1)
    edges = np.percentile(D[var].values, quantiles)
    midpoints = [(edges[i] + edges[i+1]) / 2.0 for i in range(len(edges)-1)]
    return quantiles, edges, midpoints
       

class PredictionManager:
    def __init__(self):
        """
        Class constructor. Init publishers and subscribers
        """
        self.robot_wp = ''
        self.WPs = {}
        self.PDs = {}

        # subscribers
        rospy.Subscriber("/hrisim/people_counter", WPPeopleCounters, self.cb_people_counter)
        rospy.Subscriber("/hrisim/robot_closest_wp", String, self.cb_robot_closest_wp)
        
        AUDITs = pickle.load(open(f"{CIEDIR}/AUDITs.pkl", "rb"))
        Ds = pickle.load(open(f"{CIEDIR}/Ds.pkl", "rb"))
        self.CIE = {}
        for wp in WPS_COORD.keys():
            bn = pyAgrum.loadBN(f"{CIEDIR}/CIE_{wp}.bifxml")
            cm = pyc.CausalModel(bn)
            self.CIE[wp] = {'audit': AUDITs[wp], 'd': Ds[wp], 'bn': bn, 'cm': cm}
               
        self.TTWP = self.get_ttwp()
        self.ARCs = []
        for arc in self.TTWP.keys():
            self.ARCs.append('__'.join(arc))
        rospy.Service('/hrisim/riskMap/predict', GetRiskMap, self.handle_get_risk_map)
        
        rospy.set_param('/hrisim/prediction_ready', True)
        rospy.logwarn("Prediction Manager ready!")     
        
        
    def cb_robot_closest_wp(self, wp: String):
        self.robot_wp = wp.data
    
    
    def cb_people_counter(self, wps: WPPeopleCounters):
        for wp in wps.counters:
            self.WPs[wp.WP_id.data] = wp.numberOfPeople
            self.PDs[wp.WP_id.data] = wp.numberOfPeople/WPS_COORD[wp.WP_id.data]['A']
            
            
    def get_ttwp(self, relative=False):
        """
        Calculate the time to travel between each pair of waypoints in the graph.

        Returns:
            dict: A dictionary with waypoint pairs as keys and travel times as values.
        """
        travelled_distances = {}
        if relative:
            for wp in WPS_COORD.keys():
                path = nx.astar_path(G, self.robot_wp, wp, heuristic=heuristic, weight='weight')
                travelled_distance = 0
                for wp_idx in range(1, len(path)):
                    wp_current = path[wp_idx-1]
                    wp_next = path[wp_idx]
                    travelled_distance += math.sqrt((WPS_COORD[wp_next]['x'] - WPS_COORD[wp_current]['x'])**2 + (WPS_COORD[wp_next]['y'] - WPS_COORD[wp_current]['y'])**2)
                travelled_distances[wp] = math.ceil((travelled_distance/ROBOT_MAX_VEL)/PREDICTION_STEP)
        else:
            for arc in G.edges():
                wp_i, wp_j = arc
                travelled_distance = math.sqrt((WPS_COORD[wp_i]['x'] - WPS_COORD[wp_j]['x'])**2 + (WPS_COORD[wp_i]['y'] - WPS_COORD[wp_j]['y'])**2)
                travelled_distances[arc] = math.ceil((travelled_distance/ROBOT_MAX_VEL)/PREDICTION_STEP)
        return travelled_distances
                       
    
    
    def predict_PD(self, tod, wp):

        wp_bin = 0
        tod_bin = constants.SCENARIOS[tod]['id']
        evidence = {"TOD0": tod_bin, "WP0": wp_bin}
              
        cm = self.CIE[wp]['cm']
        dwp = self.CIE[wp]['d']
               
        _, _, midpoints_PD = get_info(dwp, self.CIE[wp]['audit'], 'PD0')
      
                                
        # --- CausalModel prediction ---
        _, adj, _ = pyc.causalImpact(cm, on="PD0", doing="TOD0", knowing={"WP0"}, values=evidence)
        posterior_causal = adj.toarray()
        pred_causal = sum(posterior_causal[j] * midpoints_PD[j] for j in range(len(posterior_causal)))

        return pred_causal
    

    def handle_get_risk_map(self, req):
        rospy.logwarn("Prediction requested!")
                       
        # Init output
        PD_wps = {}
        PDs = []
        
        for wp in WPS_COORD.keys():
            PD_wps[wp] = self.predict_PD(TS, wp)
            
        for arc in self.ARCs:
            wp_i, wp_j = arc.split('__')
            PDs.append((PD_wps[wp_i] + PD_wps[wp_j])/2)
            
        rospy.logerr(self.ARCs)
        rospy.logerr(PDs)
        return GetRiskMapResponse(self.ARCs, PDs)
        

if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('prediction_manager')
    
    CIEDIR = rospy.get_param("~CIE")
    PREDICTION_STEP = rospy.get_param("~pred_step")
    ROBOT_MAX_VEL = float(ros_utils.wait_for_param("/move_base/PalLocalPlanner/max_vel_x"))
    g_path = ros_utils.wait_for_param("/graph_publisher/graph_file")
    with open(g_path, 'rb') as f:
        G = pickle.load(f)

    TS = ros_utils.wait_for_param("/hrisim/time_slot")
    WPS_COORD = ros_utils.wait_for_param("/hrisim/wps")
    for wp in WPS_COORD:
        WPS_COORD[wp]['A'] = math.pi * WPS_COORD[wp]['r']**2
    
    PM = PredictionManager()
    
    rate = rospy.Rate(1 / PREDICTION_STEP)
        
    while not rospy.is_shutdown():
        TS = rospy.get_param("/hrisim/time_slot")
        rate.sleep()