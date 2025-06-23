#!/usr/bin/env python

import math
import pickle
import rospy
import networkx as nx
from robot_srvs.srv import PathRequest, PathRequestResponse
import hrisim_util.ros_utils as ros_utils
from robot_srvs.srv import VisualisePath
import numpy as np
from functools import partial
from std_srvs.srv import Empty


def read_risk_map():
    risk_map_data = rospy.get_param('/hrisim/risk_map')
    tmp_ARCs = risk_map_data['arcs']
    ARCs = [(arc.split("__")[0], arc.split("__")[1]) for arc in tmp_ARCs]
    PDs = risk_map_data['PDs']

    risk_map = {}
    for i, arc in enumerate(ARCs):
        risk_map[arc] = {
            'PD': PDs[i],
        }
    return risk_map


class Navigator():
    def __init__(self, g_path) -> None:
        self.g_path = g_path
        self.risk_map = None
            
        ros_utils.wait_for_service('/graph/path/show')
        self.graph_path_show = rospy.ServiceProxy('/graph/path/show', VisualisePath)
        self.graph_weight_update = rospy.ServiceProxy('/graph/weights/update', Empty)

        rospy.Service('/hrisim/path', PathRequest, self.cb_pathrequest)

            
    def heuristic(self, a, b):
        pos = nx.get_node_attributes(self.G, 'pos')
        (x1, y1) = pos[a]
        (x2, y2) = pos[b]
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    
    
    def causal_heuristic(self, a, b):
    # def causal_heuristic(self, a, b, max_d, max_pd):
        pos = nx.get_node_attributes(self.G, 'pos')
        
        if (a, b) not in self.risk_map:
            return 100000

        # Calculate distance cost
        (x1, y1) = pos[a]
        (x2, y2) = pos[b]
        D_cost = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

        # Get PD values
        PD_cost = self.risk_map[(a, b)]['PD']

        # Combine with weighting factors
        max_d = 1
        max_pd = 1
        rospy.logerr(f"ARC {a}-{b}: D: {K_D * D_cost/max_d} --- PD: {K_PD * PD_cost/max_pd}")
        return K_D * D_cost/max_d + K_PD * PD_cost/max_pd
    
    # def causal_heuristic(self, a, b, max_travel, max_pd_cost):
    #     pos = nx.get_node_attributes(self.G, 'pos')

    #     # Calculate distance cost
    #     (x1, y1) = pos[a]
    #     (x2, y2) = pos[b]
    #     distance_cost = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    #     normalized_distance_cost = distance_cost / max_travel

    #     # Calculate PD cost
    #     # Time to travel from the robot to nodes a and b
    #     (xr, yr) = pos[a]
    #     a_PD_ttr = ((x1 - xr) ** 2 + (y1 - yr) ** 2) ** 0.5 / ROBOT_MAX_VEL
    #     b_PD_ttr = ((x2 - xr) ** 2 + (y2 - yr) ** 2) ** 0.5 / ROBOT_MAX_VEL

    #     idx_a = math.ceil(a_PD_ttr / PRED_STEP)
    #     idx_b = math.ceil(b_PD_ttr / PRED_STEP)

    #     # Bound indices
    #     idx_a = min(len(self.risk_map[a]['PD']) - 1, idx_a)
    #     idx_b = min(len(self.risk_map[b]['PD']) - 1, idx_b)

    #     # Get PD values
    #     PD_a = self.risk_map[a]['PD'][idx_a]
    #     PD_b = self.risk_map[b]['PD'][idx_b]
    #     avg_PD_cost = (PD_a + PD_b) / 2

    #     normalized_PD_cost = avg_PD_cost / max_pd_cost if max_pd_cost > 0 else 0
    #     rospy.logerr(f"ARC {a}-{b}: D: {K_D * normalized_distance_cost} --- PD: {K_PD * normalized_PD_cost}")
    #     # Combine with weighting factors
    #     return K_D * normalized_distance_cost + K_PD * normalized_PD_cost
    
    
    def compute_max_values(self):
        pos = nx.get_node_attributes(self.G, 'pos')
        D_costs = []
        PD_costs = []

        # Calculate all edge travel distances
        for u, v in self.G.edges():
            (x1, y1) = pos[u]
            (x2, y2) = pos[v]
            D_cost = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
            D_costs.append(D_cost)

            if (u, v) in self.risk_map:
                PD_costs.append(self.risk_map[(u, v)]['PD'])

        max_D = max(D_costs) if D_costs else 1
        max_PD_cost = max(PD_costs) if PD_costs else 1

        return max_D, max_PD_cost
    
    
    def update_G_weights(self, origin, max_d, max_pd):    
        
        # Get the position information from the graph
        pos = nx.get_node_attributes(self.G, 'pos')
        
        D_costs = []
        PD_costs = []
        for u, v in self.G.edges():
            # Calculate travel distance between nodes u and v
            (x1, y1) = pos[u]
            (x2, y2) = pos[v]
            D_cost = ((x1 - x2)**2 + (y1 - y2)**2)**0.5
            D_costs.append(D_cost)

            if (u, v) in self.risk_map:
                # # Estimate time it will take to reach the next node (simple time = distance / speed)
                # (xr, yr) = pos[origin]
                # u_PD_ttr = ((x1 - xr)**2 + (y1 - yr)**2)**0.5 / ROBOT_MAX_VEL  # Time in seconds to move from r to u
                # v_PD_ttr = ((x2 - xr)**2 + (y2 - yr)**2)**0.5 / ROBOT_MAX_VEL  # Time in seconds to move from r to v
                
                # # Find the closest time step to target_time based on your time steps
                # idx_u = math.ceil(u_PD_ttr / PRED_STEP)
                # idx_v = math.ceil(v_PD_ttr / PRED_STEP)

                # # Bound the indices to make sure they are within valid range (for list access)
                # idx_u = min(len(self.risk_map[u]['PD']) - 1, idx_u)
                # idx_v = min(len(self.risk_map[v]['PD']) - 1, idx_v)
                
                # # Get PD values at the estimated times for both u and v
                # PD_u = self.risk_map[u]['PD'][idx_u] 
                # PD_v = self.risk_map[v]['PD'][idx_v] 
                
                # Average PD cost (from current and neighbor nodes) 
                PD_cost = self.risk_map[(u, v)]['PD']
            else:
                PD_cost = max_pd
            
            PD_costs.append(PD_cost)
                
        normalized_D_costs = [d / max_d for d in D_costs]
        normalized_PD_costs = [c / max_pd for c in PD_costs]

        # Apply normalization and scaling factors
        for idx, (u, v) in enumerate(self.G.edges()):
            travel_distance = normalized_D_costs[idx]
            PD_cost = normalized_PD_costs[idx]
            
            # Combine the normalized travel cost and PD cost
            weight = K_D * D_cost + K_PD * PD_cost
            
            # Assign the combined weight to the edge between u and v
            self.G[u][v]['weight'] = weight
            self.G[u][v]['D'] = K_D * travel_distance
            self.G[u][v]['PD'] = K_PD * PD_cost
            

    def cb_pathrequest(self, request):
        """
        Service callback to PathRequest.srv.
        The service receives origin and destination as Strings and returns the next waypoint as a String.
        """
        with open(self.g_path, 'rb') as f:
            self.G = pickle.load(f)
        rospy.loginfo(f"Received path request from {request.origin} to {request.destination} | read_risk: {request.read_risk}")
        
        if not request.read_risk:
            queue = nx.astar_path(self.G, request.origin, request.destination, heuristic=self.heuristic, weight='weight')
        else:
            self.risk_map = read_risk_map()
            rospy.logerr(self.risk_map)
            # Compute the maximum values
            max_D, max_PD = self.compute_max_values()
            # max_d, max_pd = 1, 1
            
            # Use a partial function to fix the extra arguments
            # causal_heuristic_predefined = partial(self.causal_heuristic, max_d=max_D, max_pd=max_PD)
            
            self.update_G_weights(request.origin, max_D, max_PD)
            rospy.logerr(self.G.edges(data=True))
            ros_utils.load_graph_to_rosparam(self.G, "/hrisim/G")
            self.graph_weight_update()
            
            queue = nx.astar_path(self.G, request.origin, request.destination, heuristic=self.causal_heuristic, weight='weight')
            
        self.graph_path_show(','.join(queue))
        return PathRequestResponse(','.join(queue))
    
        

if __name__ == "__main__":
    rospy.init_node('Astar_navigation')
    PRED_STEP = ros_utils.wait_for_param('hrisim_prediction_manager/pred_step')
    ROBOT_MAX_VEL = float(ros_utils.wait_for_param("/move_base/PalLocalPlanner/max_vel_x"))
    K_D = 1
    K_PD = 10
    
    Navigator(g_path = rospy.get_param('~graph_path'))
    
    rospy.spin()
