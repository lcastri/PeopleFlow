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
    flattened_PDs = risk_map_data['PDs']
    n_waypoints = risk_map_data['n_waypoints']
    n_steps = risk_map_data['n_steps']

    PDs_matrix = np.array(flattened_PDs).reshape(n_waypoints, n_steps)

    risk_map = {}
    for i, wp in enumerate(risk_map_data['waypoint_ids']):
        risk_map[wp] = {
            'PD': PDs_matrix[i].tolist(),
        }
    return risk_map


class Navigator():
    def __init__(self, g_path) -> None:
        with open(g_path, 'rb') as f:
            self.G = pickle.load(f)
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
    
    
    def causal_heuristic(self, a, b, origin, max_travel, max_pd_cost):
        pos = nx.get_node_attributes(self.G, 'pos')

        # Calculate distance cost
        (x1, y1) = pos[a]
        (x2, y2) = pos[b]
        distance_cost = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
        normalized_distance_cost = distance_cost / max_travel

        # Calculate PD cost
        if a in self.risk_map and b in self.risk_map:
            # Time to travel from the robot to nodes a and b
            (xr, yr) = pos[origin]
            a_PD_ttr = ((x1 - xr) ** 2 + (y1 - yr) ** 2) ** 0.5 / 0.5
            b_PD_ttr = ((x2 - xr) ** 2 + (y2 - yr) ** 2) ** 0.5 / 0.5

            idx_a = math.ceil(a_PD_ttr / PRED_STEP)
            idx_b = math.ceil(b_PD_ttr / PRED_STEP)

            # Bound indices
            idx_a = min(len(self.risk_map[a]['PD']) - 1, idx_a)
            idx_b = min(len(self.risk_map[b]['PD']) - 1, idx_b)

            # Get PD values
            PD_a = self.risk_map[a]['PD'][idx_a]
            PD_b = self.risk_map[b]['PD'][idx_b]
            avg_PD_cost = (PD_a + PD_b) / 2
        else:
            avg_PD_cost = 0

        normalized_PD_cost = avg_PD_cost / max_pd_cost if max_pd_cost > 0 else 0

        # Combine with weighting factors
        return K_D * normalized_distance_cost + K_PD * normalized_PD_cost
    
    
    def compute_max_values(self):
        pos = nx.get_node_attributes(self.G, 'pos')
        travel_distances = []
        pd_costs = []

        # Calculate all edge travel distances
        for u, v in self.G.edges():
            (x1, y1) = pos[u]
            (x2, y2) = pos[v]
            travel_distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
            travel_distances.append(travel_distance)

            if u in self.risk_map and v in self.risk_map:
                PD_u = max(self.risk_map[u]['PD'])
                PD_v = max(self.risk_map[v]['PD'])
                pd_costs.append((PD_u + PD_v) / 2)

        max_travel = max(travel_distances) if travel_distances else 1
        max_pd_cost = max(pd_costs) if pd_costs else 1

        return max_travel, max_pd_cost
    
    
    def update_G_weights(self, origin, max_travel, max_pd_cost, robot_speed=0.5):    
        
        # Get the position information from the graph
        pos = nx.get_node_attributes(self.G, 'pos')
        
        travel_distances = []
        pd_costs = []
        for u, v in self.G.edges():
            # Calculate travel distance between nodes u and v
            (x1, y1) = pos[u]
            (x2, y2) = pos[v]
            travel_distance = ((x1 - x2)**2 + (y1 - y2)**2)**0.5
            travel_distances.append(travel_distance)

            if u in self.risk_map and v in self.risk_map:
                # Estimate time it will take to reach the next node (simple time = distance / speed)
                (xr, yr) = pos[origin]
                u_PD_ttr = ((x1 - xr)**2 + (y1 - yr)**2)**0.5 / robot_speed  # Time in seconds to move from r to u
                v_PD_ttr = ((x2 - xr)**2 + (y2 - yr)**2)**0.5 / robot_speed  # Time in seconds to move from r to v
                
                # Find the closest time step to target_time based on your time steps
                idx_u = math.ceil(u_PD_ttr / PRED_STEP)
                idx_v = math.ceil(v_PD_ttr / PRED_STEP)

                # Bound the indices to make sure they are within valid range (for list access)
                idx_u = min(len(self.risk_map[u]['PD']) - 1, idx_u)
                idx_v = min(len(self.risk_map[v]['PD']) - 1, idx_v)
                
                # Get PD values at the estimated times for both u and v
                PD_u = self.risk_map[u]['PD'][idx_u] 
                PD_v = self.risk_map[v]['PD'][idx_v] 
                
                # Average PD cost (from current and neighbor nodes) 
                PD_cost = (PD_u + PD_v) / 2
            else:
                PD_cost = max_pd_cost
            
            pd_costs.append(PD_cost)
                
        normalized_travel_distances = [d / max_travel for d in travel_distances]
        normalized_pd_costs = [c / max_pd_cost for c in pd_costs]

        # Apply normalization and scaling factors
        for idx, (u, v) in enumerate(self.G.edges()):
            travel_distance = normalized_travel_distances[idx]
            PD_cost = normalized_pd_costs[idx]
            
            # Combine the normalized travel cost and PD cost
            weight = K_D * travel_distance + K_PD * PD_cost
            
            # Assign the combined weight to the edge between u and v
            self.G[u][v]['weight'] = weight
            

    def cb_pathrequest(self, request):
        """
        Service callback to PathRequest.srv.
        The service receives origin and destination as Strings and returns the next waypoint as a String.
        """
        rospy.loginfo(f"Received path request from {request.origin} to {request.destination} | read_risk: {request.read_risk}")
        if not request.read_risk:
            queue = nx.astar_path(self.G, request.origin, request.destination, heuristic=self.heuristic, weight='weight')
        else:
            self.risk_map = read_risk_map()
            
            # Compute the maximum values
            max_travel, max_pd_cost = self.compute_max_values()
            
            # Use a partial function to fix the extra arguments
            causal_heuristic_predefined = partial(self.causal_heuristic, origin=request.origin, max_travel=max_travel, max_pd_cost=max_pd_cost)
            
            self.update_G_weights(max_travel, max_pd_cost)
            ros_utils.load_graph_to_rosparam(self.G, "/hrisim/G")
            self.graph_weight_update()
            
            queue = nx.astar_path(self.G, request.origin, request.destination, heuristic=causal_heuristic_predefined, weight='weight')
            
        self.graph_path_show(','.join(queue))
        return PathRequestResponse(','.join(queue))
    
        

if __name__ == "__main__":
    rospy.init_node('Astar_navigation')
    PRED_STEP = 5
    K_D = 1
    K_PD = 10
    
    Navigator(g_path = rospy.get_param('~graph_path'))
    
    rospy.spin()
