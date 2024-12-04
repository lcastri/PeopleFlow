import os
import sys

import numpy as np

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from AbstractAction import AbstractAction
from prediction_srvs.srv import GetRiskMap, GetRiskMapResponse

class predict(AbstractAction):

    def _start_action(self):
        
        # Wait for the service to be available
        rospy.wait_for_service('/get_risk_map')
        
        try:
            # Create a proxy for the service
            get_risk_map = rospy.ServiceProxy('/get_risk_map', GetRiskMap)
            
            # Call the service
            response = get_risk_map()
            
            # Unpack the service response
            waypoint_ids = response.waypoint_ids
            n_steps = response.n_steps
            n_waypoints = response.n_waypoint
            flattened_PDs = response.PDs
            flattened_BACs = response.BACs
            
            # Reconstruct 2D arrays for PDs and BACs
            PDs_matrix = np.array(flattened_PDs).reshape(n_waypoints, n_steps)
            BACs_matrix = np.array(flattened_BACs).reshape(n_waypoints, n_steps)
            
            # Combine waypoint IDs and risk values into a dictionary
            risk_map = {}
            for i, wp in enumerate(waypoint_ids):
                risk_map[wp] = {
                    'PD': PDs_matrix[i].tolist(),
                    'BAC': BACs_matrix[i].tolist()
                }
            
            rospy.loginfo(f"Received risk map: {risk_map}")
            
            self.params.append("done")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            
            self.params.append("interrupted")
        
    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0:
            if params[-1] == "done" or params[-1] == "interrupted":
                reached = True
        return reached
