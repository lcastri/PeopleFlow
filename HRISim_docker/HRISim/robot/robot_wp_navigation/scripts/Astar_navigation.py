#!/usr/bin/env python

import pickle
import rospy
import networkx as nx
from robot_srvs.srv import PathRequest, PathRequestResponse
import hrisim_util.ros_utils as ros_utils
from robot_srvs.srv import VisualisePath

class Navigator():
    def __init__(self, g_path) -> None:
        with open(g_path, 'rb') as f:
            self.G = pickle.load(f)
            
            ros_utils.wait_for_service('/graph/path/show')
            self.graph_path_show = rospy.ServiceProxy('/graph/path/show', VisualisePath)
            rospy.Service('/hrisim/path', PathRequest, self.cb_pathrequest)

            
    def heuristic(self, a, b):
        pos = nx.get_node_attributes(self.G, 'pos')
        (x1, y1) = pos[a]
        (x2, y2) = pos[b]
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


    def cb_pathrequest(self, request):
        """
        Service callback to PathRequest.srv.
        The service receives origin and destination as Strings and returns the next waypoint as a String.
        """
        queue = nx.astar_path(self.G, request.origin, request.destination, heuristic=self.heuristic, weight='weight')
        rospy.logerr(queue)
        self.graph_path_show(','.join(queue))
        
        return PathRequestResponse(','.join(queue))
    
        

if __name__ == "__main__":
    rospy.init_node('Astar_navigation')

    Navigator(g_path = rospy.get_param('~graph_path'))
    
    rospy.spin()
