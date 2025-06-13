#!/usr/bin/env python

import pickle
import rospy
import hrisim_util.ros_utils as ros_utils

if __name__ == "__main__":
    try:
        rospy.init_node('graph_publisher')

        graph_path = rospy.get_param("~graph_file")

        rospy.loginfo(f"Reading graph from: {graph_path}")
        g_path = str(rospy.get_param("~g_path"))
        with open(g_path, 'rb') as f:
            G = pickle.load(f)
            ros_utils.load_graph_to_rosparam(G, "/hrisim/G")
            rospy.loginfo(f"Published graph to /hrisim/G")
    except rospy.ROSInterruptException:
        pass
    