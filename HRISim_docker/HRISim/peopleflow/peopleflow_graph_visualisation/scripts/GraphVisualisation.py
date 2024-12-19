#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger, TriggerResponse
import hrisim_util.ros_utils as ros_utils


def visualize_graph_from_rosparam(request):
    # Load graph from ROS parameter
    ros_graph = ros_utils.wait_for_param('/peopleflow/G')
    
    # Extract data from ROS graph dictionary
    node_positions = {node: (data['pos'][0], data['pos'][1]) for node, data in ros_graph['nodes'].items()}
    edge_data = ros_graph['edges']

    # Get weights for normalization
    weights = [edge['weight'] for edge in edge_data]
    min_weight, max_weight = min(weights), max(weights)
    weight_range = max_weight - min_weight if max_weight != min_weight else 1  # Avoid divide-by-zero

    # Create MarkerArray for visualization
    markers = MarkerArray()
    
    # Add edges
    for i, edge in enumerate(edge_data):
        edge_marker = Marker()
        edge_marker.header.frame_id = "map"
        edge_marker.type = Marker.LINE_STRIP
        edge_marker.id = i
        edge_marker.scale.x = 0.02
        
        # Normalize weight and determine color
        normalized_weight = (edge['weight'] - min_weight) / weight_range
        edge_marker.color.r = normalized_weight  # Red increases with weight
        edge_marker.color.g = 1.0 - normalized_weight  # Green decreases with weight
        edge_marker.color.b = 0.0  # Blue remains constant
        edge_marker.color.a = 1.0

        edge_marker.points = [
            Point(node_positions[edge['source']][0], node_positions[edge['source']][1], 0),
            Point(node_positions[edge['target']][0], node_positions[edge['target']][1], 0)
        ]

        markers.markers.append(edge_marker)
        
        # Text marker for the weight
        weight_marker = Marker()
        weight_marker.header.frame_id = "map"
        weight_marker.type = Marker.TEXT_VIEW_FACING
        weight_marker.id = len(edge_data) + i  # Ensure unique IDs for text markers
        weight_marker.scale.z = 0.1  # Size of the text
        weight_marker.color.r = 1.0
        weight_marker.color.g = 1.0
        weight_marker.color.b = 1.0
        weight_marker.color.a = 1.0  # Fully opaque
        
        # Position of the text (midpoint of the edge)
        x_mid = (node_positions[edge['source']][0] + node_positions[edge['target']][0]) / 2
        y_mid = (node_positions[edge['source']][1] + node_positions[edge['target']][1]) / 2
        weight_marker.pose.position.x = x_mid
        weight_marker.pose.position.y = y_mid
        weight_marker.pose.position.z = 0.1  # Slightly above the plane
        
        # Text content
        weight_marker.text = f"{normalized_weight:.2f}"  # Format weight to two decimal places
        markers.markers.append(weight_marker)

    
    marker_pub.publish(markers)
    rospy.loginfo("Graph visualization updated")
    return TriggerResponse(success=True, message="Graph visualization updated successfully.")



if __name__ == "__main__":
    rospy.init_node('graph_visualization_service')
    marker_pub = rospy.Publisher('/graph_visualization', MarkerArray, queue_size=10)

    rospy.Service('update_graph_visualization', Trigger, visualize_graph_from_rosparam)
    rospy.loginfo("Graph visualization service started")
    rospy.spin()
