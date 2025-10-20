#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from robot_srvs.srv import VisualisePath, VisualisePathResponse
import hrisim_util.ros_utils as ros_utils

# Global variable to store the latest A* path
latest_astar_path = []

def update_astar_path(request):
    """
    Service callback to update the A* path.
    The service receives a comma-separated list of waypoints as a String.
    """
    global latest_astar_path

    # Extract path from request
    latest_astar_path = request.path.split(",") if request.path else []

    # rospy.loginfo(f"Updated A* path: {latest_astar_path}")

    # Immediately update visualization
    ros_graph = ros_utils.wait_for_param('/peopleflow/G')

    # Extract node positions and edges
    node_positions = {node: (data['pos'][0], data['pos'][1]) for node, data in ros_graph['nodes'].items()}
    edge_data = ros_graph['edges']

    # Create MarkerArray for visualization
    markers = MarkerArray()

    # Add edges to visualization
    for i, edge in enumerate(edge_data):
        edge_marker = Marker()
        edge_marker.header.frame_id = "map"
        edge_marker.type = Marker.LINE_STRIP
        edge_marker.id = i
        edge_marker.scale.x = 0.02  # Default thickness

        # Default color: Grey (all edges are grey unless they are in the A* path)
        edge_marker.color.r = 0.5
        edge_marker.color.g = 0.5
        edge_marker.color.b = 0.5
        edge_marker.color.a = 1.0  # Fully opaque

        # Check if this edge is part of the latest A* path
        if latest_astar_path and (
            (edge['source'], edge['target']) in zip(latest_astar_path, latest_astar_path[1:]) or
            (edge['target'], edge['source']) in zip(latest_astar_path, latest_astar_path[1:])
        ):
            edge_marker.color.r = 0.0  # Remove red
            edge_marker.color.g = 0.0  # Remove green
            edge_marker.color.b = 1.0  # Highlight the path in blue
            edge_marker.scale.x = 0.2  # Make path edges thicker

        # Set edge positions
        edge_marker.points = [
            Point(node_positions[edge['source']][0], node_positions[edge['source']][1], 0),
            Point(node_positions[edge['target']][0], node_positions[edge['target']][1], 0)
        ]

        markers.markers.append(edge_marker)

    # If there is a valid path, add a green sphere at the final goal
    if latest_astar_path:
        final_goal = latest_astar_path[-1]  # Get the last waypoint
        if final_goal in node_positions:
            goal_marker = Marker()
            goal_marker.header.frame_id = "map"
            goal_marker.type = Marker.SPHERE
            goal_marker.id = len(edge_data) + 1  # Unique ID
            goal_marker.scale.x = 1  # Sphere size
            goal_marker.scale.y = 1
            goal_marker.scale.z = 1
            goal_marker.color.r = 0.0
            goal_marker.color.g = 1.0  # Green color
            goal_marker.color.b = 0.0
            goal_marker.color.a = 1.0  # Fully opaque

            # Position the sphere at the final goal
            goal_marker.pose.position.x = node_positions[final_goal][0]
            goal_marker.pose.position.y = node_positions[final_goal][1]
            goal_marker.pose.position.z = 0.5  # Slightly above ground

            markers.markers.append(goal_marker)

    # Publish visualization
    marker_pub.publish(markers)
    # rospy.loginfo("Graph visualization updated with A* path and goal marker")
    
    return VisualisePathResponse()


if __name__ == "__main__":
    rospy.init_node('astar_path_visualization_service')

    # Publisher for the visualization markers
    marker_pub = rospy.Publisher('/astar_path_visualization', MarkerArray, queue_size=10)

    # Service to trigger A* path visualization
    rospy.Service('/graph/path/show', VisualisePath, update_astar_path)
    
    rospy.loginfo("A* Path visualization service started")
    rospy.spin()
