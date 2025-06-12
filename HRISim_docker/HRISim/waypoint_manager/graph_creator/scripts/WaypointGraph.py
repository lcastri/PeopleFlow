#!/usr/bin/env python

import os
import xml.etree.ElementTree as ET
import networkx as nx
import matplotlib.pyplot as plt
import yaml
import matplotlib.image as mpimg
from shapely.geometry import Point, LineString
import pickle
import rospy   
from matplotlib.patches import Circle
import math

def line_of_sight(ax, wp1, wp2, obstacles, waypoints, doors):
    """
    Function to check if the line between two points intersects any obstacles

    Args:
        ax: plot's axes
        wp1 (tuple): waypoint 1
        wp2 (tuple): waypoint 2
        obstacles (list): obstacles list

    Returns:
        (bool, LineString/None): (False, None) if obstacles hinders the connection between wp1 and wp2. Otherwise, (True, Linestring)
    """
    line = LineString([wp1, wp2])
    
    for obstacle in obstacles:
        # obs_plot = ax.plot(obstacle[0], obstacle[1], 'mo', markersize=5)
        # print(line.distance(Point(obstacle)))
        if line.distance(Point(obstacle)) < 0.1:  # Adjust tolerance level as needed
            rospy.logdebug(f"\tnot connected: obstacle")
            return False, None
        # obs_plot.pop(0).remove()
        
    # Check for intersection with other waypoints
    for wp in waypoints:
        waypoint_point = Point(wp[1], wp[2])
        waypoint_radius = wp[3]
        waypoint_circle = waypoint_point.buffer(waypoint_radius)
        if line.intersects(waypoint_circle):
            rospy.logdebug(f"\tnot connected: WP {wp[0]}")
            return False, None

    for door in doors:
        door_point = Point(door[1], door[2])
        if door_point == Point(wp1[0], wp1[1]) or door_point == Point(wp2[0], wp2[1]): continue
        distance_to_door = line.distance(door_point)
        if distance_to_door < 0.75:
            rospy.logdebug(f"\tnot connected: door {door}")
            return False, None
    rospy.logdebug("\tconnected")
    return True, line


# Load map metadata
# SCENARIO = "/home/lcastri/git/ROS-Causal_HRISim/HRISim_docker/pedsim_ros/pedsim_simulator/scenarios/warehouse"
# MAP_NAME = os.path.expanduser('/home/lcastri/git/ROS-Causal_HRISim/HRISim_docker/HRISim/hrisim_gazebo/tiago_maps/') + 'warehouse'
# RES_DIR = "/home/lcastri/git/ROS-Causal_HRISim/HRISim_docker/HRISim/peopleflow/peopleflow_manager/res"
# Parse the XML content
rospy.init_node("waypointgraph")
MAP_NAME = str(rospy.get_param("~map"))
SCENARIO = str(rospy.get_param("~scenario"))
RES_DIR = str(rospy.get_param("~res_dir"))

# Create directory for results
scenario_name = SCENARIO.split("/")[-1]
res_dir = os.path.join(RES_DIR, scenario_name)
os.makedirs(res_dir, exist_ok=True)
tree = ET.parse(SCENARIO + '.xml')
root = tree.getroot()

# f = open(os.path.join(res_dir, 'waypoint_pairs.txt'), 'w')

# Extract waypoints
waypoints = []
doors = []
for waypoint in root.findall('waypoint'):
    wp_id = waypoint.get('id')
    x = float(waypoint.get('x'))
    y = float(waypoint.get('y'))
    r = float(waypoint.get('r'))
    waypoints.append((wp_id, x, y, r))
    if wp_id.startswith('door-'): doors.append((wp_id, x, y, r))
    
with open(os.path.join(MAP_NAME, 'map.yaml'), 'r') as yaml_file:
    map_info = yaml.safe_load(yaml_file)

# Load map image
map_image = mpimg.imread(os.path.join(MAP_NAME, 'map.pgm'))

# Get resolution and origin from YAML
resolution = map_info['resolution']
origin_x, origin_y = map_info['origin'][:2]

# Extract obstacles
obstacle_threshold = 10
obstacle_mask = map_image < obstacle_threshold

# Calculate real-world coordinates of obstacles
obstacle_coordinates = []
for y in range(map_image.shape[0]):
    for x in range(map_image.shape[1]):
        if obstacle_mask[y, x]:  # Check if obstacle
            x_real = origin_x + x * resolution
            y_real = origin_y + (map_image.shape[0] - y - 1) * resolution
            obstacle_coordinates.append((x_real, y_real))
            
# Visualize obstacles and line of sight
fig, ax = plt.subplots(figsize=(12, 8))
ax.set_xlim(-20, 11)  # Replace with your desired x-axis limits
ax.set_ylim(-11, 11)  # Replace with your desired y-axis limits
# Plot map image
ax.imshow(map_image, extent=(origin_x, origin_x + map_image.shape[1] * resolution, 
                             origin_y, origin_y + map_image.shape[0] * resolution), cmap='gray')
# plt.ion() 
# plt.show()
x_coords, y_coords = zip(*obstacle_coordinates)
# ax.scatter(x_coords, y_coords, s=1, alpha=0.5)  # Adjust size and transparency as needed
    
# Create a graph
G = nx.Graph()

# Plot waypoints and add nodes with positions
for wp in waypoints:
    # ax.plot(wp[1], wp[2], 'bo', markersize=10*wp[3])
    circle_patch = Circle((wp[1], wp[2]), radius=wp[3], alpha=0.7, color='b')
    ax.add_patch(circle_patch)
    G.add_node(wp[0], pos=(wp[1], wp[2]), radius=wp[3])
    
# Check line of sight between waypoints
for i, wp1 in enumerate(waypoints):
    for j, wp2 in enumerate(waypoints):
        if i < j:
            other_waypoints = waypoints[:i] + waypoints[i+1:j] + waypoints[j+1:]  # Exclude wp1 and wp2

            # wp1_plot = ax.plot(wp1[1], wp1[2], 'ro', markersize=20)
            # wp2_plot = ax.plot(wp2[1], wp2[2], 'ro', markersize=20)
            rospy.logdebug(f"WPs {wp1[0]}-{wp2[0]}")
            line_of_sight_exists, line = line_of_sight(ax, (wp1[1], wp1[2]), (wp2[1], wp2[2]), obstacle_coordinates, other_waypoints, doors)
            if line_of_sight_exists:
                x_values, y_values = line.xy
                ax.plot(x_values, y_values, 'g-', linewidth=2)
                G.add_edge(wp1[0], wp2[0], weight=math.sqrt((wp1[1]-wp2[1])**2 + (wp1[2]-wp2[2])**2))
            # wp1_plot.pop(0).remove()  # Remove wp1 point
            # wp2_plot.pop(0).remove()  # Remove wp2 point

# Customize plot
ax.set_title('Map with Obstacles and Line of Sight')
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
plt.grid(True)
plt.savefig(os.path.join(res_dir, "waypoint_with_map.png"))
# plt.show()


# Visualize the graph
pos = nx.get_node_attributes(G, 'pos')
radius = nx.get_node_attributes(G, 'radius')
edge_weights = nx.get_edge_attributes(G, 'weight')
edge_weights = {edge: round(weight, 2) for edge, weight in edge_weights.items()}

# Scale radius for better visualization
node_sizes = [radius[node] * 500 for node in G.nodes()]  # Adjust scaling factor as needed
plt.figure(figsize=(12, 8))
nx.draw(G, pos, with_labels=True, node_size=node_sizes, node_color='skyblue', font_size=10, font_weight='bold')
nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_weights)

plt.title("Waypoints Graph")
plt.savefig(os.path.join(res_dir, "waypoint_graph.png"))
# plt.show()

# Save the graph as a pickle file
with open(os.path.join(res_dir, "graph.pkl"), "wb") as f:
    pickle.dump(G, f)