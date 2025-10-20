#!/usr/bin/env python

import os
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import yaml
import matplotlib.image as mpimg
from shapely.geometry import Point
from matplotlib.patches import Circle

# Initialize ROS node
SCENARIO = "/home/lcastri/git/PeopleFlow/HRISim_docker/pedsim_ros/pedsim_simulator/scenarios/warehouse"
MAP_NAME = os.path.expanduser('/home/lcastri/git/PeopleFlow/HRISim_docker/HRISim/hrisim_gazebo/tiago_maps/') + 'warehouse'
RES_DIR = "/home/lcastri/git/PeopleFlow/HRISim_docker/HRISim/peopleflow/peopleflow_manager/res"

# Create directory for results
scenario_name = SCENARIO.split("/")[-1]
res_dir = os.path.join(RES_DIR, scenario_name)
os.makedirs(res_dir, exist_ok=True)
tree = ET.parse(SCENARIO + '.xml')
root = tree.getroot()

# Extract waypoints
waypoints = []
for waypoint in root.findall('waypoint'):
    wp_id = waypoint.get('id')
    x = float(waypoint.get('x'))
    y = float(waypoint.get('y'))
    r = float(waypoint.get('r'))
    waypoints.append((wp_id, x, y, r))

# Load map information
with open(os.path.join(MAP_NAME, 'map.yaml'), 'r') as yaml_file:
    map_info = yaml.safe_load(yaml_file)

# Load map image
map_image = mpimg.imread(os.path.join(MAP_NAME, 'map.pgm'))

# Get resolution and origin from YAML
resolution = map_info['resolution']
origin_x, origin_y = map_info['origin'][:2]

# Visualize the map and waypoints without edges
fig, ax = plt.subplots(figsize=(12, 8))
ax.set_xlim(-20, 11)  # Replace with your desired x-axis limits
ax.set_ylim(-11, 11)  # Replace with your desired y-axis limits

# Plot map image
ax.imshow(map_image, extent=(origin_x, origin_x + map_image.shape[1] * resolution, 
                             origin_y, origin_y + map_image.shape[0] * resolution), cmap='gray')

# Plot waypoints (without edges)
for wp in waypoints:
    # Plot a circle at the waypoint position
    circle_patch = Circle((wp[1], wp[2]), radius=wp[3], alpha=0.7, color='b')
    ax.add_patch(circle_patch)

# Customize plot
ax.set_title('Map with Waypoints')
ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')
plt.grid(True)

# Save the plot
plt.savefig(os.path.join(res_dir, "waypoints_on_map.png"))
plt.show()
