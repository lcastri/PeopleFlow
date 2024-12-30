import os
import cv2
import numpy as np
import yaml
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt

yaml_path = '/home/lcastri/git/PeopleFlow/HRISim_docker/HRISim/hrisim_gazebo/tiago_maps/warehouse/map.yaml'
with open(yaml_path, 'r') as file:
    map_metadata = yaml.safe_load(file)
map_dir = os.path.dirname(yaml_path)
image_path = os.path.join(map_dir, map_metadata['image'])
resolution = map_metadata['resolution']
origin = map_metadata['origin']  # [x, y, theta]

# Load the map image
map_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# Convert the map to binary
obs_map = (map_image == 0).astype(np.uint8) 

# Plot the binary map
plt.figure(figsize=(10, 10))
plt.imshow(obs_map, cmap='gray')
plt.title('Binary Map of Obstacles')
plt.colorbar()

# Calculate the distance transform (distance to nearest obstacle)
distance_transform = distance_transform_edt(1 - obs_map) * resolution  # Convert to meters

# Visualize the distance transform
plt.figure(figsize=(10, 10))
plt.imshow(distance_transform, cmap='hot')
plt.title('Distance Transform')
plt.colorbar(label="Distance to Nearest Obstacle (m)")

# Test with a robot position in meters
robot_positions = (10.0, 10.0)
robot_x, robot_y = robot_positions
pixel_x = int((robot_x - origin[0]) / resolution)
pixel_y = int((-robot_y - origin[1] + 2) / resolution)

print(f"Robot position: ({robot_x}, {robot_y}) meters -> pixel position: ({pixel_x}, {pixel_y})")
# Check whether the robot is within the map bounds
if 0 <= pixel_x < map_image.shape[1] and 0 <= pixel_y < map_image.shape[0]:
    print(f"Position is valid and within map bounds.")
else:
    print(f"Warning: Position is outside the map bounds.")

# Plot the binary map with robot position
plt.figure(figsize=(10, 10))
plt.imshow(obs_map, cmap='gray')
# Plot the robot's position as a red dot
plt.scatter(pixel_x, pixel_y, color='red', label='Robot Position', s=100, marker='x')

plt.title('Binary Map with Robot Position')
plt.colorbar()
plt.legend()

plt.show()
