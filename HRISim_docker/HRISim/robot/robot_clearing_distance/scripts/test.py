import os
import cv2
import numpy as np
import yaml
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt

# Load the map metadata
yaml_path = '/home/lcastri/git/PeopleFlow/HRISim_docker/HRISim/hrisim_gazebo/tiago_maps/warehouse/map.yaml'
with open(yaml_path, 'r') as file:
    map_metadata = yaml.safe_load(file)

map_dir = os.path.dirname(yaml_path)
image_path = os.path.join(map_dir, map_metadata['image'])
resolution = map_metadata['resolution']
origin = map_metadata['origin']  # [x, y, theta]

# Load the map image
map_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# Convert the map to binary (1 = free space, 0 = obstacles)
obs_map = (map_image == 0).astype(np.uint8) 

# Compute the distance transform (distance to nearest obstacle in meters)
distance_transform = distance_transform_edt(1 - obs_map) * resolution

# Define robot position (in meters)
robot_positions = (-19.0, 0.0)
robot_x, robot_y = robot_positions

# Convert robot position to pixel coordinates
pixel_x = int((robot_x - origin[0]) / resolution)
pixel_y = int((-robot_y - origin[1] + 2) / resolution)

print(f"Robot position: ({robot_x}, {robot_y}) meters -> pixel position: ({pixel_x}, {pixel_y})")

# Check if the robot is within the map bounds
if 0 <= pixel_x < map_image.shape[1] and 0 <= pixel_y < map_image.shape[0]:
    print(f"Position is valid and within map bounds.")
else:
    print(f"Warning: Position is outside the map bounds.")

# Find the closest obstacle point using distance transform
obstacle_pixels = np.where(distance_transform == 0)  # Get obstacle locations
distances = np.sqrt((obstacle_pixels[1] - pixel_x) ** 2 + (obstacle_pixels[0] - pixel_y) ** 2)

# Get the closest obstacle index
closest_idx = np.argmin(distances)
closest_pixel_x = obstacle_pixels[1][closest_idx]
closest_pixel_y = obstacle_pixels[0][closest_idx]

# Convert back to world coordinates
closest_obstacle_x = closest_pixel_x * resolution + origin[0]
closest_obstacle_y = -closest_pixel_y * resolution + origin[1] - 2

print(f"Closest obstacle at: pixel ({closest_pixel_x}, {closest_pixel_y}) -> world ({closest_obstacle_x}, {closest_obstacle_y})")

# Plot the binary map with robot and closest obstacle
plt.figure(figsize=(10, 10))
plt.imshow(obs_map, cmap='gray')

# Plot the robot's position as a red cross
plt.scatter(pixel_x, pixel_y, color='red', label='Robot Position', s=100, marker='x')

# Plot the closest obstacle as a blue dot
plt.scatter(closest_pixel_x, closest_pixel_y, color='blue', label='Closest Obstacle', s=100, marker='o')

plt.title('Binary Map with Robot Position and Closest Obstacle')
plt.legend()
plt.show()
