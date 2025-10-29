#!/usr/bin/env python

import os
import numpy as np
import yaml
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import hrisim_util.ros_utils as ros_utils
from std_msgs.msg import Float32
import cv2
from scipy.ndimage import distance_transform_edt
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import GetWorldProperties, GetModelState

class ClearingDistanceNode:
    def __init__(self):
        # Clearance publisher
        self.clearance_pub = rospy.Publisher("/hrisim/robot_clearing_distance", Float32, queue_size=10)
        self.robot_marker_pub = rospy.Publisher("/robot_position_marker", Marker, queue_size=10)  
        self.closest_obs_marker_pub = rospy.Publisher("/closest_obstacle_marker", Marker, queue_size=10)

        # Load map data
        self.map_path = rospy.get_param("~map_path", "/root/ros_ws/src/HRISim/hrisim_gazebo/tiago_maps/warehouse/map.yaml")
        self.map_data, self.map_resolution, self.map_origin, self.distance_transform = self.load_map(self.map_path)

        # Gazebo services
        rospy.wait_for_service('/gazebo/get_world_properties')
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.robot_position = None

    def load_map(self, yaml_path):
        """Loads the map and precomputes the distance transform for efficient clearance calculation."""
        rospy.loginfo(f"Loading map from {yaml_path}")
        
        try:
            with open(yaml_path, 'r') as file:
                map_metadata = yaml.safe_load(file)

            # Resolve full path to the image
            map_dir = os.path.dirname(yaml_path)
            image_path = os.path.join(map_dir, map_metadata['image'])
            resolution = map_metadata['resolution']
            origin = map_metadata['origin']  # [x, y, theta]

            # Load the map image
            map_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            if map_image is None:
                rospy.logerr(f"Failed to load map image from {image_path}")
                rospy.signal_shutdown("Could not load map image")
                return None, None, None
            else:
                rospy.loginfo(f"Loaded map image, shape: {map_image.shape}, dtype: {map_image.dtype}")

            # Convert the map to binary
            binary_map = (map_image == 0).astype(np.uint8) 

            # Precompute the distance transform
            distance_transform = distance_transform_edt(1 - binary_map) * resolution  # 1-free space, 0-obstacles
            return binary_map, resolution, origin, distance_transform

        except Exception as e:
            rospy.logerr(f"Error loading map: {e}")
            rospy.signal_shutdown("Could not load map")
            return None, None, None, None

    def get_robot_position(self):
        """Retrieves the robot's position from Gazebo instead of localization."""
        try:
            model_state = self.get_model_state("tiago", "world")  
            x, y = model_state.pose.position.x, model_state.pose.position.y
            self.robot_position = (x, y)
            self.publish_robot_marker(x, y)  
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to get robot position from Gazebo: {e}")
            self.robot_position = None

    def publish_robot_marker(self, x, y):
        """Publishes a purple circle at the robot's position in RViz."""
        marker = Marker()
        marker.header.frame_id = "map"  
        marker.header.stamp = rospy.Time.now()
        marker.ns = "robot_position"
        marker.id = 0
        marker.type = Marker.SPHERE  
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = x+0.5
        marker.pose.position.y = y
        marker.pose.position.z = 0.0  
        marker.pose.orientation.w = 1.0

        # Set the scale (radius)
        marker.scale.x = ROBOT_RADIUS * 2  
        marker.scale.y = ROBOT_RADIUS * 2  
        marker.scale.z = 0.1  

        # Set color (purple)
        marker.color.r = 0.5
        marker.color.g = 0.0
        marker.color.b = 0.5
        marker.color.a = 1.0  

        self.robot_marker_pub.publish(marker)

    def calculate_clearance(self):
        """Calculates the minimum distance between the robot and the nearest obstacle."""
        self.get_robot_position()
        if self.robot_position is None:
            return float('inf')

        robot_x, robot_y = self.robot_position

        # Convert robot position to map pixels
        pixel_x = int((robot_x - self.map_origin[0]) / self.map_resolution)
        pixel_y = int((-robot_y - self.map_origin[1] + 2) / self.map_resolution)

        # Validate position within bounds
        if pixel_x < 0 or pixel_y < 0 or pixel_x >= self.map_data.shape[1] or pixel_y >= self.map_data.shape[0]:
            rospy.logwarn("Robot is outside of the map boundaries.")
            return float('inf')

        # Lookup precomputed clearance from distance transform
        clearance = self.distance_transform[pixel_y, pixel_x]

        # Find closest obstacle point
        closest_x, closest_y = self.find_closest_obstacle(pixel_x, pixel_y)
        self.publish_obstacle_marker(closest_x, closest_y)
        self.clearance_pub.publish(clearance)
        return clearance

    def find_closest_obstacle(self, pixel_x, pixel_y):
        """Finds the pixel location of the closest obstacle."""
        obstacle_pixels = np.where(self.distance_transform == 0)
        closest_idx = np.argmin(np.sqrt((obstacle_pixels[1] - pixel_x) ** 2 + (obstacle_pixels[0] - pixel_y) ** 2))
        closest_pixel_x = obstacle_pixels[1][closest_idx]
        closest_pixel_y = obstacle_pixels[0][closest_idx]

        # Convert back to world coordinates
        # rospy.logerr(f"pixelX: {closest_pixel_x} -- pixelY: {closest_pixel_y}")

        closest_obstacle_x = closest_pixel_x * self.map_resolution + self.map_origin[0]
        closest_obstacle_y = (self.map_origin[1] + (self.map_data.shape[0] - closest_pixel_y) * self.map_resolution)

        return closest_obstacle_x, closest_obstacle_y

    def publish_obstacle_marker(self, x, y):
        """Publishes a red marker at the closest obstacle position in RViz."""
        marker = Marker()
        marker.header.frame_id = "map"  
        marker.header.stamp = rospy.Time.now()
        marker.ns = "closest_obstacle"
        marker.id = 1
        marker.type = Marker.SPHERE  
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0  
        marker.pose.orientation.w = 1.0

        # Set the scale (small dot)
        marker.scale.x = 0.5  
        marker.scale.y = 0.5  
        marker.scale.z = 0.5  

        # Set color (red)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0  

        self.closest_obs_marker_pub.publish(marker)

if __name__ == "__main__":
    rospy.init_node("clearance_distance_node")
    rate = rospy.Rate(10)
    ROBOT_RADIUS = 0.27
    node = ClearingDistanceNode()
    while not rospy.is_shutdown():
        node.calculate_clearance()
        rate.sleep()