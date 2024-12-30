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

class ClearingDistanceNode:
    def __init__(self):
        rospy.init_node("clearance_distance_node")

        # Clearance publisher
        self.clearance_pub = rospy.Publisher("/hrisim/robot_clearing_distance", Float32, queue_size=10)

        # Load map data
        self.map_path = rospy.get_param("~map_path", "/root/ros_ws/src/HRISim/hrisim_gazebo/tiago_maps/warehouse/map.yaml")
        self.map_data, self.map_resolution, self.map_origin, self.distance_transform = self.load_map(self.map_path)

        # Subscriber for robot position (x, y)
        self.robot_pos_sub = rospy.Subscriber("/robot_pose", PoseWithCovarianceStamped, self.robot_position_callback)
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
            rospy.logerr(f"image_path: {image_path}")
            if map_image is None:
                rospy.logerr(f"Failed to load map image from {image_path}")
                rospy.signal_shutdown("Could not load map image")
                return None, None, None
            else:
                rospy.logerr(f"Loaded map image, shape: {map_image.shape}, dtype: {map_image.dtype}")

            # Convert the map to binary
            binary_map = (map_image == 0).astype(np.uint8) 

            # Precompute the distance transform
            distance_transform = distance_transform_edt(1 - binary_map) * resolution  # 1-free space, 0-obstacles
            return binary_map, resolution, origin, distance_transform

        except Exception as e:
            rospy.logerr(f"Error loading map: {e}")
            rospy.signal_shutdown("Could not load map")
            return None, None, None, None

    def robot_position_callback(self, pose):
        """Calculates clearance distance using the robot position."""
        if self.map_data is None:
            rospy.logwarn("Map data not available.")
            return
        x, y, yaw = ros_utils.getPose(pose.pose.pose)
        self.robot_position = (x, y)
        clearance = self.calculate_clearance()
        self.clearance_pub.publish(clearance)

    def calculate_clearance(self):
        """Calculates the minimum distance between the robot and the nearest obstacle."""
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
        # rospy.loginfo(f"Clearance distance: {clearance:.2f} meters")
        return clearance

if __name__ == "__main__":
    try:
        node = ClearingDistanceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
