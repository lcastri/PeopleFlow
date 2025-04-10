#!/usr/bin/env python


import os
import numpy as np
import yaml
import rospy
import math
import cv2
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from scipy.ndimage import distance_transform_edt

class ObstacleNode:
    def __init__(self):
        rospy.init_node("is_there_obs")

        # Publishers
        self.obs_pub = rospy.Publisher("/hrisim/robot_obs", Bool, queue_size=10)

        # Gazebo services
        rospy.wait_for_service('/gazebo/get_world_properties')
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.robot_position = None
        self.dynobs_position = None
        self.clearance = None

        rospy.Subscriber('/hrisim/robot_clearing_distance', Float32, self.cb_robot_clearing_distance)
        
    def cb_robot_clearing_distance(self, msg: Float32):
        self.clearance = msg.data

    def get_robot_position(self):
        """Retrieves the robot's position from Gazebo instead of localization."""
        try:
            model_state = self.get_model_state("tiago", "world")  
            x, y = model_state.pose.position.x, model_state.pose.position.y
            self.robot_position = (x, y)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to get robot position from Gazebo: {e}")
            self.robot_position = None
            
            
    def get_dynobs_position(self):
        try:
            model_state = self.get_model_state("dynamic_obstacle_box", "world")  
            x, y = model_state.pose.position.x, model_state.pose.position.y
            self.dynobs_position = (x, y)
        except rospy.ServiceException as e:
            self.dynobs_position = None


    def check_obstacle_clearance(self):
        """Checks if the robot is too close to any obstacle and publishes the result."""        
        if self.clearance is None: return
        distances = [self.clearance]

        self.get_robot_position()
        self.get_dynobs_position()
        if self.robot_position is not None and self.dynobs_position is not None:
            dist_dynobs = math.sqrt((self.robot_position[0] - self.dynobs_position[0])**2 + (self.robot_position[1] - self.dynobs_position[1])**2)
            distances.append(dist_dynobs)
            
        # Determine if the robot is too close to any obstacle
        OBS = any([d < (ROBOT_INFLATION + ROBOT_RADIUS) for d in distances])

        # Publish obstacle presence
        self.obs_pub.publish(OBS)
        rospy.set_param('/hrisim/robot_obs', OBS)


if __name__ == "__main__":
    try:
        ROBOT_RADIUS = 0.27  
        ROBOT_INFLATION = rospy.get_param("/move_base/local_costmap/inflation_layer/inflation_radius", 0.55)
        node = ObstacleNode()

        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            node.check_obstacle_clearance()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
