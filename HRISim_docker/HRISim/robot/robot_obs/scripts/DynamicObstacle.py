#!/usr/bin/env python

import rospy
import os
import math
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse
from move_base_msgs.msg import MoveBaseActionGoal

class DynamicObstacleSpawner:
    def __init__(self):
        rospy.init_node("dynamic_obstacle_node")
        self.obs_pub = rospy.Publisher("/hrisim/robot_obs", Bool, queue_size=10)

        # Wait for necessary Gazebo services
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/delete_model")
        rospy.wait_for_service("/gazebo/get_model_state")

        # Setup Gazebo services
        self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        # ROS Service to trigger spawning & removing
        self.spawn_service = rospy.Service("/hrisim/obstacles/spawn", Empty, self.handle_spawn_obstacle)
        self.remove_service = rospy.Service("/hrisim/obstacles/remove", Empty, self.handle_remove_obstacle)
        self.timer_service = rospy.Service("/hrisim/obstacles/timer/off", Empty, self.handle_timer_off)

        # Subscribe to the goal topic to track next goal
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.cb_robot_goal)
        self.next_goal = None  # Store the next goal position

        # Box SDF model path
        self.box_model_path = os.path.expanduser("/root/.gazebo/models/mycube/mycube.sdf")

        # Keep track of the last spawned box name
        self.last_spawned_box = None

    def cb_robot_goal(self, goal: MoveBaseActionGoal):            
        self.next_goal = (goal.goal.target_pose.pose.position.x, goal.goal.target_pose.pose.position.y)
        rospy.loginfo(f"Received new goal: {self.next_goal}")

    def get_robot_position(self):
        """Gets the robot's current position from Gazebo"""
        try:
            model_state = self.get_model_state("tiago", "world")  # Adjust "tiago" to match your robot's name
            x = model_state.pose.position.x
            y = model_state.pose.position.y
            return x, y
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to get robot position from Gazebo: {e}")
            return None

    def compute_spawn_position(self, robot_x, robot_y):
        """Computes the position 2 meters in front of the robot towards its goal"""
        if self.next_goal is None:
            rospy.logwarn("No goal received yet. Cannot compute spawn position.")
            return None

        goal_x, goal_y = self.next_goal

        # Compute direction vector
        dx = goal_x - robot_x
        dy = goal_y - robot_y
        distance = math.sqrt(dx**2 + dy**2)

        if distance == 0:
            rospy.logwarn("Robot and goal are at the same position. Cannot compute direction.")
            return None

        # Normalize the direction vector
        dx /= distance
        dy /= distance

        # Compute new position 2 meters ahead in the goal direction
        spawn_x = robot_x + 1 * dx
        spawn_y = robot_y + 1 * dy

        return spawn_x, spawn_y

    def spawn_obstacle(self):
        """Spawns a single box in front of the robot in the direction of its goal"""
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            rospy.logwarn("Could not retrieve robot position.")
            return

        robot_x, robot_y = robot_pos
        spawn_position = self.compute_spawn_position(robot_x, robot_y)

        if spawn_position is None:
            rospy.logwarn("Could not compute spawn position.")
            return

        spawn_x, spawn_y = spawn_position

        # Define Pose
        box_pose = Pose()
        box_pose.position.x = spawn_x
        box_pose.position.y = spawn_y
        box_pose.position.z = 0.5  # Elevation to prevent sinking

        # Read model SDF file
        with open(self.box_model_path, "r") as f:
            model_xml = f.read()

        # Unique box name
        box_name = "dynamic_obstacle_box"

        try:
            self.spawn_srv(box_name, model_xml, "", box_pose, "world")
            rospy.loginfo(f"Spawned obstacle at ({spawn_x}, {spawn_y})")
            self.last_spawned_box = box_name
            
            self.obs_pub.publish(True)
            rospy.set_param('/hrisim/robot_obs', True)
            
            # Start timer to remove obstacle if not cleared
            self.timer = rospy.Timer(rospy.Duration(30), self.remove_obstacle_if_still_there)
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn failed: {e}")

    def remove_obstacle_if_still_there(self, event):
        """Removes the last spawned obstacle if still there after 30 seconds"""
        rospy.logwarn("Checking if obstacle is still there...")
        if rospy.get_param('/hrisim/robot_obs', False):
            rospy.logwarn("Obstacle is still there. Removing...")
            self.remove_obstacle()
            self.timer.shutdown()

    def remove_obstacle(self):
        """Removes the last spawned obstacle"""
        if self.last_spawned_box is None:
            rospy.logwarn("No obstacle to remove.")
            return

        try:
            self.delete_srv(self.last_spawned_box)
            rospy.loginfo(f"Removed obstacle: {self.last_spawned_box}")
            self.last_spawned_box = None
            
            self.obs_pub.publish(False)
            rospy.set_param('/hrisim/robot_obs', False)
        except rospy.ServiceException as e:
            rospy.logerr(f"Delete failed: {e}")

    # ROS Service Callbacks
    def handle_spawn_obstacle(self, req):
        """Service callback to spawn an obstacle"""
        self.spawn_obstacle()
        return EmptyResponse()

    def handle_remove_obstacle(self, req):
        """Service callback to remove the obstacle"""
        self.remove_obstacle()
        return EmptyResponse()
    
    def handle_timer_off(self, req):
        """Service callback to turn off the timer"""
        self.timer.shutdown()
        rospy.logwarn("Timer turned off.")
        return EmptyResponse()


if __name__ == "__main__":
    obstacle_manager = DynamicObstacleSpawner()
    rospy.spin()
