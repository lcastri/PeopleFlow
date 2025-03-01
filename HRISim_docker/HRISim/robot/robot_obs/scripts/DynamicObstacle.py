#!/usr/bin/env python

import rospy
import os
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty, EmptyResponse

class DynamicObstacles:
    def __init__(self):
        rospy.init_node("dynamic_obstacle_node")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/delete_model")
        
        # Setup ROS services
        self.spawn_service = rospy.Service("/hrisim/obstacles/spawn", Empty, self.handle_spawn_obstacles)
        self.remove_service = rospy.Service("/hrisim/obstacles/remove", Empty, self.handle_remove_obstacles)

        self.spawn_srv = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_srv = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

        # Waypoints where we will place the obstacles
        self.waypoints = {
            "wa-2-l": (-18.2, 4),
            "wa-2-c": (-14, 4),
            "wa-2-r": (-10, 4),
            "wa-4-l": (-18.2, -3.5),
            "wa-4-c": (-14, -3.5),
            "wa-4-r": (-10, -3.5)
        }
        self.box_model_path = os.path.expanduser("/root/.gazebo/models/mycube/mycube.sdf")
        
        
    def spawn_obstacles(self):
        """Spawn two boxes (left & right) for each waypoint."""
        with open(self.box_model_path, "r") as f:
            model_xml = f.read()

        # Offset values (adjust based on robot width)
        offset = 0.6  # Distance from the waypoint center
        boxwidth = 1  # Distance from the waypoint center
        for i, (wp, (x, y)) in enumerate(self.waypoints.items()):
            left_pose = Pose()
            right_pose = Pose()


            # Left Box (on the negative side of y-axis)
            left_pose.position.x = x - (boxwidth/2 + offset)
            left_pose.position.y = y
            left_pose.position.z = 0.5  # Elevation to prevent sinking

            # Right Box (on the positive side of y-axis)
            right_pose.position.x = x + (boxwidth/2 + offset)
            right_pose.position.y = y
            right_pose.position.z = 0.5

            try:
                self.spawn_srv(f"dynamicbox_{wp}_left", model_xml, "", left_pose, "world")
                self.spawn_srv(f"dynamicbox_{wp}_right", model_xml, "", right_pose, "world")
                rospy.logwarn(f"Spawned obstacles at {wp}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Spawn failed at {wp}: {e}")


    def remove_obstacles(self):
        """Remove all dynamically spawned obstacles."""
        for wp in self.waypoints.keys():
            try:
                self.delete_srv(f"dynamicbox_{wp}_left")
                self.delete_srv(f"dynamicbox_{wp}_right")
                rospy.logwarn(f"Removed obstacles at {wp}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Delete failed at {wp}: {e}")
                
                
    # ROS Service Callbacks
    def handle_spawn_obstacles(self, req):
        """Service callback to spawn obstacles."""
        self.spawn_obstacles()
        return EmptyResponse()


    def handle_remove_obstacles(self, req):
        """Service callback to remove obstacles."""
        self.remove_obstacles()
        return EmptyResponse()


if __name__ == "__main__":
    obstacle_manager = DynamicObstacles()
    rospy.spin()