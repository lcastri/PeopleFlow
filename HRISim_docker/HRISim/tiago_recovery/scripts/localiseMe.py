#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Header
from math import sqrt

# Threshold for position difference to detect misalignment
POSITION_THRESHOLD = 2.5  # Meters (you can adjust this value)

last_gazebo_pose = None  # To store the Gazebo pose
model_name = "tiago"  # Name of the robot model in Gazebo

# Publisher for the /initialpose topic
initial_pose_pub = None

def publish_initial_pose(pose):
    """Publish the pose to the /initialpose topic to reset localization in RViz."""
    pose_msg = PoseWithCovarianceStamped()

    pose_msg.header = Header()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "map"

    pose_msg.pose.pose = pose  # Set the pose from Gazebo
    pose_msg.pose.covariance[0] = 0.01  # x covariance
    pose_msg.pose.covariance[7] = 0.01  # y covariance
    pose_msg.pose.covariance[35] = 0.01  # yaw covariance

    initial_pose_pub.publish(pose_msg)
    rospy.logwarn("Published initial pose to /initialpose to reset localization.")

def pose_difference(pose1, pose2):
    """Calculate the Euclidean distance between two positions."""
    dx = pose1.x - pose2.x
    dy = pose1.y - pose2.y
    dz = pose1.z - pose2.z

    return sqrt(dx**2 + dy**2 + dz**2)

def cb_robot_pose(data):
    """Callback to check if the robot's pose in /robot_pose is aligned with the one in Gazebo."""
    global last_gazebo_pose

    if last_gazebo_pose is None:
        rospy.logwarn("No Gazebo pose received yet, skipping alignment check.")
        return

    # Compare the position from /robot_pose with the last Gazebo pose
    robot_position = data.pose.pose.position
    gazebo_position = last_gazebo_pose.position

    # Check if the positions differ by more than the threshold
    if pose_difference(robot_position, gazebo_position) > POSITION_THRESHOLD:
        rospy.logerr("Robot's position in /robot_pose is misaligned with Gazebo! Resetting.")
        publish_initial_pose(last_gazebo_pose)

def cb_model_states(data):
    """Callback to store the Gazebo model state for the TIAGo robot."""
    global last_gazebo_pose
    try:
        # Find the index of the TIAGo model in the Gazebo model states
        model_index = data.name.index(model_name)

        # Extract the pose from the Gazebo model state
        last_gazebo_pose = data.pose[model_index]
    except ValueError:
        rospy.logerr("Model name 'tiago' not found in /gazebo/model_states")

def listener():
    global initial_pose_pub

    rospy.init_node('tiago_localise_me', anonymous=True)

    # Subscribe to the /robot_pose topic
    rospy.Subscriber("/robot_pose", PoseWithCovarianceStamped, cb_robot_pose)

    # Subscribe to the /gazebo/model_states topic to get the robot's real pose in Gazebo
    rospy.Subscriber("/gazebo/model_states", ModelStates, cb_model_states)

    # Publisher to send 2D pose estimate to /initialpose
    initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
