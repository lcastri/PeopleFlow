#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf

# Variable to store the robot's last valid pose
last_valid_pose = None


from gazebo_msgs.srv import GetLinkState

def get_head_position():
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        head_link = "tiago::head_2_link"  # Adjust the link name according to TIAGo's URDF

        head_state = get_link_state(head_link, 'world')
        head_position_z = head_state.link_state.pose.position.z
        # rospy.loginfo("TIAGo head z-position: {}".format(head_position_z))
        return head_position_z
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None
    

def reset_robot():
    """Reset the robot to the last known valid pose."""
    if last_valid_pose is None:
        rospy.logerr("No valid pose recorded, can't reset.")
        return

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state_msg = ModelState()
        state_msg.model_name = 'tiago'

        # Use the stored valid pose to reset the robot (extract the `Pose` from `PoseWithCovariance`)
        state_msg.pose = last_valid_pose.pose.pose  # Access the 'pose' inside 'PoseWithCovariance'
        state_msg.reference_frame = 'world'
        set_model_state(state_msg)
        rospy.logwarn("Robot reset to last valid position.")

        # Publish the pose to /initialpose
        publish_initial_pose(last_valid_pose)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

        
        
def publish_initial_pose(pose_stamped):
    """Publish the reset pose to /initialpose for 2D localization."""
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header = pose_stamped.header
    pose_msg.pose.pose = pose_stamped.pose.pose  # Correctly extract the Pose

    # Optional: Set the covariance to indicate confidence in this estimate
    pose_msg.pose.covariance[0] = 0.25  # x covariance
    pose_msg.pose.covariance[7] = 0.25  # y covariance
    pose_msg.pose.covariance[35] = 0.0685  # yaw covariance (approx. 5 degrees)

    initial_pose_pub.publish(pose_msg)
    rospy.loginfo("Published initial pose to /initialpose")

    

def cb_pose(data):
    global last_valid_pose

    # Monitor the head position (z value)
    head_position_z = get_head_position()
    
    if head_position_z is not None:
        if head_position_z < 0.5:  # You can adjust the threshold based on your robot's normal head height
            rospy.logerr("Robot has fallen (head z-position: {:.2f}), resetting position.".format(head_position_z))
            reset_robot()
        else:
            # Store the current pose as the last valid one if the robot is upright
            last_valid_pose = data
    else:
        rospy.logerr("Could not retrieve the head position.")

def listener():
    global initial_pose_pub

    rospy.init_node('tiago_stand_up', anonymous=True)
    rospy.Subscriber("/robot_pose", PoseWithCovarianceStamped, cb_pose)
    initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
