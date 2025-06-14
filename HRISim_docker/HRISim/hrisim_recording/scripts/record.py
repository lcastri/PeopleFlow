#!/usr/bin/env python

import subprocess

import rospy
import signal
import hrisim_util.ros_utils as ros_utils

TOPICS = [
    "/map",
    "/tf",
    "/tf_static",
    "/robot_pose",
    "/mobile_base_controller/odom",
    "/move_base/goal",
    "/people_tracker/people",
    "/hrisim/people_counter",
    "/power/battery_level",
    "/hrisim/robot_closest_wp",
    "/velodyne_points"
]
   
if __name__ == '__main__':
    rospy.init_node('hrisim_recording')
    rate = rospy.Rate(10)  # 10 Hz
    
    TS = ros_utils.wait_for_param("/hrisim/time_slot")
    
    try:
        bag_process = subprocess.Popen(['rosbag', 'record', '-O', f'/root/shared/{TS}.bag'] + TOPICS, shell=False)
    except Exception as e:
        rospy.logerr(f"Failed to start ROS bag recording: {str(e)}")
        
    
    def shutdown_hook():
        if bag_process is not None:
            try:
                rospy.loginfo("Stopping the ROS bag recording...")
                bag_process.send_signal(signal.SIGINT) # Send SIGINT to the process (equivalent to pressing Ctrl+C)
                bag_process.wait() # Wait for the process to terminate
                rospy.loginfo("ROS bag recording stopped.")
            except Exception as e:
                rospy.logerr(f"Failed to stop ROS bag recording: {str(e)}")
                
    # Register the shutdown hook
    rospy.on_shutdown(shutdown_hook)             

    rospy.spin()