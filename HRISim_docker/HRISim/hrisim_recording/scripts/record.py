#!/usr/bin/env python

import subprocess

import rospy
from peopleflow_msgs.msg import Time as pT
import signal
import hrisim_util.ros_utils as ros_utils

TSTOP = False
elapsedTime = None

def cb_time(t: pT):
    global elapsedTime
    elapsedTime = t.elapsed

def isFinished():
    global TSTOP
    global elapsedTime
    
    if elapsedTime is not None and (elapsedTime > T or not ros_utils.wait_for_param("/peopleflow/robot_plan_on")) and not TSTOP:
        try:
            rospy.logwarn(f"Calling shutdown...")
            subprocess.Popen(['bash', '-c', 'tmux send-keys -t HRISim_bringup:0.0 "tstop" C-m'], shell=False)
            TSTOP = True
            rospy.logwarn(f"Shutting down...")
        except Exception as e:
            rospy.logerr(f"Failed to execute tstop: {str(e)}")

TOPICS = "/map /tf /tf_static /robot_pose /mobile_base_controller/odom /move_base/goal /pedsim_simulator/simulated_agents /peopleflow/counter /peopleflow/time /hrisim/robot_battery /hrisim/robot_task /hrisim/robot_closest_wp /hrisim/robot_bac /hrisim/robot_past_bac"
   
if __name__ == '__main__':
    rospy.init_node('hrisim_recording')
    rate = rospy.Rate(10)  # 10 Hz
    
    schedule = ros_utils.wait_for_param("/peopleflow/schedule")
    T = sum([schedule[time]['duration'] for time in schedule])          
    
    try:
        bag_process = subprocess.Popen(['rosbag', 'record', '-O', '/root/shared/experiment.bag'] + TOPICS.split(), shell=False)
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