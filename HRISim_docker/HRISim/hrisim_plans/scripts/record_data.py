import os
import sys
import rospy
try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import pnp_cmd_ros
from pnp_cmd_ros import *
import hrisim_util.ros_utils as ros_utils
import subprocess


def shutdown():
    try:
        rospy.logwarn(f"Calling shutdown...")
        subprocess.Popen(['bash', '-c', 'tmux send-keys -t TIAGo:0.0 "tstop" C-m'], shell=False)
        rospy.logwarn(f"Shutting down...")
    except Exception as e:
        rospy.logerr(f"Failed to execute tstop: {str(e)}")
        
        
def Plan(p):
    while not ros_utils.wait_for_param("/pnp_ros/ready"): rospy.sleep(0.1)
        
    p.exec_action('speak', 'Recording_data')
    
    # Wait for DURATION seconds
    start_time = rospy.Time.now()
    duration = rospy.Duration(DURATION)

    while rospy.Time.now() - start_time < duration:
        rospy.sleep(1.0)  # sleep to avoid busy-waiting

    p.exec_action('speak', 'Recording_completed!')
    shutdown()
    
   
if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python script.py SCENARIO")
        sys.exit(1)
    SCENARIO = sys.argv[1]
    DURATION = int(sys.argv[2])
    
    p = PNPCmd()
    rospy.set_param('/hrisim/time_slot', SCENARIO)

    p.begin()

    Plan(p)

    p.end()