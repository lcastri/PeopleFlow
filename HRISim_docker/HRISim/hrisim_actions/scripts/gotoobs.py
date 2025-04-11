import os
import random
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from AbstractAction import AbstractAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math
from std_srvs.srv import Empty  # Import the Empty service


class gotoobs(AbstractAction):

    def _start_action(self):

        rospy.set_param('/hri/robot_busy', True)
        rospy.logwarn(f"STARTED goto " + " ".join(self.params))
        TIME_THRESHOLD = float(self.params[3])
        SPAWN_ENABLED = True if self.params[4] == '1' else False

        if len(self.params) < 1:
            rospy.logwarn("Wrong use of action, pass the coordinates the robots needs to reach in /map frame as X_Y_Theta")
        else:
            self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            self.client.wait_for_server()

            # Create goal
            self.goal_msg = MoveBaseGoal()
            self.goal_msg.target_pose.header.frame_id = "map"
            self.goal_msg.target_pose.header.stamp = rospy.Time.now()
            self.goal_msg.target_pose.pose.position.x = float(self.params[0])
            self.goal_msg.target_pose.pose.position.y = float(self.params[1])
            self.goal_msg.target_pose.pose.orientation.z = math.sin(float(self.params[2]) / 2)
            self.goal_msg.target_pose.pose.orientation.w = math.cos(float(self.params[2]) / 2)

            self.client.send_goal(self.goal_msg)

            #! Dynamic obstacle spawning only if the next goal is not the last one
            if SPAWN_ENABLED:
                if random.choices([True, False], [0.25, 0.75])[0]:
                # if random.choices([True, False], [0.3, 0.7])[0]:
                    dynobs_spawn_service = rospy.ServiceProxy('/hrisim/obstacles/spawn', Empty)
                    dynobs_spawn_service()
            
            rospy.loginfo("Waiting for goTo result...")
            if TIME_THRESHOLD > 0:
                self.client.wait_for_result(timeout=rospy.Duration(TIME_THRESHOLD))
            else:
                self.client.wait_for_result()
            
            # Check result state
            result_state = self.client.get_state()
            if result_state == actionlib.GoalStatus.SUCCEEDED:
                rospy.set_param('/hrisim/goal_status', 1)
                self._on_goTo_done()
            else:
                rospy.set_param('/hrisim/goal_status', -1)
                self.client.cancel_all_goals()
                self._stop_action()

    def _on_goTo_done(self):
        self.params.append("done")
        rospy.logwarn(f"DONE goto " + " ".join(self.params))

    def _stop_action(self):
        rospy.loginfo("Waiting for goTo cancelling...")
        self.params.append("interrupted")
        rospy.logwarn(f"INTERRUPTED goto " + " ".join(self.params))
        
    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0:
            if params[-1] == "done" or params[-1] == "interrupted":
                rospy.set_param('/hrisim/robot_busy', False)
                reached = True
        return reached
