import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from AbstractAction import AbstractAction
import actionlib
from pal_interaction_msgs.msg import TtsActionGoal, TtsAction, TtsGoal

"""
Move the head to a specific pan-tilt angle configuration.
Arguments:
    - pan angle
    - tile angle
"""
class speak(AbstractAction):

    def _start_action(self):
        rospy.loginfo('About to say ' + " ".join(self.params) + ' ...')
        #self.starting_time = rospy.Time.now()

        if len(self.params) < 1:
            rospy.logwarn("Wrong use of action, pass the string replacing all spaces with underscores. E.g.: 'Hello_world!'")
        else:
            self.ac = actionlib.SimpleActionClient("/tts", TtsAction)
            rospy.loginfo("Connecting to /tts AS...")
            self.ac.wait_for_server()
            rospy.loginfo("Connected.")

            # create goal
            self.goal = TtsActionGoal()
            # rospy.loginfo(self.goal)
            self.goal.goal.rawtext.text = " ".join(self.params)
            self.goal.goal.rawtext.lang_id = "en_GB"

            # # collect spoken words
            # self.spoken = []
            # send goal
            self.ac.send_goal(self.goal.goal, done_cb=self._on_speak_done)
            rospy.loginfo("Waiting for result...")


    def _on_speak_done(self, goalState, result):
        print("Speak DONE", goalState, result)

        # self.spoken.append(str(result.text))

        self.params.append("done")

    def _stop_action(self):

        self.ac.cancel_all_goals()
        rospy.loginfo('STOPPED speak action')

    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True

        return reached