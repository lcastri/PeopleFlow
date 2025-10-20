#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Header
from pedsim_msgs.msg import AgentStates
from peopleflow_msgs.msg import WPPeopleCounter, WPPeopleCounters
import hrisim_util.ros_utils as ros_utils


class PeopleCounter():
    def __init__(self) -> None:
        # self.readScenario()
        self.timeOfDay = ''
        rospy.Subscriber('/pedsim_simulator/simulated_agents', AgentStates, self.cb_agentstates)
        self.counter_pub = rospy.Publisher('/peopleflow/counter', WPPeopleCounters, queue_size=10)
                
    def get_closestWP(self, p):
        potential_wp = {}
        for wp in WPS:
            d = math.sqrt((WPS[wp]['x'] - p[0])**2 + (WPS[wp]['y'] - p[1])**2)
            potential_wp[wp] = d
            
        return min(potential_wp, key=potential_wp.get) if potential_wp else None
         
    def cb_agentstates(self, data):
        counter = {wp: 0 for wp in WPS}
        self.timeOfDay = rospy.get_param("/peopleflow/timeday")
        
        for agent in data.agent_states:
            p = [agent.pose.position.x, agent.pose.position.y]
            wp = self.get_closestWP(p)
            
            if wp is not None:
                counter[wp] += 1
                                
        msg = WPPeopleCounters()
        msg.header = Header()
        msg.counters = []
        msg.numberOfWorkingPeople = 0
        
        for wp in counter:
            c = WPPeopleCounter()
            c.WP_id.data = wp
            c.time.data = self.timeOfDay
            c.numberOfPeople = counter[wp]
            
            msg.counters.append(c)
            msg.numberOfWorkingPeople += counter[wp]
            
        self.counter_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('peopleflow_peoplecounter')
    rate = rospy.Rate(10)  # 10 Hz
    SCENARIO = str(ros_utils.wait_for_param("/peopleflow_manager/scenario"))
    WPS = ros_utils.wait_for_param("/peopleflow/wps")
    ros_utils.wait_for_param("/peopleflow/timeday")

    PC = PeopleCounter()
    
    rospy.spin()
