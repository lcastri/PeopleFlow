#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Header
from pedsim_msgs.msg import AgentStates
from peopleflow_msgs.msg import WPPeopleCounter, WPPeopleCounters
import hrisim_util.ros_utils as ros_utils

SELECTED_WP = ["door_entrance", "entrance", "door_entrance-canteen", "delivery_point",
               "corridor_entrance", "door_corridor1", "door_corridor2", "door_corridor3",
               "door_office1", "door_office2", "door_toilet1", "door_toilet2",
               "corridor0", "corridor1", "corridor2", "corridor3", "corridor4", "corridor5", 
               "office1", "office2", 
               "toilet1", "toilet2", 
               "table2", "table3", "table4", "table5", "table6",
               "kitchen1", "kitchen2", "kitchen3",
               "corr_canteen_1", "corr_canteen_2", "corr_canteen_3", 
               "corr_canteen_4", "corr_canteen_5", "corr_canteen_6", "corridor_canteen",
               "shelf12", "shelf23", "shelf34", "shelf45", "shelf56",
               "shelf1", "shelf2", "shelf3", "shelf4", "shelf5", "shelf6"]

HALL_DISTANCE = 3.7

class PeopleCounter():
    def __init__(self) -> None:
        # self.readScenario()
        self.timeOfDay = ''
        rospy.Subscriber('/pedsim_simulator/simulated_agents', AgentStates, self.cb_agentstates)
        self.counter_pub = rospy.Publisher('/peopleflow/counter', WPPeopleCounters, queue_size=10)
                
    def get_closestWP(self, p):
        potential_wp = {}
        for wp in WPS:
            if wp in SELECTED_WP:
                d = math.sqrt((WPS[wp]['x'] - p[0])**2 + (WPS[wp]['y'] - p[1])**2)
                if d <= HALL_DISTANCE: 
                    potential_wp[wp] = d
            
        return min(potential_wp, key=potential_wp.get) if potential_wp else None
         
    def cb_agentstates(self, data):
        counter = {wp: 0 for wp in SELECTED_WP}
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
    SCENARIO = str(rospy.get_param("/peopleflow_manager/scenario"))
    WPS = ros_utils.wait_for_param("/peopleflow/wps")
    ros_utils.wait_for_param("/peopleflow/timeday")

    PC = PeopleCounter()
    
    rospy.spin()
