#!/usr/bin/env python

import math
import rospy
import xml.etree.ElementTree as ET
from std_msgs.msg import Header
from pedsim_msgs.msg import AgentStates
from peopleflow_msgs.msg import WPPeopleCounter, WPPeopleCounters

SELECTED_WP = ["entrance", "delivery_point", "corridor0", "corridor1", "corridor2", "corridor3", "corridor4", 
               "corridor5", "office1", "office2", "toilet1", "toilet2", "table2", "table3", "table4", "table5",
               "table6", "shelf12", "shelf23", "shelf34", "shelf45", "shelf56", "kitchen1", "kitchen2", "kitchen3",
               "shelf1", "shelf2", "shelf3", "shelf4", "shelf5", "shelf6"]

HALL_DISTANCE = 3.7

class PeopleCounter():
    def __init__(self) -> None:
        self.readScenario()
        self.timeOfDay = ''
        rospy.Subscriber('/pedsim_simulator/simulated_agents', AgentStates, self.cb_agentstates)
        self.counter_pub = rospy.Publisher('/peopleflow/counter', WPPeopleCounters, queue_size=10)

    def readScenario(self):
        # Load and parse the XML file
        self.schedule = {}
        tree = ET.parse(SCENARIO + '.xml')
        root = tree.getroot()
               
        self.wps = {}
        for waypoint in root.findall('waypoint'):
            waypoint_id = waypoint.get('id')
            if waypoint_id in SELECTED_WP:
                x = float(waypoint.get('x'))
                y = float(waypoint.get('y'))
                self.wps[waypoint_id] = {'x': x, 'y': y}
                
    def get_closestWP(self, p):
        potential_wp = {}
        for wp in self.wps:
            d = math.sqrt((self.wps[wp]['x'] - p[0])**2 + (self.wps[wp]['y'] - p[1])**2)
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

    PC = PeopleCounter()
    
    rospy.spin()
