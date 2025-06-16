#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Header
from people_msgs.msg import People
from hrisim_people_counter.msg import WPPeopleCounter, WPPeopleCounters
import hrisim_util.ros_utils as ros_utils


class PeopleCounter():
    def __init__(self) -> None:
        rospy.Subscriber('/people_tracker/people', People, self.cb_people)
                
                
    def get_closestWP(self, p):
        potential_wp = {}
        for wp in WPS:
            d = math.sqrt((WPS[wp]['x'] - p[0])**2 + (WPS[wp]['y'] - p[1])**2)
            potential_wp[wp] = d
            
        return min(potential_wp, key=potential_wp.get) if potential_wp else None
         
         
    def cb_people(self, data: People):
        global COUNTERS
        # counter = {wp: 0 for wp in WPS}
        
        for person in data.people:
            p = [person.position.x, person.position.y]
            wp = self.get_closestWP(p)
            
            if wp is not None:
                COUNTERS[wp] += 1


if __name__ == '__main__':
    rospy.init_node('hrisim_people_counter')
    rate = rospy.Rate(10)  # 10 Hz
    WPS = ros_utils.wait_for_param("/hrisim/wps")
    
    counter_pub = rospy.Publisher('/hrisim/people_counter', WPPeopleCounters, queue_size=10)
    COUNTERS = {wp: 0 for wp in WPS}
    
    PC = PeopleCounter()
    
    while not rospy.is_shutdown():
        counters_msg = WPPeopleCounters()
        counters_msg.header = Header()
        counters_msg.counters = []
        for wp in COUNTERS:
            counter_msg = WPPeopleCounter()
            counter_msg.WP_id.data = wp
            counter_msg.numberOfPeople = COUNTERS[wp]
            counters_msg.counters.append(counter_msg)
        counter_pub.publish(counters_msg)
        COUNTERS = {wp: 0 for wp in WPS}
        rate.sleep()