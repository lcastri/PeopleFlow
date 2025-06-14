#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Header
from people_msgs.msg import People
from people_msgs.msg import WPPeopleCounter, WPPeopleCounters
import hrisim_util.ros_utils as ros_utils


class PeopleCounter():
    def __init__(self) -> None:
        rospy.Subscriber('/people_tracker/people', People, self.cb_people)
        self.counter_pub = rospy.Publisher('/hrisim/people_counter', WPPeopleCounters, queue_size=10)
                
                
    def get_closestWP(self, p):
        potential_wp = {}
        for wp in WPS:
            d = math.sqrt((WPS[wp]['x'] - p[0])**2 + (WPS[wp]['y'] - p[1])**2)
            potential_wp[wp] = d
            
        return min(potential_wp, key=potential_wp.get) if potential_wp else None
         
         
    def cb_people(self, data: People):
        counter = {wp: 0 for wp in WPS}
        
        for person in data.people:
            p = [person.position.x, person.position.y]
            wp = self.get_closestWP(p)
            
            if wp is not None:
                counter[wp] += 1
                                
        msg = WPPeopleCounters()
        msg.header = Header()
        msg.counters = []
        
        for wp in counter:
            c = WPPeopleCounter()
            c.WP_id.data = wp
            c.numberOfPeople = counter[wp]
            
            msg.counters.append(c)
            
        self.counter_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('hrisim_people_counter')
    rate = rospy.Rate(10)  # 10 Hz
    WPS = ros_utils.wait_for_param("/hrisim/wps")

    PC = PeopleCounter()
    
    rospy.spin()
