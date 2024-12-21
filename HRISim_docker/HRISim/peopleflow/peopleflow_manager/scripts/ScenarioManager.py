#!/usr/bin/env python

import xml.etree.ElementTree as ET

import rospy
from std_msgs.msg import Header, String
from peopleflow_msgs.msg import Time as pT
import hrisim_util.ros_utils as ros_utils
import subprocess

TIME_INIT = 8
TSTOP = False


class Time:
    def __init__(self, name, duration) -> None:
        self.name = name
        self.duration = duration
        self.dests = {}
           
        
class ScenarioManager():
    def __init__(self):
        self.readScenario()
        rospy.sleep(0.1) # This is needed to not have rospy.Time.now() equal to 0
        self.initial_time = rospy.Time.now()
        
    @property
    def T(self):
        return sum([self.schedule[time].duration for time in self.schedule])
           
    @property
    def timeOfTheDay(self):
        d = 0
        for time in self.schedule:
            d += self.schedule[time].duration
            if self.elapsedTime > d:
                continue
            else:
                return self.schedule[time].name
            
    @property
    def elapsedTimeString(self):
        d = 0
        for time in self.schedule:
            d += self.schedule[time].duration
            if self.elapsedTime > d:
                continue
            else:
                break
        return str(ros_utils.seconds_to_hhmmss(TIME_INIT*3600 + self.elapsedTime))
    
    @property
    def elapsedTime(self):
        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.initial_time).to_sec() + STARTING_ELAPSED*3600
        return int(elapsed_time)
            
            
    def readScenario(self):
        # Load and parse the XML file
        self.schedule = {}
        tree = ET.parse(SCENARIO + '.xml')
        root = tree.getroot()
        
        # Parse schedule
        for time in root.find('schedule').findall('time'):
            tmp = Time(time.get('name'), float(time.get('duration')))
            for adddest in time.findall('adddest'):
                dest_name = adddest.get('name')
                tmp.dests[dest_name] = {'mean': tmp.duration * float(adddest.get('p')), 'std': float(adddest.get('std'))}
            self.schedule[tmp.name] = tmp
        
        self.wps = {}
        for waypoint in root.findall('waypoint'):
            waypoint_id = waypoint.get('id')
            x = float(waypoint.get('x'))
            y = float(waypoint.get('y'))
            r = float(waypoint.get('r'))
            self.wps[waypoint_id] = {'x': x, 'y': y, 'r': r}
            
        rospy.set_param("/peopleflow/schedule", self.schedule)
        rospy.set_param("/peopleflow/wps", self.wps)

 
def pub_time():
    msg = pT()
    msg.header = Header()
    msg.time_of_the_day.data = str(SM.timeOfTheDay)
    msg.hhmmss.data = str(SM.elapsedTimeString)
    msg.elapsed = SM.elapsedTime
    time_pub.publish(msg)
    
    
def isFinished():
    global TSTOP
    rospy.set_param("/peopleflow/robot_plan_on", True)
    if SM.elapsedTime is not None and (SM.elapsedTime > SM.T or not ros_utils.wait_for_param("/peopleflow/robot_plan_on")) and not TSTOP:
        try:
            rospy.logwarn(f"Calling shutdown...")
            subprocess.Popen(['bash', '-c', 'tmux send-keys -t HRISim_bringup:0.0 "tstop" C-m'], shell=False)
            TSTOP = True
            rospy.logwarn(f"Shutting down...")
        except Exception as e:
            rospy.logerr(f"Failed to execute tstop: {str(e)}")
    
    
if __name__ == '__main__':
    rospy.init_node('peopleflow_manager')
    rate = rospy.Rate(10)  # 10 Hz
    
    SCENARIO = str(rospy.get_param("~scenario"))
    STARTING_ELAPSED = int(rospy.get_param("~starting_elapsed", 8)) - TIME_INIT
    
    SM = ScenarioManager()
                    
    time_pub = rospy.Publisher('/peopleflow/time', pT, queue_size=10)
    task_pub = rospy.Publisher('/hrisim/robot_task', String, queue_size=10)

    while not rospy.is_shutdown():
        # Time
        pub_time()
        rospy.set_param('/peopleflow/timeday', str(SM.timeOfTheDay))
        
        isFinished()
        
        
        # Robot task
        task_pub.publish(String(rospy.get_param("/hrisim/robot_task", 'none')))
        
        rate.sleep()