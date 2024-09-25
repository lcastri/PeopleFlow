#!/usr/bin/env python

import copy
import random
import subprocess
import xml.etree.ElementTree as ET
import networkx as nx

import rospy
import pickle
from pedsim_srvs.srv import GetNextDestination, GetNextDestinationResponse
from Agent import Agent
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA, Header, String
from geometry_msgs.msg import Point
from peopleflow_msgs.msg import pfAgents, pfAgent, Time as pT
import traceback
import signal
import time

TIME_INIT = 8

def seconds_to_hhmmss(seconds):
    return time.strftime("%H:%M:%S", time.gmtime(seconds))

SHELFS = ["shelf1", "shelf2", "shelf3", "shelf4", "shelf5", "shelf6"]

# Define the heuristic function
def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


class Time:
    def __init__(self, name, duration) -> None:
        self.name = name
        self.duration = duration
        self.dests = {}
           
        
class ScenarioManager():
    def __init__(self):
        self.scenario = str(rospy.get_param("~scenario"))
        self.readScenario()
        rospy.sleep(0.1) # This is needed to not have rospy.Time.now() equal to 0
        self.initial_time = rospy.Time.now()
        self.time_of_the_day = ''
        self.agents = {}
        self.T = sum([self.schedule[time].duration for time in self.schedule])

        rospy.Service('get_next_destination', GetNextDestination, self.handle_get_next_destination)
        rospy.loginfo('ROS service /get_next_destination advertised')
           
           
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
        return str(seconds_to_hhmmss(TIME_INIT*3600 + self.elapsedTime))
    
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
            tmp = Time(time.get('name'), float(time.get('duration')) * (1/SCALING_FACTOR))
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


    def handle_get_next_destination(self, req):
        try:
            # Retrieving agent info
            if req.agent_id not in self.agents:
                self.agents[req.agent_id] = Agent(req.agent_id, copy.deepcopy(self.schedule), G)
            self.agents[req.agent_id].x = req.origin.x
            self.agents[req.agent_id].y = req.origin.y
            
            # Entrance logic
            if len(self.agents[req.agent_id].path) == 0:
                if (self.timeOfTheDay == 'starting' and not self.agents[req.agent_id].atWork and
                    not self.agents[req.agent_id].isQuitting and
                    self.agents[req.agent_id].closestWP == 'parking'):
                    
                    if self.agents[req.agent_id].startingTime is None: 
                        self.agents[req.agent_id].taskDuration = random.randint(0, self.schedule['starting'].duration - 10)
                        self.agents[req.agent_id].startingTime = self.agents[req.agent_id].taskDuration
                        self.agents[req.agent_id].setPath(['parking'])
                        self.agents[req.agent_id].exitTime = int(sum([self.schedule[t].duration for t in self.schedule if t in ['starting','morning','lunch','afternoon']]) + self.agents[req.agent_id].startingTime)
                        
                    elif self.agents[req.agent_id].startingTime is not None:
                        self.agents[req.agent_id].atWork = True
                
                # Quitting logic
                elif ((self.timeOfTheDay == 'quitting' or self.timeOfTheDay == 'off') and
                    self.agents[req.agent_id].atWork and
                    not self.agents[req.agent_id].isQuitting and
                    self.elapsedTime >= self.agents[req.agent_id].exitTime):
                    
                    path = nx.astar_path(G, self.agents[req.agent_id].closestWP, 'parking', heuristic=heuristic, weight='weight')
                    self.agents[req.agent_id].setPath(path)

                    self.agents[req.agent_id].isQuitting = True
                    self.agents[req.agent_id].taskDuration = 0
    
            # New goal logic
            if self.agents[req.agent_id].atWork:
                self.agents[req.agent_id].isStuck = req.is_stuck
                
                if req.is_stuck:
                    if not self.agents[req.agent_id].isQuitting:
                        next_destination = self.agents[req.agent_id].selectDestination(self.timeOfTheDay, req.destinations)
                    else:
                        next_destination = 'parking'
                    path = nx.astar_path(G, self.agents[req.agent_id].closestWP, next_destination, heuristic=heuristic, weight='weight')
                    self.agents[req.agent_id].setPath(path)
                    self.agents[req.agent_id].taskDuration = 0
                    rospy.logerr(f"Agent {req.agent_id} is stuck. new desination: {path[-1]}")
                    
                elif not req.is_stuck and not self.agents[req.agent_id].isQuitting:
                    if len(self.agents[req.agent_id].path) == 0:               
                        if self.agents[req.agent_id].closestWP in SHELFS:
                            next_destination = 'delivery_point'
                        else:
                            next_destination = self.agents[req.agent_id].selectDestination(self.timeOfTheDay, req.destinations)
                            self.agents[req.agent_id].taskDuration = 0
                            
                        path = nx.astar_path(G, self.agents[req.agent_id].closestWP, next_destination, heuristic=heuristic, weight='weight')
                        self.agents[req.agent_id].setPath(path)
                                            
                    elif len(self.agents[req.agent_id].path) == 1:
                        if ALLOW_TASK:
                            if self.agents[req.agent_id].finalDest.startswith("toilet"):
                                self.agents[req.agent_id].taskDuration = random.randint(2, 4)
                            else:
                                self.agents[req.agent_id].taskDuration = random.randint(2, MAX_TASKTIME)
                    else:
                        self.agents[req.agent_id].taskDuration = 0
                        
                elif not req.is_stuck and self.agents[req.agent_id].isQuitting:
                    if len(self.agents[req.agent_id].path) == 1:
                        self.agents[req.agent_id].taskDuration = self.schedule['quitting'].duration - self.agents[req.agent_id].startingTime + self.schedule['off'].duration
                        self.agents[req.agent_id].atWork = False
                        self.agents[req.agent_id].isQuitting = False
                    else:
                        self.agents[req.agent_id].taskDuration = 0
                    
            # Response
            wpname, wp = self.agents[req.agent_id].nextWP         
            self.agents[req.agent_id].nextDestRadius = self.wps[wpname]["r"] if wpname in self.wps else 1.0
            response = GetNextDestinationResponse(destination_id=wpname, 
                                                destination=wp, 
                                                destination_radius=self.wps[wpname]["r"] if wpname in self.wps else 1.0,
                                                task_duration=self.agents[req.agent_id].taskDuration)
            return response
        
        except Exception as e:
            rospy.logerr(f"Time: {self.timeOfTheDay} - {self.elapsedTimeString}")
            rospy.logerr(f"Agent {req.agent_id} generated error: {str(e)}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")

 
def pub_overlay_text():
    text = OverlayText()
    text.width = 300  # Width of the overlay
    text.height = 25  # Height of the overlay
    text.left = 10  # X position (left offset)
    text.top = 10  # Y position (top offset)
    text.text_size = 13  # Font size
    text.line_width = 2
    text.text = f"{str(SM.timeOfTheDay).capitalize()}: {str(SM.elapsedTimeString)}"
    text.font = "DejaVu Sans Mono"
       
    # Text color
    text.fg_color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # RGBA (White)
    
    rospy.set_param('/peopleflow/timeday', str(SM.timeOfTheDay))
    overlaytime_pub.publish(text)
    
    
def pub_time():
    msg = pT()
    msg.header = Header()
    msg.time_of_the_day.data = str(SM.timeOfTheDay)
    msg.hhmmss.data = str(SM.elapsedTimeString)
    msg.elapsed = SM.elapsedTime
    time_pub.publish(msg)
    
    
def pub_agents():
    msg_Agents = pfAgents()
    msg_Agents.header = Header()
        
    for a in SM.agents.values():
        msg_Agent = pfAgent()
        msg_Agent.header = Header()
        msg_Agent.id = a.id
        msg_Agent.starting_time = a.startingTime if a.startingTime is not None else 0
        msg_Agent.exit_time = a.exitTime if a.exitTime is not None else 0
        msg_Agent.position = Point(a.x, a.y, 0)
        msg_Agent.is_stuck.data = a.isStuck
        msg_Agent.at_work.data = a.atWork
        msg_Agent.is_quitting.data = a.isQuitting
        msg_Agent.past_WP_id = String(data=a.pastDest if a.pastDest is not None else "")
        msg_Agent.current_WP_id = String(data=a.currDest if a.currDest is not None else "")
        msg_Agent.path = [String(data=wp or "") for wp in a.path]
        msg_Agent.next_WP_id = String(data=a.nextDest if a.nextDest is not None else "")
        msg_Agent.next_destination = a.nextDestPos if a.nextDestPos is not None else Point(0,0,0)
        msg_Agent.next_destination_radius = a.nextDestRadius if a.nextDestPos is not None else 0
        msg_Agent.final_WP_id = String(data=a.finalDest if a.finalDest is not None else "")
        msg_Agent.task_duration = a.taskDuration if a.taskDuration is not None else 0

        msg_Agents.agents.append(msg_Agent)
    agents_pub.publish(msg_Agents)
    

def isFinished():
    if SM.elapsedTime > SM.T:
        try:
            subprocess.Popen(['bash', '-c', 'tmux send-keys -t HRISim_bringup:0.0 "tstop" C-m'], shell=False)
        except Exception as e:
            rospy.logerr(f"Failed to execute tstop: {str(e)}")

TOPICS = "/map /tf /tf_static /robot_pose /mobile_base_controller/odom /move_base/goal /pedsim_simulator/simulated_agents /peopleflow/counter /peopleflow/time /hrisim/tiago_battery /hrisim/robot_task"
   
if __name__ == '__main__':
    rospy.init_node('peopleflow_manager')
    rate = rospy.Rate(10)  # 10 Hz
    
    SCENARIO = str(rospy.get_param("~scenario"))
    ALLOW_TASK = rospy.get_param("~allow_task", False)
    MAX_TASKTIME = int(rospy.get_param("~max_tasktime"))
    SCALING_FACTOR = int(rospy.get_param("~scaling_factor", 1))
    g_path = str(rospy.get_param("~g_path"))
    RECORD = rospy.get_param("~record", False)
    STARTING_ELAPSED = int(rospy.get_param("~starting_elapsed", 8)) - TIME_INIT
    
    with open(g_path, 'rb') as f:
        G = pickle.load(f)
        
    if RECORD:
        try:
            bag_process = subprocess.Popen(['rosbag', 'record', '-O', '/root/shared/experiment.bag'] + TOPICS.split(), shell=False)
        except Exception as e:
            rospy.logerr(f"Failed to start ROS bag recording: {str(e)}")
        
    SM = ScenarioManager()
    
    def shutdown_hook():
        if bag_process is not None:
            try:
                rospy.loginfo("Stopping the ROS bag recording...")
                # Send SIGINT to the process (equivalent to pressing Ctrl+C)
                bag_process.send_signal(signal.SIGINT)
                bag_process.wait()  # Wait for the process to terminate
                rospy.loginfo("ROS bag recording stopped.")
            except Exception as e:
                rospy.logerr(f"Failed to stop ROS bag recording: {str(e)}")
                
    # Register the shutdown hook
    if RECORD: rospy.on_shutdown(shutdown_hook)
                
    overlaytime_pub = rospy.Publisher('/peopleflow/timeday', OverlayText, queue_size=10)
    time_pub = rospy.Publisher('/peopleflow/time', pT, queue_size=10)
    agents_pub = rospy.Publisher('/peopleflow/agents', pfAgents, queue_size=10)
    task_pub = rospy.Publisher('/hrisim/robot_task', String, queue_size=10)

    while not rospy.is_shutdown():
        if RECORD: isFinished()
        
        # Overlay
        pub_overlay_text()

        # Time text
        pub_time()
        
        # Agents
        pub_agents()
        
        # Robot task
        task_pub.publish(String(rospy.get_param("/hrisim/robot_task", 'none')))
        
        rate.sleep()