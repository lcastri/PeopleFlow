#!/usr/bin/env python

import copy
import random

import rospy
import pickle
from pedsim_srvs.srv import GetNextDestination, GetNextDestinationResponse
from Agent import Agent
from std_msgs.msg import  Header, String
from geometry_msgs.msg import Point
from peopleflow_msgs.msg import pfAgents, pfAgent, Time as pT
import traceback
import hrisim_util.ros_utils as ros_utils
import time

SHELFS = ["shelf1", "shelf2", "shelf3", "shelf4", "shelf5", "shelf6"]
TIME_INIT = 8

def seconds_to_hhmmss(seconds):
    return time.strftime("%H:%M:%S", time.gmtime(seconds))

class PedsimBridge():
    def __init__(self):
        rospy.Subscriber("/peopleflow/time", pT, self.cb_time)
        self.timeOfDay = None
        self.agents = {}
        self.agents_pub = rospy.Publisher('/peopleflow/agents', pfAgents, queue_size=10)

        rospy.Service('get_next_destination', GetNextDestination, self.handle_get_next_destination)
        rospy.loginfo('ROS service /get_next_destination advertised')
        
    def cb_time(self, t: pT):
        self.timeOfDay = t.time_of_the_day.data
        self.elapsedTimeString = t.hhmmss.data
        self.elapsedTime = t.elapsed
        
    def get_agent(self, req) -> Agent:
        # Retrieving agent info
        if req.agent_id not in self.agents:
            self.agents[req.agent_id] = Agent(req.agent_id, copy.deepcopy(SCHEDULE), G, ALLOW_TASK, MAX_TASKTIME)
        self.agents[req.agent_id].x = req.origin.x
        self.agents[req.agent_id].y = req.origin.y
        self.agents[req.agent_id].isStuck = req.is_stuck
        
        return self.agents[req.agent_id]
     
    def handle_get_next_destination(self, req):
        try:
            agent = self.get_agent(req)
            
            # Entrance logic
            if (self.timeOfDay == 'starting' and not agent.atWork and 
                agent.isFree and not agent.isQuitting and 
                agent.closestWP == 'parking'):
                    
                # startingTime definition
                # next_destination response: 
                #   dest = parking, 
                #   task_duration = random.randint(0, SCHEDULE['starting']['duration'] - 10) 
                # ! this agent won't ask again a destination for "task_duration" seconds
                if agent.startingTime is None:
                    rospy.logwarn(f"{agent.id} ELIF 1.1")
                    agent.startingTime = random.randint(self.elapsedTime, SCHEDULE['starting']['duration'] - 10)
                    agent.exitTime = int(sum([SCHEDULE[t]['duration'] for t in SCHEDULE if t in ['starting','morning','lunch','afternoon']]) + agent.startingTime)
                    agent.setTask('parking', agent.startingTime)
                        
                # startingTime is now
                # next_destination response: 
                #   dest = agent.selectDestination(self.timeOfDay, req.destinations), 
                #   task_duration = 0 
                # ! atWork = True --> this agent won't enter again this if
                elif agent.startingTime is not None:
                    rospy.logwarn(f"{agent.id} ELIF 1.2")
                    next_destination = agent.selectDestination(self.timeOfDay, req.destinations)                           
                    agent.setTask(next_destination, agent.getTaskDuration())
                    agent.atWork = True
                
            # Quitting logic
            # next_destination response: 
            #   dest = parking, 
            #   task_duration = SCHEDULE['quitting']['duration'] - agent.startingTime + SCHEDULE['off']['duration']
            # ! isQuitting = True --> this agent won't enter again this if
            elif ((self.timeOfDay == 'quitting' or self.timeOfDay == 'off') and 
                  agent.atWork and agent.isFree and not agent.isQuitting and
                  self.elapsedTime >= agent.exitTime):
                
                rospy.logwarn(f"{agent.id} ELIF 2")
                agent.setTask('parking', SCHEDULE['quitting']['duration'] - agent.startingTime + SCHEDULE['off']['duration'])
                agent.isQuitting = True
    
            # New goal logic                
                
            elif agent.atWork and agent.isStuck:
                rospy.logwarn(f"{agent.id} ELIF 3")
                next_destination = agent.selectDestination(self.timeOfDay, req.destinations) if not agent.isQuitting else 'parking'
                agent.setTask(next_destination)
                rospy.logerr(f"Agent {agent.id} is stuck. new desination: {next_destination}")
                        
            elif agent.atWork and not agent.isStuck and agent.isQuitting and len(agent.path) == 1:
                rospy.logwarn(f"{agent.id} ELIF 4")
                agent.atWork = False
                agent.isQuitting = False
                
            elif agent.isFree and agent.atWork and not agent.isStuck and not agent.isQuitting:
                rospy.logwarn(f"{agent.id} ELIF 5")
                next_destination = agent.selectDestination(self.timeOfDay, req.destinations)
                agent.setTask(next_destination, agent.getTaskDuration())
                
            elif not agent.isFree:
                rospy.logwarn(f"{agent.id} ELIF 6")
                pass
                
            else:
                rospy.logerr(f"THERE IS A CASE I DID NOT COVER:")
                rospy.logerr(f"TOD {self.timeOfDay}")
                rospy.logerr(f"elapsedTime {self.elapsedTime}")
                rospy.logerr(f"Agent {agent.id}")
                rospy.logerr(f"startingTime {agent.startingTime}")
                rospy.logerr(f"exitTime {agent.exitTime}")
                rospy.logerr(f"atWork {agent.atWork}")
                rospy.logerr(f"isFree {agent.isFree}")
                rospy.logerr(f"isQuitting {agent.isQuitting}")
                rospy.logerr(f"isStuck {agent.isStuck}")
                rospy.logerr(f"closestWP {agent.closestWP}")
                
                    
            # Response
            wpname, wp = agent.nextWP         
            agent.nextDestRadius = WPS[wpname]["r"] if wpname in WPS else 1.0
            rospy.logwarn(f"{agent.id} RESPONSE")
            rospy.logwarn(f"---- {wpname}")
            rospy.logwarn(f"---- {agent.taskDuration[wpname]}")
            response = GetNextDestinationResponse(destination_id=wpname, 
                                                  destination=wp, 
                                                  destination_radius=WPS[wpname]["r"] if wpname in WPS else 1.0,
                                                  task_duration=agent.taskDuration[wpname])
            return response
        
        except Exception as e:
            rospy.logerr(f"Time: {self.timeOfDay} - {self.elapsedTimeString}")
            rospy.logerr(f"Agent {agent.id} generated error: {str(e)}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
    
    
    def pub_agents(self):
        msg_Agents = pfAgents()
        msg_Agents.header = Header()
            
        for a in self.agents.values():
            msg_Agent = pfAgent()
            msg_Agent.header = Header()
            msg_Agent.id = a.id
            msg_Agent.starting_time = String(data=seconds_to_hhmmss(TIME_INIT*3600 + a.startingTime) if a.startingTime is not None else "")
            msg_Agent.exit_time = String(data=seconds_to_hhmmss(TIME_INIT*3600 + a.exitTime) if a.exitTime is not None else "")
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
            msg_Agent.task_duration = a.taskDuration[a.nextDest] if a.nextDest is not None and a.taskDuration[a.nextDest] is not None else 0

            msg_Agents.agents.append(msg_Agent)
        self.agents_pub.publish(msg_Agents)
    
  
if __name__ == '__main__':
    rospy.init_node('peopleflow_pedsim_bridge')
    rate = rospy.Rate(10)  # 10 Hz
    
    SCHEDULE = ros_utils.wait_for_param("/peopleflow/schedule")
    WPS = ros_utils.wait_for_param("/peopleflow/wps")
    ALLOW_TASK = rospy.get_param("~allow_task", False)
    MAX_TASKTIME = int(rospy.get_param("~max_tasktime"))
    g_path = str(rospy.get_param("~g_path"))

    with open(g_path, 'rb') as f:
        G = pickle.load(f)
        G.remove_node("charging_station")
        
    pedsimBridge = PedsimBridge()
                
    while not rospy.is_shutdown():
        
        pedsimBridge.pub_agents()
               
        rate.sleep()