#!/usr/bin/env python

import rospy
import pickle
from std_msgs.msg import  Header, String
from geometry_msgs.msg import Point
from peopleflow_msgs.msg import pfAgents, pfAgent
import hrisim_util.ros_utils as ros_utils
from hrisim_util.Agent import Agent 
import time

SHELFS = ["shelf1", "shelf2", "shelf3", "shelf4", "shelf5", "shelf6"]
TIME_INIT = 8

def seconds_to_hhmmss(seconds):
    return time.strftime("%H:%M:%S", time.gmtime(seconds))

class PFAgentsInfo():
    def __init__(self):
        self.agents_pub = rospy.Publisher('/peopleflow/agents', pfAgents, queue_size=10)
        
    def load_agents(self):
        """Load agents from the ROS parameter server."""
        self.agents = {}
        agents_param = rospy.get_param('/peopleflow/agents', None)
        if agents_param is not None:
            self.agents = {agent_id: Agent.from_dict(agent_data, SCHEDULE, G, ALLOW_TASK, MAX_TASKTIME) for agent_id, agent_data in agents_param.items()} 
       
    def pub_agents(self):
        self.load_agents()

        msg_Agents = pfAgents()
        msg_Agents.header = Header()
            
        for a in self.agents.values():
            msg_Agent = pfAgent()
            msg_Agent.header = Header()
            msg_Agent.id = int(a.id)
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
            msg_Agent.next_destination = Point(a.nextDestPos[0], a.nextDestPos[1], 0) if a.nextDestPos is not None else Point(0,0,0)
            msg_Agent.next_destination_radius = a.nextDestRadius if a.nextDestPos is not None else 0
            msg_Agent.final_WP_id = String(data=a.finalDest if a.finalDest is not None else "")
            msg_Agent.task_duration = a.taskDuration[a.nextDest] if not a.nextDest in [None, ''] and a.taskDuration[a.nextDest] is not None else 0

            msg_Agents.agents.append(msg_Agent)
        self.agents_pub.publish(msg_Agents)
    
  
if __name__ == '__main__':
    rospy.init_node('peopleflow_agents_info')
    rate = rospy.Rate(10)  # 10 Hz
    
    SCHEDULE = ros_utils.wait_for_param("/peopleflow/schedule")
    WPS = ros_utils.wait_for_param("/peopleflow/wps")
    ALLOW_TASK = rospy.get_param("/peopleflow_pedsim_bridge/allow_task", False)
    MAX_TASKTIME = int(rospy.get_param("/peopleflow_pedsim_bridge/max_tasktime"))
    g_path = str(rospy.get_param("/peopleflow_pedsim_bridge/g_path"))

    with open(g_path, 'rb') as f:
        G = pickle.load(f)
        G.remove_node("charging_station")
        
    pfagents = PFAgentsInfo()
                
    while not rospy.is_shutdown():
        
        pfagents.pub_agents()
               
        rate.sleep()