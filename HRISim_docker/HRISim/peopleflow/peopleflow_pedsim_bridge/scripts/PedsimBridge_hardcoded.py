#!/usr/bin/env python

import rospy
import pickle
from pedsim_srvs.srv import GetNextDestination, GetNextDestinationResponse
from peopleflow_msgs.msg import Time as pT
from hrisim_util.Agent import Agent 
import hrisim_util.ros_utils as ros_utils
import hrisim_util.constants as constants
import traceback
import time
from std_srvs.srv import Trigger

TIME_INIT = 8

def seconds_to_hhmmss(seconds):
    return time.strftime("%H:%M:%S", time.gmtime(seconds))

class PedsimBridge():
    def __init__(self):
        self.timeDefined = False
        rospy.Subscriber("/peopleflow/time", pT, self.cb_time)
        
        while not self.timeDefined: rospy.sleep(0.1)
        
        rospy.Service('get_next_destination', GetNextDestination, self.handle_get_next_destination)
        rospy.loginfo('ROS service /get_next_destination advertised')
        
    def cb_time(self, t: pT):
        self.timeOfDay = t.time_of_the_day.data
        self.elapsedTimeString = t.hhmmss.data
        self.elapsedTime = t.elapsed
        self.timeDefined = True
        
    def load_agents(self, req):
        """Load agents from the ROS parameter server."""
        agent_id = str(req.agent_id)
        agents_param = rospy.get_param(f'/peopleflow/agents/{agent_id}', None)
        if agents_param is not None:
            a = Agent.from_dict(agents_param, SCHEDULE, G, ALLOW_TASK, MAX_TASKTIME)
        else:
            a = Agent(agent_id, SCHEDULE, G, ALLOW_TASK, MAX_TASKTIME)
        a.x = req.origin.x
        a.y = req.origin.y
        a.isStuck = req.is_stuck
        return a
    
    def save_agents(self, agent):
        """Save agents to the ROS parameter server."""
        rospy.set_param(f'/peopleflow/agents/{agent.id}', agent.to_dict())
             
    def handle_get_next_destination(self, req):
        try:
            # Load agent from rosparam
            agent = self.load_agents(req)
                        
            # Entrance logic
            if (self.timeOfDay == constants.TOD.H1.value and not agent.atWork and 
                agent.isFree and not agent.isQuitting and 
                agent.closestWP == constants.WP.PARKING.value):
                    
                # startingTime definition
                # next_destination response: 
                #   dest = parking, 
                #   task_duration = random.randint(0, SCHEDULE['starting']['duration'] - 10) 
                # ! this agent won't ask again a destination for "task_duration" seconds
                if agent.startingTime is None:
                    agent.startingTime = AGENTSPLAN[int(agent.id)]['startTime']
                    agent.exitTime = AGENTSPLAN[int(agent.id)]['exitTime']
                    agent.setTask(constants.WP.PARKING.value, agent.startingTime)
                        
                # startingTime is now
                # next_destination response: 
                #   dest = agent.selectDestination(self.timeOfDay, req.destinations), 
                #   task_duration = 0 
                # ! atWork = True --> this agent won't enter again this if
                else:
                    # next_destination = agent.selectDestination(self.timeOfDay, req.destinations)                           
                    next_destination = AGENTSPLAN[int(agent.id)]['tasks'][self.timeOfDay]['destinations'].pop(0)                           
                    agent.setTask(next_destination, AGENTSPLAN[int(agent.id)]['tasks'][self.timeOfDay]['durations'].pop(0))
                    agent.atWork = True
                
            # Quitting logic
            # next_destination response: 
            #   dest = parking, 
            #   task_duration = SCHEDULE['quitting']['duration'] - agent.startingTime + SCHEDULE['off']['duration']
            # ! isQuitting = True --> this agent won't enter again this if
            elif ((self.timeOfDay == constants.TOD.H10.value or self.timeOfDay == constants.TOD.OFF.value) and 
                  agent.atWork and agent.isFree and not agent.isQuitting and
                  self.elapsedTime >= agent.exitTime):
                
                rospy.logerr(f'Agent {agent.id} is quitting..')
                
                agent.setTask(constants.WP.PARKING.value, SCHEDULE[constants.TOD.H10.value]['duration'] - agent.startingTime + SCHEDULE[constants.TOD.OFF.value]['duration'])
                agent.isQuitting = True
    
            # New goal logic                
            elif agent.atWork and agent.isStuck:
                if not agent.isQuitting:
                    next_destination = AGENTSPLAN[int(agent.id)]['tasks'][self.timeOfDay]['destinations'].pop(0)                           
                    agent.setTask(next_destination, AGENTSPLAN[int(agent.id)]['tasks'][self.timeOfDay]['durations'].pop(0))                          
                else:
                    agent.setTask(constants.WP.PARKING.value, SCHEDULE[constants.TOD.H10.value]['duration'] - agent.startingTime + SCHEDULE[constants.TOD.OFF.value]['duration'])
                                            
            elif agent.atWork and not agent.isStuck and agent.isQuitting and len(agent.path) == 1:
                agent.atWork = False
                agent.isQuitting = False
                
            elif agent.isFree and agent.atWork and not agent.isStuck and not agent.isQuitting:
                next_destination = AGENTSPLAN[int(agent.id)]['tasks'][self.timeOfDay]['destinations'].pop(0)                               
                agent.setTask(next_destination, AGENTSPLAN[int(agent.id)]['tasks'][self.timeOfDay]['durations'].pop(0))
                
            elif not agent.isFree:
                pass
                
            else:
                rospy.logerr("THERE IS A CASE I DID NOT COVER:")
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
            response = GetNextDestinationResponse(destination_id=wpname, 
                                                  destination=wp, 
                                                  destination_radius=WPS[wpname]["r"] if wpname in WPS else 1.0,
                                                  task_duration=agent.taskDuration[wpname])
            # Save agents to rosparam
            self.save_agents(agent)
            return response
        
        except Exception as e:
            rospy.logerr(f"Time: {self.timeOfDay} - {self.elapsedTimeString}")
            rospy.logerr(f"Agent {agent.id} generated error: {str(e)}")
            rospy.logerr(f"Traceback: {traceback.format_exc()}")
       
  
if __name__ == '__main__':
    rospy.init_node('peopleflow_pedsim_bridge')
    rate = rospy.Rate(10)  # 10 Hz
    
    SCHEDULE = ros_utils.wait_for_param("/peopleflow/schedule")
    WPS = ros_utils.wait_for_param("/peopleflow/wps")
    ALLOW_TASK = rospy.get_param("~allow_task", False)
    MAX_TASKTIME = int(rospy.get_param("~max_tasktime"))
    rospy.wait_for_service('/update_graph_visualization')

    
    g_path = str(rospy.get_param("~g_path"))
    with open(g_path, 'rb') as f:
        G = pickle.load(f)
        ros_utils.load_graph_to_rosparam(G, "/peopleflow/G")
        
        # Create a handle for the Trigger service
        update_service = rospy.ServiceProxy('/update_graph_visualization', Trigger)
        # Call the service
        response = update_service()
        # Check the response
        if response.success:
            rospy.loginfo(f"Graph visualization updated successfully: {response.message}")
        else:
            rospy.logwarn(f"Graph visualization update failed: {response.message}")

        G.remove_node(constants.WP.CHARGING_STATION.value)
        
    agentsplan_path = '/root/ros_ws/src/HRISim/peopleflow/peopleflow_manager/hardcode/agent_task_list.pkl'
    with open(agentsplan_path, 'rb') as f:
        AGENTSPLAN = pickle.load(f)
        
    pedsimBridge = PedsimBridge()
                
    rospy.spin()