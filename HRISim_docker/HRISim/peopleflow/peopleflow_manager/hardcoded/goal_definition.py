import pickle
import random
import numpy as np
import constants as constants
import xml.etree.ElementTree as ET

MAX_TASK_TIME = 60

class Time:
    def __init__(self, name, duration) -> None:
        self.name = name
        self.duration = duration
        self.dests = {}

def readScenario():
    # Load and parse the XML file
    tree = ET.parse(SCENARIO)
    root = tree.getroot()       
        
    # Parse schedule
    schedule = {}
    for time in root.find('schedule').findall('time'):
        tmp = Time(time.get('name'), float(time.get('duration')))
        for adddest in time.findall('adddest'):
            dest_name = adddest.get('name')
            tmp.dests[dest_name] = {'mean': float(adddest.get('p')), 'std': float(adddest.get('std'))}
        schedule[tmp.name] = tmp
    
    
    # Parse agents
    agents = {}
    for agent in root.findall('agent'):
        agent_id = len(agents) + 1 #! THIS MUST BE DECOMMENTED IF YOU CONSIDER TIAGo AS AN AGENT
        # agent_id = len(agents)  #! THIS MUST BE DECOMMENTED IF YOU DON'T CONSIDER TIAGo AS AN AGENT
        agent_type = agent.get('type')
        if agent_type == "2": continue

        agents[agent_id] = {}
        agents[agent_id]['potential_dests'] = []
        for wp in agent.findall('addwaypoint'):
            agents[agent_id]['potential_dests'].append(wp.get('id'))
        
    # return wps, agents, schedule
    return agents, schedule


def selectDestination(selected_time, potential_dests):
    destinations = SCHEDULE[selected_time].dests
    #! I am commenting this line to allow the agents to select the same destination
    # if self.pastFinalDest is not None and self.pastFinalDest != 'delivery_point': potential_dests.remove(self.pastFinalDest)
    
    # Generate probabilities
    tmp_dest = []
    probabilities = []
    for dest in potential_dests:
        mean = destinations[dest]['mean']
        
        if mean == 0: continue
        # probability = stats.norm(mean, std).pdf(mean)
        probability = mean
        tmp_dest.append(dest)
        probabilities.append(probability)
    # Normalize the probabilities
    probabilities = np.array(probabilities)
    # Randomly select a destination
    selected_destination = np.random.choice(tmp_dest, p=probabilities)
    
    return selected_destination


def getTaskDuration( destination):
    if destination.startswith("toilet"):
        return random.randint(2, 4)
    else:
        return random.randint(2, MAX_TASK_TIME)
   
   
if __name__ == "__main__":  

    SCENARIO = "/home/lcastri/git/PeopleFlow/HRISim_docker/pedsim_ros/pedsim_simulator/scenarios/warehouse.xml"
    AGENTS, SCHEDULE = readScenario()
    
    for agent in AGENTS:
        AGENTS[agent]['tasks'] = {tod: {"destinations":[], "durations":[]} for tod in SCHEDULE}
        AGENTS[agent]['startTime'] = random.randint(20, SCHEDULE[constants.TOD.H1.value].duration - 30)
        AGENTS[agent]['exitTime'] = int(sum([SCHEDULE[t].duration for t in SCHEDULE if t in [e.value for e in constants.TOD if e != constants.TOD.H10 and e != constants.TOD.OFF]]) + AGENTS[agent]['startTime'])
        print(f"Agent {agent}: startTime {AGENTS[agent]['startTime']} exitTime {AGENTS[agent]['exitTime']}")
                   
        for tod in SCHEDULE:
            print(f"TOD {tod}")
            while len(AGENTS[agent]['tasks'][tod]['destinations']) < 2500:
                destination = selectDestination(tod, AGENTS[agent]['potential_dests'])
                AGENTS[agent]['tasks'][tod]['destinations'].append(destination)
                AGENTS[agent]['tasks'][tod]['durations'].append(getTaskDuration(destination))
                    
    with open('/home/lcastri/git/PeopleFlow/HRISim_docker/HRISim/peopleflow/peopleflow_manager/hardcoded/agent_task_list.pkl', 'wb') as f:
        pickle.dump(AGENTS, f)