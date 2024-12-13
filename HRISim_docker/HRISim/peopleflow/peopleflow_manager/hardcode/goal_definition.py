import pickle
import random
import numpy as np
from scipy import stats
import constants as constants
import xml.etree.ElementTree as ET

MAX_TASK_TIME = 15

SHELFS = [constants.WP.SHELF1.value, 
          constants.WP.SHELF2.value, 
          constants.WP.SHELF3.value, 
          constants.WP.SHELF4.value, 
          constants.WP.SHELF5.value, 
          constants.WP.SHELF6.value]

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
            tmp.dests[dest_name] = {'mean': tmp.duration * float(adddest.get('p')), 'std': float(adddest.get('std'))}
        schedule[tmp.name] = tmp
    
    
    # Parse agents
    agents = {}
    for agent in root.findall('agent'):
        agent_id = len(agents) + 1
        agent_type = agent.get('type')
        if agent_type == "2": continue

        agents[agent_id] = {}
        agents[agent_id]['potential_dests'] = []
        for wp in agent.findall('addwaypoint'):
            agents[agent_id]['potential_dests'].append(wp.get('id'))
        
    # return wps, agents, schedule
    return agents, schedule


def selectDestination(selected_time, agent):
    destinations = SCHEDULE[selected_time].dests
    
    # Generate probabilities
    probabilities = []
    for dest in AGENTS[agent]['potential_dests']:
        mean = destinations[dest]['mean']
        std = destinations[dest]['std']
        
        if std == 0:
            probability = 0.0
        else:
            probability = stats.norm(mean, std).pdf(mean)
        probabilities.append(probability)
    # Normalize the probabilities
    probabilities = np.array(probabilities)
    normalized_probabilities = probabilities / probabilities.sum()
    # Randomly select a destination
    selected_destination = np.random.choice(AGENTS[agent]['potential_dests'], p=normalized_probabilities)
    
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
        AGENTS[agent]['startTime'] = random.randint(60, SCHEDULE['starting'].duration - 10)
        AGENTS[agent]['exitTime'] = int(sum([SCHEDULE[t].duration for t in SCHEDULE if t in ['starting','morning','lunch','afternoon']]) + AGENTS[agent]['startTime'])
        print(f"Agent {agent}: startTime {AGENTS[agent]['startTime']} exitTime {AGENTS[agent]['exitTime']}")
                   
        for tod in SCHEDULE:
            print(f"TOD {tod}")
            while len(AGENTS[agent]['tasks'][tod]['destinations']) < 2500:
                destination = selectDestination(tod, agent)
                AGENTS[agent]['tasks'][tod]['destinations'].append(destination)
                AGENTS[agent]['tasks'][tod]['durations'].append(getTaskDuration(destination))
                    
    with open('agent_task_list.pkl', 'wb') as f:
        pickle.dump(AGENTS, f)