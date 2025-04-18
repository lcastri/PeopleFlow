import copy
import random
import networkx as nx
from geometry_msgs.msg import Point
import numpy as np

DEFAULT_VALUE = -1000
CONSECUTIVE_STUCK_THRESHOLD = 2

class Agent:
    def __init__(self, id, schedule, graph, allowTask, maxTaskTime) -> None:
        self.id = id
        self.schedule = schedule
        self.G = graph
        self.allowTask = allowTask
        self.maxTaskTime = maxTaskTime
        self.x = None
        self.y = None
        self.path = []
        self.original_path = []
        self.pastDest = None
        self.pastFinalDest = None
        self.currDest = None
        self.nextDest = None
        self.nextDestPos = None
        self.nextDestRadius = None
        self.startingTime = None
        self.exitTime = None
        self.atWork = False
        self.isQuitting = False
        self.isStuck = False
        self.taskDuration = None
        self.nConsecutiveStuck = 0
        self.closestWPidx = 0
          
               
    @property
    def closestWP(self):
        if self.x is None or self.y is None:
            return None

        pos = nx.get_node_attributes(self.G, 'pos')

        # Calculate distances from the agent's position to all waypoints
        distances = [(wp, ((self.x - pos[wp][0]) ** 2 + (self.y - pos[wp][1]) ** 2) ** 0.5) for wp in self.G.nodes]
        
        # Sort by distance
        sorted_distances = sorted(distances, key=lambda x: x[1])

        # Select the closest waypoint by default
        if not self.isStuck:
            return sorted_distances[0][0]

        # If the agent is stuck, attempt to use the second closest waypoint
        else:
            if self.nConsecutiveStuck <= CONSECUTIVE_STUCK_THRESHOLD:
                return sorted_distances[0][0]
            elif self.nConsecutiveStuck > CONSECUTIVE_STUCK_THRESHOLD and len(sorted_distances) > 1:
                self.closestWPidx += 1
                self.closestWPidx = min(len(sorted_distances) - 1, self.closestWPidx)
                return sorted_distances[self.closestWPidx][0]

        # Fallback in case there’s only one waypoint
        return sorted_distances[0][0]
  
    @property
    def isFree(self):
        return len(self.path) == 0
        
    @property
    def nextWP(self):
        destname = self.path.pop(0)
        pos = nx.get_node_attributes(self.G, 'pos')
        
        dest = Point()
        dest.x = pos[destname][0]
        dest.y = pos[destname][1]
        
        self.pastDest = self.original_path[max(0, self.original_path.index(destname) - 1)]
        self.currDest = self.original_path[self.original_path.index(destname)]
        self.nextDest = self.original_path[min(len(self.original_path) - 1, self.original_path.index(destname) + 1)]
        self.nextDestPos = [dest.x, dest.y, 0]
        
        return destname, dest
    
    @property
    def finalDest(self):
        return self.original_path[-1] if self.original_path else None
    
       
    def to_dict(self):
        """Serialize the Agent object to a dictionary, replacing None values with defaults."""
        return {
            'id': self.id,
            'x': self.x if self.x is not None else DEFAULT_VALUE,
            'y': self.y if self.y is not None else DEFAULT_VALUE,
            'path': self.path if self.path is not None else [],
            'original_path': self.original_path if self.original_path is not None else [],
            'pastDest': self.pastDest if self.pastDest is not None else '',
            'pastFinalDest': self.pastFinalDest if self.pastFinalDest is not None else '',
            'currDest': self.currDest if self.currDest is not None else '',
            'nextDest': self.nextDest if self.nextDest is not None else '',
            'nextDestPos': self.nextDestPos if self.nextDestPos is not None else [DEFAULT_VALUE, DEFAULT_VALUE, DEFAULT_VALUE],
            'nextDestRadius': self.nextDestRadius if self.nextDestRadius is not None else DEFAULT_VALUE,
            'startingTime': self.startingTime if self.startingTime is not None else DEFAULT_VALUE,
            'exitTime': self.exitTime if self.exitTime is not None else DEFAULT_VALUE,
            'atWork': self.atWork,
            'isQuitting': self.isQuitting,
            'isStuck': self.isStuck,
            'taskDuration': self.taskDuration if self.taskDuration is not None else DEFAULT_VALUE
        }

    @classmethod
    def from_dict(cls, data, schedule, graph, allowTask, maxTaskTime):
        """Deserialize a dictionary to an Agent object."""
        agent = cls(data['id'], schedule, graph, allowTask, maxTaskTime)
        agent.x = data['x'] if data['x'] != DEFAULT_VALUE else None
        agent.y = data['y'] if data['y'] != DEFAULT_VALUE else None
        agent.path = data['path']
        agent.original_path = data['original_path'] if data['original_path'] != [] else None
        agent.pastDest = data['pastDest'] if data['pastDest'] != '' else None
        agent.pastFinalDest = data['pastFinalDest'] if data['pastFinalDest'] != '' else None
        agent.currDest = data['currDest'] if data['currDest'] != '' else None
        agent.nextDest = data['nextDest'] if data['nextDest'] != '' else None
        agent.nextDestPos = data['nextDestPos'] if data['nextDestPos'] != [DEFAULT_VALUE, DEFAULT_VALUE] else None
        agent.nextDestRadius = data['nextDestRadius'] if data['nextDestRadius'] != DEFAULT_VALUE else None
        agent.startingTime = data['startingTime'] if data['startingTime'] != DEFAULT_VALUE else None
        agent.exitTime = data['exitTime'] if data['exitTime'] != DEFAULT_VALUE else None
        agent.atWork = data['atWork']
        agent.isQuitting = data['isQuitting']
        agent.isStuck = data['isStuck']
        agent.taskDuration = data['taskDuration'] if data['taskDuration'] != DEFAULT_VALUE else None
        return agent
    
    def heuristic(self, a, b):
        pos = nx.get_node_attributes(self.G, 'pos')
        (x1, y1) = pos[a]
        (x2, y2) = pos[b]
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    
    def selectDestination(self, selected_time, potential_dests):
        destinations = self.schedule[selected_time]['dests']
        #! I am commenting this line to allow the agents to select the same destination
        # if self.pastFinalDest is not None and self.pastFinalDest != 'delivery_point': potential_dests.remove(self.pastFinalDest)
        
        # Generate probabilities
        tmp_dest = []
        probabilities = []
        for dest in potential_dests:
            mean = destinations[dest]['mean']
            std = destinations[dest]['std']
            
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
    
    
    def setTask(self, destination, duration = None, isStuck = False):
        if isStuck: 
            self.nConsecutiveStuck += 1
        else: 
            self.nConsecutiveStuck = 0
            self.closestWPidx = 0
        
        self.pastFinalDest = self.finalDest if self.finalDest else None
        self.path = nx.astar_path(self.G, self.closestWP, destination, heuristic=self.heuristic, weight='weight')
        self.original_path = copy.deepcopy(self.path)
            
        self.taskDuration = {wp: 0 for wp in self.path}
        if duration is not None:
            self.taskDuration[self.path[-1]] = duration
        
    
    def getTaskDuration(self):
        if self.allowTask:
            if self.finalDest.startswith("toilet"):
                return random.randint(2, 4)
            else:
                return random.randint(2, self.maxTaskTime)
        else:
            return 0
