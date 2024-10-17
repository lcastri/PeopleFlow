import copy
import random
import networkx as nx
from scipy import stats
from geometry_msgs.msg import Point
import numpy as np
import rospy

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
        
    @property
    def closestWP(self):
        tmp_dist = []
        if self.x is not None and self.y is not None:
            pos = nx.get_node_attributes(self.G, 'pos')
            for wp in self.G.nodes:
                d = ((self.x - pos[wp][0]) ** 2 + (self.y - pos[wp][1]) ** 2) ** 0.5
                tmp_dist.append(d)            
            return list(self.G.nodes)[np.argmin(tmp_dist)]
        return None
    
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
        self.nextDestPos = dest
        
        return destname, dest
    
    @property
    def finalDest(self):
        return self.original_path[-1] if self.original_path else None
    
    def heuristic(self, a, b):
        pos = nx.get_node_attributes(self.G, 'pos')
        (x1, y1) = pos[a]
        (x2, y2) = pos[b]
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    
    def selectDestination(self, selected_time, potential_dests):
        destinations = self.schedule[selected_time]['dests']
        if self.pastFinalDest is not None and self.pastFinalDest != 'delivery_point': potential_dests.remove(self.pastFinalDest)
        
        # Generate probabilities
        probabilities = []
        for dest in potential_dests:
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
        selected_destination = np.random.choice(potential_dests, p=normalized_probabilities)
        
        return selected_destination
    
    
    def setTask(self, destination, duration = None):
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
