import pickle
import random
import networkx as nx
import constants as constants

WORKING_TOP_TARGETS = [constants.WP.TARGET_1.value, constants.WP.TARGET_2.value, constants.WP.TARGET_3.value]
WORKING_BOTTOM_TARGETS = [constants.WP.TARGET_4.value, constants.WP.TARGET_5.value, constants.WP.TARGET_6.value]
LUNCH_TARGETS = [constants.WP.ENTRANCE.value, constants.WP.TARGET_7.value]

TASK_LIST = {
    constants.Task.DELIVERY.value: [],
    'LUNCH': [],
    constants.Task.CLEANING.value: [],
    }
   
if __name__ == "__main__":  

    GPATH = "/home/lcastri/git/PeopleFlow/HRISim_docker/HRISim/peopleflow/peopleflow_manager/res/warehouse/graph.pkl"
    with open(GPATH, 'rb') as f:
        G = pickle.load(f)
        G.remove_node("parking")
    CLEANING_PATH = nx.approximation.traveling_salesman_problem(G, cycle=False)

    # DELIVERY
    random_target = 2000
    for s in range(random_target):
        TASK_LIST[constants.Task.DELIVERY.value].append((random.choice(WORKING_TOP_TARGETS), bool(random.getrandbits(1))))
        TASK_LIST[constants.Task.DELIVERY.value].append((random.choice(WORKING_BOTTOM_TARGETS), bool(random.getrandbits(1))))
        
    # LUNCH
    random_target = 2000
    for s in range(random_target):
        TASK_LIST['LUNCH'].append((constants.WP.ENTRANCE.value, bool(random.getrandbits(1))))
        TASK_LIST['LUNCH'].append((constants.WP.TARGET_7.value, bool(random.getrandbits(1))))
    
    # CLEANING
    CLEANING_PATH = nx.approximation.traveling_salesman_problem(G, cycle=False)
    TASK_LIST[constants.Task.CLEANING.value] = [(p, False) for p in CLEANING_PATH]
    
    with open('/home/lcastri/git/PeopleFlow/HRISim_docker/HRISim/hrisim_plans/hardcoded/task_list.pkl', 'wb') as f:
        pickle.dump(TASK_LIST, f)
