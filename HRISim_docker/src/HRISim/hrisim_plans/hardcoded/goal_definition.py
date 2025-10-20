import pickle
import json
import random
import networkx as nx
import constants as constants

WORKING_TOP_TARGETS = [constants.WP.TARGET_2.value]
WORKING_BOTTOM_TARGETS = [constants.WP.TARGET_5.value]
LUNCH_TARGETS = [constants.WP.ENTRANCE.value, constants.WP.TARGET_7.value]
# WORKING_TOP_TARGETS = [constants.WP.TARGET_1.value, constants.WP.TARGET_2.value, constants.WP.TARGET_3.value]
# WORKING_BOTTOM_TARGETS = [constants.WP.TARGET_4.value, constants.WP.TARGET_5.value, constants.WP.TARGET_6.value]
# LUNCH_TARGETS = [constants.WP.ENTRANCE.value, constants.WP.TARGET_7.value]

TASK_LIST = {tod.value: [] for tod in constants.TOD}
   
if __name__ == "__main__":  

    GPATH = "/home/lcastri/git/PeopleFlow/HRISim_docker/HRISim/peopleflow/peopleflow_manager/res/warehouse/graph.pkl"
    with open(GPATH, 'rb') as f:
        G = pickle.load(f)
        G.remove_node("parking")
    # CLEANING
    CLEANING_PATH = nx.approximation.traveling_salesman_problem(G, cycle=False)
    TASK_LIST[constants.TOD.OFF.value] = CLEANING_PATH

    random_target = 200
    whereIam = 'T'
    for tod in constants.TOD:
        if tod in [constants.TOD.H1, constants.TOD.H2, constants.TOD.H3, constants.TOD.H4,
                   constants.TOD.H5, constants.TOD.H7, constants.TOD.H8, constants.TOD.H9, constants.TOD.H10]:
            # DELIVERY
            for s in range(random_target):
                if whereIam == 'T':
                    TASK_LIST[tod.value].append(random.choice(WORKING_BOTTOM_TARGETS))
                    whereIam = 'B'
                else:
                    TASK_LIST[tod.value].append(random.choice(WORKING_TOP_TARGETS))
                    whereIam = 'T'
                
        elif tod == constants.TOD.H6:            
            # LUNCH
            for s in range(random_target):
                if whereIam == 'T':
                    TASK_LIST[tod.value].append(constants.WP.TARGET_7.value)
                    whereIam = 'B'
                else:
                    TASK_LIST[tod.value].append(constants.WP.ENTRANCE.value)
                    whereIam = 'T'
        
    # LUNCH
    random_target = 2000
    for s in range(random_target):
        TASK_LIST['LUNCH'].append(constants.WP.ENTRANCE.value)
        TASK_LIST['LUNCH'].append(constants.WP.TARGET_7.value)
    
    # CLEANING
    CLEANING_PATH = nx.approximation.traveling_salesman_problem(G, cycle=False)
    TASK_LIST[constants.Task.CLEANING.value] = [p for p in CLEANING_PATH]
    
    with open('/home/lcastri/git/PeopleFlow/HRISim_docker/HRISim/hrisim_plans/hardcoded/task_list.json', 'w') as f:
        json.dump(TASK_LIST, f)
