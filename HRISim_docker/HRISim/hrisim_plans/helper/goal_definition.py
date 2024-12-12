import pickle
import random
import networkx as nx
import constants as constants

SHELFS = [constants.WP.SHELF1, constants.WP.SHELF2, constants.WP.SHELF3, constants.WP.SHELF4, constants.WP.SHELF5, constants.WP.SHELF6]

TASK_LIST = {
    constants.Task.DELIVERY: [],
    constants.Task.INVENTORY: [],
    constants.Task.CLEANING: [],
    }
   
if __name__ == "__main__":  

    GPATH = "/home/lcastri/git/PeopleFlow/HRISim_docker/HRISim/peopleflow/peopleflow_manager/res/warehouse/graph.pkl"
    with open(GPATH, 'rb') as f:
        G = pickle.load(f)
        G.remove_node("parking")
    CLEANING_PATH = nx.approximation.traveling_salesman_problem(G, cycle=False)

    # DELIVERY
    random_shelfs = random.choices(SHELFS, k=1000)
    for s in random_shelfs:
        TASK_LIST[constants.Task.DELIVERY].append(s)
        TASK_LIST[constants.Task.DELIVERY].append(constants.WP.DELIVERY_POINT)
        
    # INVENTORY
    random_shelfs = random.choices(SHELFS, k=1000)
    previous_shelf = None
    filtered_shelfs = []

    for shelf in random_shelfs:
        if shelf != previous_shelf:
            filtered_shelfs.append(shelf)
            previous_shelf = shelf
        else:
            # Choose a different shelf if the current one matches the previous
            alternative_shelfs = [s for s in SHELFS if s != previous_shelf]
            new_shelf = random.choice(alternative_shelfs)
            filtered_shelfs.append(new_shelf)
            previous_shelf = new_shelf

    for s in filtered_shelfs:
        TASK_LIST[constants.Task.INVENTORY].append(s)
        
    # CLEANING
    CLEANING_PATH = nx.approximation.traveling_salesman_problem(G, cycle=False)
    TASK_LIST[constants.Task.CLEANING] = CLEANING_PATH
    
    with open('task_list.pkl', 'wb') as f:
        pickle.dump(TASK_LIST, f)
