import math
import os
import pickle
import numpy as np
import pandas as pd
from utils import *
import networkx as nx
import xml.etree.ElementTree as ET


def get_battery_consumption(wp_origin, wp_dest):
    pos = nx.get_node_attributes(G, 'pos')
    path = nx.astar_path(G, wp_origin, wp_dest, heuristic=heuristic, weight='weight')
    distanceToCharger = 0
    for wp_idx in range(1, len(path)):
        (x1, y1) = pos[path[wp_idx-1]]
        (x2, y2) = pos[path[wp_idx]]
        distanceToCharger += math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    time_to_goal = math.ceil(distanceToCharger/ROBOT_MAX_VEL)
    return time_to_goal * (static_consumption + K * ROBOT_MAX_VEL)


def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def get_initrow(df):
    for r in range(len(df)):
        if (df.iloc[r]["G_X"] != -1000 and df.iloc[r]["G_Y"] != -1000 and df.iloc[r].notnull().all()):
            return r
        
        
tree = ET.parse('/home/lcastri/git/PeopleFlow/HRISim_docker/pedsim_ros/pedsim_simulator/scenarios/warehouse.xml')
root = tree.getroot()

wps = {}
for waypoint in root.findall('waypoint'):
    waypoint_id = waypoint.get('id')
    x = float(waypoint.get('x'))
    y = float(waypoint.get('y'))
    r = float(waypoint.get('r'))
    wps[waypoint_id] = {'x': x, 'y': y, 'r': r}


# Load map information
INCSV_PATH= os.path.expanduser('utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/shrunk')
OUTCSV_PATH= os.path.expanduser(f'utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/my_nonoise/')
BAGNAME= ['noncausal-03012025']

static_duration = 5
dynamic_duration = 4
charging_time = 2
ROBOT_MAX_VEL = 0.5
static_consumption = 100 / (static_duration * 3600)
K = (100 / (dynamic_duration * 3600) - static_consumption)/ROBOT_MAX_VEL
charge_rate = 100 / (charging_time * 3600)
with open('/home/lcastri/git/PeopleFlow/HRISim_docker/HRISim/peopleflow/peopleflow_manager/res/warehouse/graph.pkl', 'rb') as f:
    G = pickle.load(f)

# Load data
for bag in BAGNAME:
    print(f"\n### Analysing rosbag: {bag}")
    for tod in TOD:
        print(f"- time: {tod.value}")
        tmp_bag = bag.replace('_', '-')
        DF = pd.read_csv(os.path.join(INCSV_PATH, f"{bag}", tod.value, "static", f"{tmp_bag}_{tod.value}.csv"))
        n_rows = len(DF)
        
        
        BACs = {wp: np.zeros(n_rows) for wp in wps}

        # Initial battery levels
        RB = DF['R_B']
        for wp in wps:
            BACs[wp][0] = DF.iloc[0][f'{wp}_BAC']
        
               
        for wp in wps:
            BACs[wp][1:] = np.maximum(0, RB[1:] - get_battery_consumption(wp, 'charging-station'))

        # Add computed values to DF
        for wp, bac_array in BACs.items():
            DF[f'{wp}_BAC'] = bac_array
        
        # Create output directory if it doesn't exist
        out_path = os.path.join(OUTCSV_PATH, f'{bag}', f'{tod.value}')
        os.makedirs(out_path, exist_ok=True)
        
        # Save the updated DF
        DF.to_csv(os.path.join(out_path, f"{tmp_bag}_{tod.value}.csv"), index=False)

        # Save WP-specific DataFrames
        for wp in WP:
            if wp in [WP.PARKING, WP.CHARGING_STATION]: continue 
        
            WPDF = pd.read_csv(os.path.join(INCSV_PATH, f"{bag}", tod.value, "static", f"{tmp_bag}_{tod.value}_{wp.value}.csv"))
            WPDF["BAC"] = BACs[wp.value]
            
            WPDF.to_csv(os.path.join(out_path, f"{tmp_bag}_{tod.value}_{wp.value}.csv"), index=False)