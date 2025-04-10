import math
import os
import pickle
import numpy as np
import pandas as pd
from metrics.utils import *
import networkx as nx
import xml.etree.ElementTree as ET


def get_battery_consumption(wp_origin, wp_dest, load):
    pos = nx.get_node_attributes(G, 'pos')
    path = nx.astar_path(G, wp_origin, wp_dest, heuristic=heuristic, weight='weight')
    distanceToCharger = 0
    for wp_idx in range(1, len(path)):
        (x1, y1) = pos[path[wp_idx-1]]
        (x2, y2) = pos[path[wp_idx]]
        distanceToCharger += math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    if load: 
        time_to_goal = math.ceil(distanceToCharger/LOAD_ROBOT_MAX_VEL)
        return time_to_goal * (K_l_s + K_l_d * LOAD_ROBOT_MAX_VEL)
    else:
        time_to_goal = math.ceil(distanceToCharger/NOLOAD_ROBOT_MAX_VEL)
        return time_to_goal * (K_nl_s + K_nl_d * NOLOAD_ROBOT_MAX_VEL)
    
    
def get_distance(wp_origin, wp_dest):
    pos = nx.get_node_attributes(G, 'pos')
    path = nx.astar_path(G, wp_origin, wp_dest, heuristic=heuristic, weight='weight')
    distanceToCharger = 0
    for wp_idx in range(1, len(path)):
        (x1, y1) = pos[path[wp_idx-1]]
        (x2, y2) = pos[path[wp_idx]]
        distanceToCharger += math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    return distanceToCharger

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
# BAGNAME= ['test-obs-01', 'test-obs-02', 'test-obs-04', 'test-obs-05']
BAGNAME= ['noncausal-28022025']

static_duration = 5
dynamic_duration = 4
charging_time = 2
LOAD_FACTOR = 5
NOLOAD_ROBOT_MAX_VEL = 0.75
LOAD_ROBOT_MAX_VEL = 0.25
K_nl_s = 100 / (static_duration * 3600)
K_nl_d = (100 / (dynamic_duration * 3600) - K_nl_s)/(NOLOAD_ROBOT_MAX_VEL)
K_l_s = K_nl_s * LOAD_FACTOR
K_l_d = K_nl_d * LOAD_FACTOR
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
        
        # EC = np.full_like(DF['R_V'], 0)

        # dt = np.diff(DF['pf_elapsed_time']).tolist()
        # dt.insert(0, 5.0)
        # dt = np.array(dt)
        # EC = np.where(DF["OBS"] == 0, 
        #               dt * (K_nl_s + K_nl_d * DF['R_V']), 
        #               dt * (K_l_s + K_l_d * DF['R_V']))
        # DF['R_B'] = DF['R_B'].replace(0, np.nan).bfill()
        EC = np.diff(DF['R_B'].values)
        ec_list = list(EC)
        ec_list.insert(0, ec_list[0])
        EC = np.abs(np.array(ec_list))
        DF['EC'] = EC
        # DF['EC'].replace(0, min(EC), inplace=True)
        # RB = DF['R_B'][0] - pd.Series(EC.cumsum()).shift(1).fillna(0)       
               
        # ELTs = {wp: np.zeros(n_rows) for wp in wps}
        # for wp in wps:
        #     for i in range(n_rows):
        #         if DF['L'][i] == 0:
        #             ELTs[wp][i] = np.maximum(0, RB[i] - get_battery_consumption(wp, 'charging-station', False))
        #         else:
        #             ELTs[wp][i] = np.maximum(0, RB[i] - get_battery_consumption(wp, 'charging-station', True))
        # L = DF['OBS'].to_numpy()
        # RB_array = np.array(RB)

        # # Precompute battery consumption values for both cases
        # battery_consumption_false = {wp: get_battery_consumption(wp, 'charging-station', False) for wp in wps}
        # battery_consumption_true = {wp: get_battery_consumption(wp, 'charging-station', True) for wp in wps}

        # # Use NumPy operations to compute ELTs
        # ELTs = {
        #     wp: np.maximum(0, RB_array - np.where(L == 0, battery_consumption_false[wp], battery_consumption_true[wp]))
        #     for wp in wps
        # }

        # Add computed values to DF
        # DF['R_B'] = RB
        # DF['EC'] = EC
        # for wp, elt_array in ELTs.items():
        #     DF[f'{wp}_ELT'] = elt_array
        
        # Create output directory if it doesn't exist
        out_path = os.path.join(OUTCSV_PATH, f'{bag}', f'{tod.value}')
        os.makedirs(out_path, exist_ok=True)
        
        # Save the updated DF
        DF.to_csv(os.path.join(out_path, f"{tmp_bag}_{tod.value}.csv"), index=False)

        # Save WP-specific DataFrames
        for wp in WP:
            if wp in [WP.PARKING, WP.CHARGING_STATION]: continue 
        
            WPDF = pd.read_csv(os.path.join(INCSV_PATH, f"{bag}", tod.value, "static", f"{tmp_bag}_{tod.value}_{wp.value}.csv"))
            # WPDF["ELT"] = ELTs[wp.value]
            WPDF['EC'] = EC

            WPDF.to_csv(os.path.join(out_path, f"{tmp_bag}_{tod.value}_{wp.value}.csv"), index=False)