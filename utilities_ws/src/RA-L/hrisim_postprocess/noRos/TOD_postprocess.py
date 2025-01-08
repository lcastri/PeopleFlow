import math
import os
import pickle
import numpy as np
import pandas as pd
from metrics.utils import *
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
ADD_NOISE = True
INCSV_PATH= os.path.expanduser('utilities_ws/src/RA-L/hrisim_postprocess/csv/TOD/shrunk')
OUTCSV_PATH= os.path.expanduser(f'utilities_ws/src/RA-L/hrisim_postprocess/csv/TOD/my{"_noised" if ADD_NOISE else "_nonoise"}/')
# BAGNAME= ['BL100_21102024']
BAGNAME= ['BL100_21102024', 'BL75_29102024', 'BL50_22102024', 'BL25_28102024']

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
        DF = pd.read_csv(os.path.join(INCSV_PATH, f"{bag}", "noRT", tod.value, "static", f"{bag}_{tod.value}.csv"))
        r = get_initrow(DF)
        DF = DF[r:]
        n_rows = len(DF)
        
        if tod in [TOD.STARTING, TOD.MORNING, TOD.LUNCH]:
            A = np.zeros_like(DF['T'].values)
        elif tod in [TOD.AFTERNOON, TOD.QUITTING]:
            A = np.ones_like(DF['T'].values)
        elif tod in [TOD.OFF]:
            A = 2*np.ones_like(DF['T'].values)
        
        isTaskActive = DF['G_X'].diff().fillna(0).values + DF['G_Y'].diff().fillna(0).values
        T = np.where((isTaskActive == 0) & (DF['B_S'].values == 0), 1, 0)
        T = pd.Series(T).shift(periods = -1, fill_value=0).values
        
        # Initialize arrays for RV, RB, and BACs for each waypoint
        DG = np.sqrt((DF['R_X'].values - DF['G_X'].values)**2 + (DF['R_Y'].values - DF['G_Y'].values)**2)
        DG = pd.Series(DG).shift(periods = 1, fill_value=0).values
        RV = DF['R_V'].values 
        if ADD_NOISE: RV += np.random.normal(0, 0.1, n_rows)
        
        RB = np.zeros(n_rows)
        BACs = {wp: np.zeros(n_rows) for wp in wps}

        # Initial battery levels
        RB[0] = DF.iloc[0]['R_B']
        for wp in wps:
            BACs[wp][0] = DF.iloc[0][f'{wp}_BAC']
        
        # Vectorize the elapsed time differences
        elapsed_time_diff = DF['ros_time'].diff().fillna(0).values
        
        # Vectorize static consumption and K*velocity calculation
        dynamic_consumption = static_consumption + K * pd.Series(RV).shift(periods = 1, fill_value=0).values
        energy_used = elapsed_time_diff * dynamic_consumption
        energy_charged = elapsed_time_diff * charge_rate
        
        # Compute battery level `RB` for all rows at once
        charge_status = DF['B_S'].values == 1  # boolean array for charging status
        EC = np.where(charge_status, energy_charged, -energy_used)
        
        RB[1:] = RB[0] + np.cumsum(EC[1:])
        if ADD_NOISE: RB += np.random.uniform(-0.052, 0.052, n_rows)
        RB = np.clip(RB, 0, 100)  # Ensure battery level remains between 0 and 100
        
        # Compute BACs for each waypoint
        R_CLOSEST_WP = np.zeros(n_rows, dtype=object)
        for i in range(n_rows):
            x, y = DF.loc[i+r, ['R_X', 'R_Y']]
            closest_wp = min(wps, key=lambda wp: ((x - wps[wp]['x'])**2 + (y - wps[wp]['y'])**2))
            R_CLOSEST_WP[i] = closest_wp
        for wp in wps:
            for i in range(len(R_CLOSEST_WP) - 1):
                BACs[wp][1:] = np.maximum(0, RB[1:] - get_battery_consumption(R_CLOSEST_WP[i], wp) - get_battery_consumption(wp, 'charging_station'))
            if ADD_NOISE: BACs[wp] += np.random.uniform(-0.04, 0.04, n_rows)

        # Add computed values to DF
        DF['R_V'] = RV
        DF['R_B'] = RB
        DF["A"] = A
        DF["T"] = T
        for wp, bac_array in BACs.items():
            DF[f'{wp}_BAC'] = bac_array
        
        # Create output directory if it doesn't exist
        out_path = os.path.join(OUTCSV_PATH, f'{bag}', f'{tod.value}')
        os.makedirs(out_path, exist_ok=True)
        
        # Save the updated DF
        DF.to_csv(os.path.join(out_path, f"{bag}_{tod.value}.csv"), index=False)

        # Save WP-specific DataFrames
        general_columns_name = ['pf_elapsed_time', 'TOD', 'T', 'A', 'R_V', 'T_R', 'R_B', 'B_S']
        for wp_name, wp_id in WPS.items():
            if wp_name in ['parking', 'charging_station']:
                continue 
        
            wp_df = DF[general_columns_name + [f"{wp_name}_PD", f"{wp_name}_BAC"]]
            wp_df = wp_df.rename(columns={f"{wp_name}_PD": "PD", f"{wp_name}_BAC": "BAC"})
            wp_df["WP"] = wp_id
            
            wp_df.to_csv(os.path.join(out_path, f"{bag}_{tod.value}_{wp_name}.csv"), index=False)