import math
import os
import pickle
import numpy as np
import pandas as pd
from metrics.utils_2 import *
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
    return time_to_goal * (Ks + Kd * ROBOT_MAX_VEL)


def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def is_in_danger(robot_positions, static_obstacles, agents, inflation_radius):
    """
    Check if the robot is in a "dangerous" zone at each time step.

    Args:
        robot_positions (np.ndarray): Array of shape (t, 2) with robot positions.
        static_obstacles (np.ndarray): Array of shape (k, 3) with static obstacles [x, y, r].
        agents (np.ndarray): Array of shape (t, n, 2) with agent positions for all time steps.
        inflation_radius (float): Inflation radius to determine dangerous zones.

    Returns:
        np.ndarray: Array of shape (t,) where each value is 1 if in danger, 0 otherwise.
    """
    obs_proximity = np.zeros(len(robot_positions), dtype=int)

    for t, robot_pos in enumerate(robot_positions):
        # Check proximity to static obstacles
        static_distances = np.sqrt(np.sum((static_obstacles[:, :2] - robot_pos) ** 2, axis=1))
        static_radii = static_obstacles[:, 2] + inflation_radius
        if np.any(static_distances <= static_radii):
            obs_proximity[t] = 1
            continue

        # Check proximity to dynamic obstacles (agents)
        dynamic_distances = np.sqrt(np.sum((agents[t] - robot_pos) ** 2, axis=1))
        if np.any(dynamic_distances <= inflation_radius):
            obs_proximity[t] = 1

    return obs_proximity


def get_initrow(df):
    for r in range(len(df)):
        if (df.iloc[r]["G_X"] != -1000 and df.iloc[r]["G_Y"] != -1000 and df.iloc[r].notnull().all()):
            return r
        
        
tree = ET.parse('/home/lcastri/git/PeopleFlow/HRISim_docker/pedsim_ros/pedsim_simulator/scenarios/warehouse_old.xml')
root = tree.getroot()

wps = {}
obstacles = []
for waypoint in root.findall('waypoint'):
    waypoint_id = waypoint.get('id')
    x = float(waypoint.get('x'))
    y = float(waypoint.get('y'))
    r = float(waypoint.get('r'))
    wps[waypoint_id] = {'x': x, 'y': y, 'r': r}
for obstacle in root.findall("obstacle"):
    x1 = float(obstacle.get("x1"))
    y1 = float(obstacle.get("y1"))
    x2 = float(obstacle.get("x2"))
    y2 = float(obstacle.get("y2"))
    obstacles.append([(x1 + x2) / 2, (y1 + y2) / 2, 0])
obstacles = np.array(obstacles)

# Load map information
INCSV_PATH= os.path.expanduser('utilities_ws/src/RA-L/hrisim_postprocess/csv/TOD/shrunk')
OUTCSV_PATH= os.path.expanduser(f'utilities_ws/src/RA-L/hrisim_postprocess/csv/TOD/my_nonoise/')
BAGNAME= ['BL100_21102024']

static_duration = 5
dynamic_duration = 4
charging_time = 2
ROBOT_MAX_VEL = 0.5
INFLATION_RADIUS = 0.55
Ks = 100 / (static_duration * 3600)
Kd = (100 / (dynamic_duration * 3600) - Ks)/ROBOT_MAX_VEL
Kobs = 0.1
charge_rate = 100 / (charging_time * 3600)
with open('/home/lcastri/git/PeopleFlow/HRISim_docker/HRISim/peopleflow/peopleflow_manager/res/warehouse_old/graph.pkl', 'rb') as f:
    G = pickle.load(f)

# Load data
for bag in BAGNAME:
    print(f"\n### Analysing rosbag: {bag}")
    for tod in TOD:
        print(f"- time: {tod.value}")
        DF = pd.read_csv(os.path.join(INCSV_PATH, f"{bag}", tod.value, "static", f"{bag}_{tod.value}.csv"))
        n_rows = len(DF)
        
        BACs = {wp: np.zeros(n_rows) for wp in wps}

        # EXTRACTING DATA
        RX = DF['R_X'].to_numpy()  # Robot's X position
        RY = DF['R_Y'].to_numpy()  # Robot's Y position
        RB = DF['R_B']             # Robot's battery
        
        # Dynamically extract agent positions
        agent_columns = [col for col in DF.columns if '_X' in col and 'a' in col]
        num_agents = 20

        # Initialize the agents array (t, n, 2)
        agents = np.full((len(DF), num_agents, 2), np.nan)  # Fill with NaN for missing values

        for i in range(1, num_agents + 1):
            # Fill agent positions across all time steps
            agents[:, i - 1, 0] = DF[f'a{i}_X'].to_numpy()  # X-coordinates
            agents[:, i - 1, 1] = DF[f'a{i}_Y'].to_numpy()  # Y-coordinates
            
        OBS = is_in_danger(np.stack((RX, RY), axis=1), obstacles, agents, INFLATION_RADIUS)
        for wp in wps:
            BACs[wp][0] = DF.iloc[0][f'{wp}_BAC']
               
        for wp in wps:
            BACs[wp][1:] = np.maximum(0, RB[1:] - get_battery_consumption(wp, WP.CHARGING_STATION.value) - Kobs * OBS[1:])

        # Add computed values to DF
        DF[f'OBS'] = OBS
        for wp, bac_array in BACs.items():
            DF[f'{wp}_BAC'] = bac_array
        
        # Create output directory if it doesn't exist
        out_path = os.path.join(OUTCSV_PATH, f'{bag}', f'{tod.value}')
        os.makedirs(out_path, exist_ok=True)
        
        # Save the updated DF
        DF.to_csv(os.path.join(out_path, f"{bag}_{tod.value}.csv"), index=False)

        # Save WP-specific DataFrames
        for wp in WP:
            if wp in [WP.PARKING, WP.CHARGING_STATION]: continue 
        
            WPDF = pd.read_csv(os.path.join(INCSV_PATH, f"{bag}", tod.value, "static", f"{bag}_{tod.value}_{wp.value}.csv"))
            WPDF["BAC"] = BACs[wp.value]
            WPDF["OBS"] = OBS
            
            WPDF.to_csv(os.path.join(out_path, f"{bag}_{tod.value}_{wp.value}.csv"), index=False)