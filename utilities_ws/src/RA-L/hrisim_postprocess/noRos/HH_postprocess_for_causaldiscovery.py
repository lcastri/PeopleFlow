import math
import os
import pickle
import numpy as np
import pandas as pd
from metrics.utils_for_causaldiscovery import *
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


def get_initrow(df):
    for r in range(len(df)):
        if (df.iloc[r]["G_X"] != -1000 and df.iloc[r]["G_Y"] != -1000 and df.iloc[r].notnull().all()):
            return r
        
        
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
ADD_NOISE = True
INCSV_PATH= os.path.expanduser('utilities_ws/src/RA-L/hrisim_postprocess/csv/TOD/shrunk')
OUTCSV_PATH= os.path.expanduser(f'utilities_ws/src/RA-L/hrisim_postprocess/csv/TOD/my{"_noised" if ADD_NOISE else "_nonoise"}/')
BAGNAME= ['BL100_21102024']

static_duration = 5
dynamic_duration = 4
charging_time = 2
ROBOT_MAX_VEL = 0.5
INFLATION_RADIUS = 0.55
ALPHA = 0.8
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
        r = get_initrow(DF)
        DF = DF[r:]
        n_rows = len(DF)
        
        
        # Initialize arrays for RV, RB, and BACs for each waypoint
        RX = DF['R_X'].to_numpy()  # Robot's X position
        RY = DF['R_Y'].to_numpy()  # Robot's Y position
        RV = DF['R_V'].values 
        if ADD_NOISE: RV += np.random.normal(0, 0.11, n_rows)
        
        RB = np.zeros(n_rows)
        BACs = {wp: np.zeros(n_rows) for wp in wps}

        # Initial battery levels
        RB[0] = DF.iloc[0]['R_B']
        for wp in wps:
            BACs[wp][0] = DF.iloc[0][f'{wp}_BAC']
        
        # Vectorize the elapsed time differences
        elapsed_time_diff = DF['ros_time'].diff().fillna(0).values
        
        # Vectorize static consumption and K*velocity calculation
        dynamic_consumption = Ks + Kd * pd.Series(RV).shift(periods = 1, fill_value=0).values
        energy_used = elapsed_time_diff * dynamic_consumption
        energy_charged = elapsed_time_diff * charge_rate
        
        # Compute battery level `RB` for all rows at once
        charge_status = DF['B_S'].values == 1  # boolean array for charging status
        EC = np.where(charge_status, energy_charged, -energy_used)
        
        RB[1:] = RB[0] + np.cumsum(EC[1:])
        if ADD_NOISE: RB += np.random.uniform(-0.051, 0.051, n_rows)
        RB = np.clip(RB, 0, 100)  # Ensure battery level remains between 0 and 100
        
        # Compute BACs for each waypoint
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
            # Compute base BAC values
            base_bac = RB[1:] - get_battery_consumption(wp, WP.CHARGING_STATION.value) - Kobs * OBS[1:]
            smoothed_bac = np.zeros_like(base_bac)
            smoothed_bac[0] = BACs[wp][0]  # Initialize with the first value
            
            # Apply exponential smoothing
            for t in range(1, len(base_bac)):
                smoothed_bac[t] = ALPHA * base_bac[t] + (1 - ALPHA) * smoothed_bac[t - 1] + (1 if ADD_NOISE else 0) * np.random.uniform(-0.4, 0.4)

            BACs[wp][1:] = smoothed_bac

        # Add computed values to DF
        DF['R_V'] = RV
        DF['R_B'] = RB
        for wp, bac_array in BACs.items():
            DF[f'{wp}_BAC'] = bac_array
        DF[f'OBS'] = OBS

        
        # Create output directory if it doesn't exist
        out_path = os.path.join(OUTCSV_PATH, f'{bag}', f'{tod.value}')
        os.makedirs(out_path, exist_ok=True)
        
        # Save the updated DF
        DF.to_csv(os.path.join(out_path, f"{bag}_{tod.value}.csv"), index=False)

        # Save WP-specific DataFrames
        general_columns_name = ['pf_elapsed_time', 'TOD', 'R_V', 'R_B', 'B_S']
        for wp_name, wp_id in WPS.items():
            if wp_name in [WP.PARKING.value, WP.CHARGING_STATION.value]: continue 
                    
            wp_df = DF[general_columns_name + [f"{wp_name}_PD", f"{wp_name}_BAC", "OBS"]]
            wp_df = wp_df.rename(columns={f"{wp_name}_PD": "PD", f"{wp_name}_BAC": "ELT", "B_S": "C_S"})
            wp_df["WP"] = wp_id
            
            wp_df.to_csv(os.path.join(out_path, f"{bag}_{tod.value}_{wp_name}.csv"), index=False)