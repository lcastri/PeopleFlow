import json
import math
import os
import numpy as np
import pandas as pd
from utils import *
import xml.etree.ElementTree as ET


def readScenario():
    # Load and parse the XML file
    tree = ET.parse('/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/scenarios/' + SCENARIO + '.xml')
    root = tree.getroot()
    
    tmp = {}
    for waypoint in root.findall('waypoint'):
        waypoint_id = waypoint.get('id')
        x = float(waypoint.get('x'))
        y = float(waypoint.get('y'))
        r = float(waypoint.get('r'))
        tmp[waypoint_id] = {'x': x, 'y': y, 'r': r}
    return tmp
        

def get_initrow(df):
    for r in range(len(df)):
        if (df.iloc[r]["G_X"] != -1000 and df.iloc[r]["G_Y"] != -1000 and df.iloc[r].notnull().all()):
            return r
        
def compute_min_h_distance(df):
    robot_coords = df[['R_X', 'R_Y']].to_numpy()
    min_distance_to_humans = []
    for col in df.columns:
        if col.startswith('a') and col.endswith('_X'):
            human_coords = df[[col, col.replace('_X', '_Y')]].to_numpy()
            min_distance_to_humans.append(np.min(np.sqrt(np.sum((robot_coords - human_coords)**2, axis=1))))
    return np.min(min_distance_to_humans)

def compute_stalled_time(df):
    # Filter the data to only consider rows where the robot velocity is below the stall threshold
    stalled_df = df[df['R_V'] <= STALLED_THRESHOLD]

    stalled_time = 0

    # Iterate through the index of the filtered dataframe (stalled_df)
    for t in range(1, len(stalled_df)):
        # Compare the current and previous indices to check if they are consecutive
        if stalled_df.index[t] == stalled_df.index[t-1] + 1:
            # Add the time difference between consecutive timestamps
            stalled_time += stalled_df.loc[stalled_df.index[t], 'ros_time'] - stalled_df.loc[stalled_df.index[t-1], 'ros_time']

    return stalled_time

def compute_sc_for_zones(df):
    """
    Compute the Personal Space Compliance (PSC) metric for multiple humans across proxemic zones.

    Parameters:
        df (pd.DataFrame): DataFrame containing robot and human positions over time.
        thresholds (list): Thresholds defining proxemic boundaries in meters.
            [intimate, personal, social, public]

    Returns:
        dict: PSC scores for each proxemic zone.
    """
    def penalty_function(distance, threshold):
        """Calculate the penalty for a given distance."""
        return 0 if distance >= threshold else 1 - (distance / threshold)

    # Identify human columns dynamically
    human_columns = [(col, col.replace('_X', '_Y')) for col in df.columns if col.startswith('a') and col.endswith('_X')]

    # Initialize a dictionary for PSC scores by zone
    zone_compliance = {zone: [] for zone in PROXEMIC_THRESHOLDS.keys()}

    for _, row in df.iterrows():
        # Robot position
        r_x, r_y = row['R_X'], row['R_Y']

        for ai_x_col, ai_y_col in human_columns:
            # Human position
            h_x, h_y = row[ai_x_col], row[ai_y_col]

            # Calculate the Euclidean distance between robot and human
            distance = np.sqrt((r_x - h_x)**2 + (r_y - h_y)**2)

            # Determine which zone the interaction falls into and add penalties
            for zone, threshold in PROXEMIC_THRESHOLDS.items():
                zone_compliance[zone].append(penalty_function(distance, threshold))

    # Compute the average PSC for each proxemic zone
    return {zone: float(1 - np.mean(penalties)) for zone, penalties in zone_compliance.items()}


INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
BAGNAME= 'noncausal_27122024'
SCENARIO = 'warehouse'
WPS_COORD = readScenario()

STALLED_THRESHOLD = 0.01
PROXEMIC_THRESHOLDS =  {'intimate': 0.5, 
                        'personal': 1.2, 
                        'social': 3.6, 
                        'public': 7.6}

with open(os.path.join(INDIR, BAGNAME, 'tasks.json')) as json_file:
    TASKS = json.load(json_file)

dfs = []
for tod in TOD:
    DF = pd.read_csv(os.path.join(INDIR, f"{BAGNAME}", f"{BAGNAME}_{tod.value}.csv"))
    r = get_initrow(DF)
    DF = DF[r:]
    DF.reset_index(drop=True, inplace=True)
    dfs.append(DF)
        
DF = pd.concat(dfs, axis=0)
DF.reset_index(drop=True, inplace=True)
del dfs

DF['Task_ID'] = DF['T']
TASK_IDs = [int(task_id) for task_id in DF['Task_ID'].unique()]
METRICS = {task_id: {'success': None,
                     'human_collision': None,
                     'stalled_time': None,
                     'time_to_reach_goal': None,
                     'path_length': None,
                     'min_velocity': None,
                     'max_velocity': None,
                     'average_velocity': None,
                     'min_clearing_distance': None,
                     'max_clearing_distance': None,
                     'average_clearing_distance': None,
                     'min_distance_to_humans': None,
                     'space_compliance': None} for task_id in TASK_IDs}
# Iterate over unique tasks
for task_id in TASK_IDs:
    
    task_df = DF[DF['Task_ID'] == task_id]
    
    SUCCESS = TASKS[str(task_id)]['result']
    HUMAN_COLLISION = task_df['R_HC'].sum()
    STALLED_TIME = compute_stalled_time(task_df)
    TIME_TO_GOAL = TASKS[str(task_id)]['end'] - TASKS[str(task_id)]['start']
    PATH_LENGTH = np.sum([math.sqrt((WPS_COORD[TASKS[str(task_id)]['path'][wp_idx]]['x'] - WPS_COORD[TASKS[str(task_id)]['path'][wp_idx+1]]['x'])**2 + (WPS_COORD[TASKS[str(task_id)]['path'][wp_idx]]['y'] - WPS_COORD[TASKS[str(task_id)]['path'][wp_idx+1]]['y'])**2)
                             for wp_idx in range(len(TASKS[str(task_id)]['path'])-1)])
    MIN_VELOCITY = task_df['R_V'].min()
    MAX_VELOCITY = task_df['R_V'].max()
    AVERAGE_VELOCITY = task_df['R_V'].mean()
    MIN_CLEARING_DISTANCE = task_df['R_CD'].min()
    MAX_CLEARING_DISTANCE = task_df['R_CD'].max()
    AVERAGE_CLEARING_DISTANCE = task_df['R_CD'].mean()
    MIN_DISTANCE_TO_HUMANS = compute_min_h_distance(task_df)
    SPACE_COMPLIANCE = compute_sc_for_zones(task_df)
  
    
    METRICS[task_id]['success'] = SUCCESS
    METRICS[task_id]['human_collision'] = int(HUMAN_COLLISION)
    METRICS[task_id]['stalled_time'] = float(STALLED_TIME)
    METRICS[task_id]['time_to_reach_goal'] = float(TIME_TO_GOAL)
    METRICS[task_id]['path_length'] = float(PATH_LENGTH)
    METRICS[task_id]['min_velocity'] = float(MIN_VELOCITY)
    METRICS[task_id]['max_velocity'] = float(MAX_VELOCITY)
    METRICS[task_id]['average_velocity'] = float(AVERAGE_VELOCITY)
    METRICS[task_id]['min_clearing_distance'] = float(MIN_CLEARING_DISTANCE)
    METRICS[task_id]['max_clearing_distance'] = float(MAX_CLEARING_DISTANCE)
    METRICS[task_id]['average_clearing_distance'] = float(AVERAGE_CLEARING_DISTANCE)
    METRICS[task_id]['min_distance_to_humans'] = float(MIN_DISTANCE_TO_HUMANS)
    METRICS[task_id]['space_compliance'] = SPACE_COMPLIANCE
    
# Filter data where B_S == 1 (robot in charging state)
BS = DF[DF['B_S'] == 1]
# Compute contiguous charging sessions
charging_sessions = []
battery_at_start_charging = []
start_time = None
end_time = None
# Iterate through BS to find start and end of contiguous charging states
for i, row in BS.iterrows():
    if start_time is None:  # Start of new charging session
        start_time = row['ros_time']
        battery_at_start_charging.append(row['R_B'])
    
    # Check if this is the end of a contiguous block
    if i + 1 not in BS.index:
        end_time = row['ros_time']
        charging_sessions.append((start_time, end_time))
        start_time = None
# Calculate durations of charging sessions
charging_times = [end - start for start, end in charging_sessions]
AVERAGE_CHARGING_TIME = np.mean(charging_times)
AVERAGE_BATTERY_LEVEL = np.mean(battery_at_start_charging)
# Add results to metrics dictionary
METRICS['overall_success'] = sum([1 if METRICS[task]['success'] == 1 else 0 for task in TASK_IDs])
METRICS['overall_failure'] = sum([1 if METRICS[task]['success'] == -1 else 0 for task in TASK_IDs])
METRICS['overall_human_collision'] = sum([METRICS[task]['human_collision'] for task in TASK_IDs])
METRICS['mean_stalled_time'] = float(np.mean([METRICS[task]['stalled_time'] for task in TASK_IDs]))
METRICS['mean_time_to_reach_goal'] = float(np.mean([METRICS[task]['time_to_reach_goal'] for task in TASK_IDs]))
METRICS['mean_path_length'] = float(np.mean([METRICS[task]['path_length'] for task in TASK_IDs]))
METRICS['mean_min_velocity'] = float(np.mean([METRICS[task]['min_velocity'] for task in TASK_IDs]))
METRICS['mean_max_velocity'] = float(np.mean([METRICS[task]['max_velocity'] for task in TASK_IDs]))
METRICS['mean_average_velocity'] = float(np.mean([METRICS[task]['average_velocity'] for task in TASK_IDs]))
METRICS['mean_min_clearing_distance'] = float(np.mean([METRICS[task]['min_clearing_distance'] for task in TASK_IDs]))
METRICS['mean_max_clearing_distance'] = float(np.mean([METRICS[task]['max_clearing_distance'] for task in TASK_IDs]))
METRICS['mean_average_clearing_distance'] = float(np.mean([METRICS[task]['average_clearing_distance'] for task in TASK_IDs]))
METRICS['mean_min_distance_to_humans'] = float(np.mean([METRICS[task]['min_distance_to_humans'] for task in TASK_IDs]))
METRICS['mean_space_compliance'] = {proxemic: None for proxemic in PROXEMIC_THRESHOLDS.keys()}
METRICS['mean_battery_charging_time'] = float(AVERAGE_CHARGING_TIME)
METRICS['mean_battery_at_start_charging'] = float(AVERAGE_BATTERY_LEVEL)
for proxemic in PROXEMIC_THRESHOLDS.keys():
    METRICS['mean_space_compliance'][proxemic] = float(np.mean([METRICS[task]['space_compliance'][proxemic] for task in TASK_IDs]))

with open(os.path.join(INDIR, f"{BAGNAME}", "metrics.json"), 'w') as json_file:
    json.dump(METRICS, json_file)
