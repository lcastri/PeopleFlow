import copy
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

def compute_travelled_distance(task_df):
    """Compute the actual distance travelled by the robot."""
    distance = 0
    for i in range(1, len(task_df)):
        distance += math.sqrt((task_df['R_X'].iloc[i] - task_df['R_X'].iloc[i-1])**2 + (task_df['R_Y'].iloc[i] - task_df['R_Y'].iloc[i-1])**2)
    return distance


def compute_planned_battery_consumption(dist):
    time_to_goal = math.ceil(dist/ROBOT_MAX_VEL)
    return time_to_goal * (Ks + Kd * ROBOT_MAX_VEL)


def compute_actual_battery_consumption(task_df):
    battery = task_df['R_B'].to_numpy()
    return battery[0] - battery[-1]

def compute_human_collision(df):
    """
    Calculate the number of unique human collisions.
    A collision is counted only when a human enters the robot's base diameter and then exits.

    Parameters:
        df (pd.DataFrame): DataFrame containing robot and human positions over time.

    Returns:
        int: Total number of unique collisions with humans.
    """
    robot_coords = df[['R_X', 'R_Y']].to_numpy()
    collision_count = 0
    
    # Track collision state for each human
    human_collision_states = {}

    # Identify human coordinate columns dynamically
    for col in df.columns:
        if col.startswith('a') and col.endswith('_X'):
            human_id = col[:-2]  # Extract human identifier (e.g., 'a1', 'a2')
            human_collision_states[human_id] = False  # Initialize collision state

    # Iterate over time steps
    for _, row in df.iterrows():
        r_x, r_y = row['R_X'], row['R_Y']

        for col in df.columns:
            if col.startswith('a') and col.endswith('_X'):
                human_id = col[:-2]  # Extract human identifier
                h_x, h_y = row[col], row[col.replace('_X', '_Y')]

                # Calculate distance
                distance = np.sqrt((r_x - h_x)**2 + (r_y - h_y)**2)

                # Check for collision entry
                if distance < (ROBOT_BASE_DIAMETER/2) and not human_collision_states[human_id]:
                    collision_count += 1
                    human_collision_states[human_id] = True  # Human enters collision zone

                # Check for collision exit
                elif distance >= (ROBOT_BASE_DIAMETER/2):
                    human_collision_states[human_id] = False  # Human exits collision zone

    return collision_count

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
BAGNAME= 'causal-04012025'
SCENARIO = 'warehouse'
WPS_COORD = readScenario()

static_duration = 5
dynamic_duration = 4
ROBOT_MAX_VEL = 0.5
ROBOT_BASE_DIAMETER = 0.54
Ks = 100 / (static_duration * 3600)
Kd = (100 / (dynamic_duration * 3600) - Ks)/ROBOT_MAX_VEL

STALLED_THRESHOLD = 0.05
PROXEMIC_THRESHOLDS =  {'intimate': 0.5, 
                        'personal': 1.2, 
                        'social': 3.6, 
                        'public': 7.6}

with open(os.path.join(INDIR, BAGNAME, 'tasks.json')) as json_file:
    TASKS = json.load(json_file)

METRICS_ALL = {}
tasks_to_continue = {}  # Dictionary to store tasks that continue to the next DF

for tod in TOD:
    DF = pd.read_csv(os.path.join(INDIR, f"{BAGNAME}", f"{BAGNAME}_{tod.value}.csv"))
    r = get_initrow(DF)
    DF = DF[r:]
    DF.reset_index(drop=True, inplace=True)
    
    DF['Task_ID'] = DF['T']
    TASK_IDs = [int(task_id) for task_id in DF['Task_ID'].unique() if task_id != -1]
    METRICS = {task_id: {'result': None,
                        'path_length': None,
                        'planned_battery_consumption': None,
                        'time_to_reach_goal': None,
                        'travelled_distance': None,
                        'battery_consumption': None,
                        'wasted_time_to_reach_goal': None,
                        'wasted_travelled_distance': None, 
                        'wasted_battery_consumption': None,
                        'robot_falled': None,
                        'human_collision': None,
                        'stalled_time': None,
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
        if TASKS[str(task_id)]['end'] == 0: 
            del METRICS[task_id]
            continue
            
        task_df = DF[DF['Task_ID'] == task_id]
        if task_id in tasks_to_continue:
            prev_df = tasks_to_continue[task_id]
            task_df = pd.concat([prev_df, task_df], ignore_index=True)
            tasks_to_continue = {}
        
        # Check if the task needs to be continued in the next DF
        if TASKS[str(task_id)]['end'] > DF['ros_time'].iloc[-1]:
            tasks_to_continue[task_id] = task_df  # Store the partial data
            del METRICS[task_id]
            continue  # Skip further processing for now

        # Process task metrics if the task is entirely within this DF
        SUCCESS = TASKS[str(task_id)]['result']
        HUMAN_COLLISION = task_df['R_HC'].sum()
        STALLED_TIME = compute_stalled_time(task_df)
        TIME_TO_GOAL = TASKS[str(task_id)]['end'] - TASKS[str(task_id)]['start']
        PATH_LENGTH = np.sum([
            math.sqrt(
                (WPS_COORD[TASKS[str(task_id)]['path'][wp_idx]]['x'] - WPS_COORD[TASKS[str(task_id)]['path'][wp_idx+1]]['x'])**2 +
                (WPS_COORD[TASKS[str(task_id)]['path'][wp_idx]]['y'] - WPS_COORD[TASKS[str(task_id)]['path'][wp_idx+1]]['y'])**2
            )
            for wp_idx in range(len(TASKS[str(task_id)]['path'])-1)
        ])
        PLANNED_BATTERY_CONSUMPTION = compute_planned_battery_consumption(PATH_LENGTH)
        if SUCCESS == 1:
            TIME_TO_GOAL = TASKS[str(task_id)]['end'] - TASKS[str(task_id)]['start']
            TRAVELLED_DISTANCE = compute_travelled_distance(task_df)
            BATTERY_CONSUMPTION = compute_actual_battery_consumption(task_df)
            WASTED_TIME_TO_GOAL = 0
            WASTED_TRAVELLED_DISTANCE = 0
            WASTED_BATTERY_CONSUMPTION = 0
        else:
            TIME_TO_GOAL = 0
            TRAVELLED_DISTANCE = 0
            BATTERY_CONSUMPTION = 0
            WASTED_TIME_TO_GOAL = TASKS[str(task_id)]['end'] - TASKS[str(task_id)]['start']
            WASTED_TRAVELLED_DISTANCE = compute_travelled_distance(task_df)
            WASTED_BATTERY_CONSUMPTION = compute_actual_battery_consumption(task_df)
        STALLED_TIME = compute_stalled_time(task_df)
        ROBOT_FALLEN = task_df['R_HC'].sum()
        HUMAN_COLLISION = compute_human_collision(task_df)
        MIN_VELOCITY = task_df['R_V'].min()
        MAX_VELOCITY = task_df['R_V'].max()
        AVERAGE_VELOCITY = task_df['R_V'].mean()
        MIN_CLEARING_DISTANCE = task_df['R_CD'].min()
        MAX_CLEARING_DISTANCE = task_df['R_CD'].max()
        AVERAGE_CLEARING_DISTANCE = task_df['R_CD'].mean()
        MIN_DISTANCE_TO_HUMANS = compute_min_h_distance(task_df)
        SPACE_COMPLIANCE = compute_sc_for_zones(task_df)
    
        METRICS[task_id]['result'] = SUCCESS
        METRICS[task_id]['path_length'] = float(PATH_LENGTH)
        METRICS[task_id]['planned_battery_consumption'] = float(PLANNED_BATTERY_CONSUMPTION)
        METRICS[task_id]['time_to_reach_goal'] = float(TIME_TO_GOAL)
        METRICS[task_id]['travelled_distance'] = float(TRAVELLED_DISTANCE)
        METRICS[task_id]['battery_consumption'] = float(BATTERY_CONSUMPTION)
        METRICS[task_id]['wasted_time_to_reach_goal'] = float(WASTED_TIME_TO_GOAL)
        METRICS[task_id]['wasted_travelled_distance'] = float(WASTED_TRAVELLED_DISTANCE)
        METRICS[task_id]['wasted_battery_consumption'] = float(WASTED_BATTERY_CONSUMPTION)
        METRICS[task_id]['robot_fallen'] = int(ROBOT_FALLEN)
        METRICS[task_id]['human_collision'] = int(HUMAN_COLLISION)
        METRICS[task_id]['stalled_time'] = float(STALLED_TIME)
        METRICS[task_id]['min_velocity'] = float(MIN_VELOCITY)
        METRICS[task_id]['max_velocity'] = float(MAX_VELOCITY)
        METRICS[task_id]['average_velocity'] = float(AVERAGE_VELOCITY)
        METRICS[task_id]['min_clearing_distance'] = float(MIN_CLEARING_DISTANCE)
        METRICS[task_id]['max_clearing_distance'] = float(MAX_CLEARING_DISTANCE)
        METRICS[task_id]['average_clearing_distance'] = float(AVERAGE_CLEARING_DISTANCE)
        METRICS[task_id]['min_distance_to_humans'] = float(MIN_DISTANCE_TO_HUMANS)
        METRICS[task_id]['space_compliance'] = SPACE_COMPLIANCE
    
    tmp_tasks = copy.deepcopy(list(METRICS.keys()))
    METRICS['task_count'] = len(tmp_tasks)
    METRICS['overall_success'] = sum([1 if METRICS[task]['result'] == 1 else 0 for task in tmp_tasks])
    METRICS['overall_failure_people'] = sum([1 if METRICS[task]['result'] == -1 else 0 for task in tmp_tasks])
    METRICS['overall_failure_critical_battery'] = sum([1 if METRICS[task]['result'] == -2 else 0 for task in tmp_tasks])
    METRICS['overall_failure'] = METRICS['overall_failure_people'] + METRICS['overall_failure_critical_battery']

    METRICS['mean_path_length'] = float(np.mean([METRICS[task]['path_length'] for task in tmp_tasks]))
    METRICS['mean_planned_battery_consumption'] = float(np.mean([METRICS[task]['planned_battery_consumption'] for task in tmp_tasks]))

    METRICS['overall_time_to_reach_goal'] = float(np.sum([METRICS[task]['time_to_reach_goal'] for task in tmp_tasks]))
    METRICS['overall_travelled_distance'] = float(np.sum([METRICS[task]['travelled_distance'] for task in tmp_tasks]))
    METRICS['overall_battery_consumption'] = float(np.sum([METRICS[task]['battery_consumption'] for task in tmp_tasks]))
    METRICS['mean_time_to_reach_goal'] = float(np.sum([METRICS[task]['time_to_reach_goal'] for task in tmp_tasks])/METRICS['overall_success'])
    METRICS['mean_travelled_distance'] = float(np.sum([METRICS[task]['travelled_distance'] for task in tmp_tasks])/METRICS['overall_success'])
    METRICS['mean_battery_consumption'] = float(np.sum([METRICS[task]['battery_consumption'] for task in tmp_tasks])/METRICS['overall_success'])

    METRICS['overall_wasted_time_to_reach_goal'] = float(np.sum([METRICS[task]['wasted_time_to_reach_goal'] for task in tmp_tasks]))
    METRICS['overall_wasted_travelled_distance'] = float(np.sum([METRICS[task]['wasted_travelled_distance'] for task in tmp_tasks]))
    METRICS['overall_wasted_battery_consumption'] = float(np.sum([METRICS[task]['wasted_battery_consumption'] for task in tmp_tasks]))
    METRICS['mean_wasted_time_to_reach_goal'] = float(np.sum([METRICS[task]['wasted_time_to_reach_goal'] for task in tmp_tasks])/METRICS['overall_failure'])
    METRICS['mean_wasted_travelled_distance'] = float(np.sum([METRICS[task]['wasted_travelled_distance'] for task in tmp_tasks])/METRICS['overall_failure'])
    METRICS['mean_wasted_battery_consumption'] = float(np.sum([METRICS[task]['wasted_battery_consumption'] for task in tmp_tasks])/METRICS['overall_failure'])

    METRICS['overall_robot_fallen'] = sum([METRICS[task]['robot_fallen'] for task in tmp_tasks])
    METRICS['overall_human_collision'] = sum([METRICS[task]['human_collision'] for task in tmp_tasks])
    METRICS['overall_stalled_time'] = float(np.sum([METRICS[task]['stalled_time'] for task in tmp_tasks]))
    METRICS['mean_stalled_time'] = float(np.mean([METRICS[task]['stalled_time'] for task in tmp_tasks]))
    METRICS['mean_min_velocity'] = float(np.mean([METRICS[task]['min_velocity'] for task in tmp_tasks]))
    METRICS['mean_max_velocity'] = float(np.mean([METRICS[task]['max_velocity'] for task in tmp_tasks]))
    METRICS['mean_average_velocity'] = float(np.mean([METRICS[task]['average_velocity'] for task in tmp_tasks]))
    METRICS['mean_min_clearing_distance'] = float(np.mean([METRICS[task]['min_clearing_distance'] for task in tmp_tasks]))
    METRICS['mean_max_clearing_distance'] = float(np.mean([METRICS[task]['max_clearing_distance'] for task in tmp_tasks]))
    METRICS['mean_average_clearing_distance'] = float(np.mean([METRICS[task]['average_clearing_distance'] for task in tmp_tasks]))
    METRICS['mean_min_distance_to_humans'] = float(np.mean([METRICS[task]['min_distance_to_humans'] for task in tmp_tasks]))
    METRICS['mean_space_compliance'] = {proxemic: None for proxemic in PROXEMIC_THRESHOLDS.keys()}
    for proxemic in PROXEMIC_THRESHOLDS.keys():
        METRICS['mean_space_compliance'][proxemic] = float(np.mean([METRICS[task]['space_compliance'][proxemic] for task in tmp_tasks]))

    METRICS_ALL[tod.value] = copy.deepcopy(METRICS)

with open(os.path.join(INDIR, f"{BAGNAME}", "metrics_timesplit.json"), 'w') as json_file:
    json.dump(METRICS_ALL, json_file, indent=4)

