import json
import math
import os
import numpy as np
import pandas as pd
from utils import *
from metrics_utils import *
from tqdm import tqdm


INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
BAGNAME= ['base']
SCENARIO = 'warehouse'
WPS_COORD = readScenario(SCENARIO)

Astar_time = 0.75
static_duration = 5
dynamic_duration = 4
ROBOT_MAX_VEL = 0.5
ROBOT_BASE_DIAMETER = 0.54
Ks = 100 / (static_duration * 3600)
Kd = (100 / (dynamic_duration * 3600) - Ks)/ROBOT_MAX_VEL

STALLED_THRESHOLD = 0.05
PROXEMIC_THRESHOLDS =  {
                        'no-interaction': (7.6, -1),
                        'public': (3.6, 7.6),
                        'social': (1.2, 3.6), 
                        'personal': (0.5, 1.2), 
                        'intimate': (0, 0.5), 
                        }

for bag in BAGNAME:
    METRICS = {tod.value: None for tod in TOD}
    
    for tod in TOD:
        with open(os.path.join(INDIR, bag, f'tasks-{tod.value}.json')) as json_file:
            TASKS = json.load(json_file)
        TASK_IDs = list(range(TASKS['n_tasks']))
        
        DF = pd.read_csv(os.path.join(INDIR, f"{bag}", f"{bag}-{tod.value}.csv"))
        r = get_initrow(DF)
        DF = DF[r:]
            
        TOD_METRICS = {task_id: {'result': None,
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
                            'min_velocity': None,
                            'max_velocity': None,
                            'average_velocity': None,
                            'min_clearing_distance': None,
                            'max_clearing_distance': None,
                            'average_clearing_distance': None,
                            'min_distance_to_humans': None,
                            'space_compliance': None} for task_id in TASK_IDs}
        # Iterate over unique tasks
        for task_id in tqdm(TASK_IDs, desc=f"{bag}-{tod.value}"):
            start_time = TASKS[str(task_id)]['start']
            end_time = TASKS[str(task_id)]['end']
            
            if end_time == 0: 
                del TOD_METRICS[task_id]
                continue
            
            # Filter the data for the task based on the start and end time
            task_df = DF[(DF['ros_time'] >= start_time) & (DF['ros_time'] <= end_time)]
            
            SUCCESS = TASKS[str(task_id)]['result']
            PATH_LENGTH = np.sum([
                math.sqrt(
                    (WPS_COORD[TASKS[str(task_id)]['path'][wp_idx]]['x'] - WPS_COORD[TASKS[str(task_id)]['path'][wp_idx+1]]['x'])**2 + 
                    (WPS_COORD[TASKS[str(task_id)]['path'][wp_idx]]['y'] - WPS_COORD[TASKS[str(task_id)]['path'][wp_idx+1]]['y'])**2
                )
                for wp_idx in range(len(TASKS[str(task_id)]['path'])-1)])
            PLANNED_BATTERY_CONSUMPTION = compute_planned_battery_consumption(PATH_LENGTH, len(TASKS[str(task_id)]['path'])-1,
                                                                            ROBOT_MAX_VEL, Ks, Kd, Astar_time)
            if SUCCESS == 1:
                TIME_TO_GOAL = TASKS[str(task_id)]['end'] - TASKS[str(task_id)]['start']
                TRAVELLED_DISTANCE = compute_travelled_distance(task_df)
                BATTERY_CONSUMPTION = compute_actual_battery_consumption(task_df)
                STALLED_TIME = compute_stalled_time(task_df, STALLED_THRESHOLD) - Astar_time*(len(TASKS[str(task_id)]['path'])-1)
                WASTED_TIME_TO_GOAL = 0
                WASTED_TRAVELLED_DISTANCE = 0
                WASTED_BATTERY_CONSUMPTION = 0
            else:
                TIME_TO_GOAL = 0
                TRAVELLED_DISTANCE = 0
                BATTERY_CONSUMPTION = 0
                STALLED_TIME = 0
                WASTED_TIME_TO_GOAL = TASKS[str(task_id)]['end'] - TASKS[str(task_id)]['start']
                WASTED_TRAVELLED_DISTANCE = compute_travelled_distance(task_df)
                WASTED_BATTERY_CONSUMPTION = compute_actual_battery_consumption(task_df)
            ROBOT_FALLS = task_df['R_HC'].sum()
            HUMAN_COLLISION = compute_human_collision(task_df, ROBOT_BASE_DIAMETER)
            MIN_VELOCITY = task_df['R_V'].min()
            MAX_VELOCITY = task_df['R_V'].max()
            AVERAGE_VELOCITY = task_df['R_V'].mean()
            MIN_CLEARING_DISTANCE = task_df['R_CD'].min()
            MAX_CLEARING_DISTANCE = task_df['R_CD'].max() if task_df['R_CD'].max() != float('inf') else 0
            AVERAGE_CLEARING_DISTANCE = task_df['R_CD'].mean() if task_df['R_CD'].mean() != float('inf') else 0
            MIN_DISTANCE_TO_HUMANS = compute_min_h_distance(task_df)
            SPACE_COMPLIANCE, AGENT_DISTANCES = compute_hall_count(task_df, PROXEMIC_THRESHOLDS)
        
            
            TOD_METRICS[task_id]['result'] = SUCCESS
            TOD_METRICS[task_id]['path_length'] = float(PATH_LENGTH)
            TOD_METRICS[task_id]['planned_battery_consumption'] = float(PLANNED_BATTERY_CONSUMPTION)
            TOD_METRICS[task_id]['time_to_reach_goal'] = float(TIME_TO_GOAL)
            TOD_METRICS[task_id]['travelled_distance'] = float(TRAVELLED_DISTANCE)
            TOD_METRICS[task_id]['battery_consumption'] = float(BATTERY_CONSUMPTION)
            TOD_METRICS[task_id]['wasted_time_to_reach_goal'] = float(WASTED_TIME_TO_GOAL)
            TOD_METRICS[task_id]['wasted_travelled_distance'] = float(WASTED_TRAVELLED_DISTANCE)
            TOD_METRICS[task_id]['wasted_battery_consumption'] = float(WASTED_BATTERY_CONSUMPTION)
            TOD_METRICS[task_id]['robot_fallen'] = int(ROBOT_FALLS)
            TOD_METRICS[task_id]['human_collision'] = int(HUMAN_COLLISION)
            TOD_METRICS[task_id]['stalled_time'] = float(STALLED_TIME)
            TOD_METRICS[task_id]['min_velocity'] = float(MIN_VELOCITY)
            TOD_METRICS[task_id]['max_velocity'] = float(MAX_VELOCITY)
            TOD_METRICS[task_id]['average_velocity'] = float(AVERAGE_VELOCITY)
            TOD_METRICS[task_id]['min_clearing_distance'] = float(MIN_CLEARING_DISTANCE)
            TOD_METRICS[task_id]['max_clearing_distance'] = float(MAX_CLEARING_DISTANCE)
            TOD_METRICS[task_id]['average_clearing_distance'] = float(AVERAGE_CLEARING_DISTANCE)
            TOD_METRICS[task_id]['min_distance_to_humans'] = float(MIN_DISTANCE_TO_HUMANS)
            TOD_METRICS[task_id]['space_compliance'] = SPACE_COMPLIANCE
            TOD_METRICS[task_id]['agent_distances'] = AGENT_DISTANCES
            

        # Add results to metrics dictionary
        tmp_tasks = list(TOD_METRICS.keys())
        TOD_METRICS['task_count'] = len(tmp_tasks)
        TOD_METRICS['overall_success'] = sum([1 if TOD_METRICS[task]['result'] == 1 else 0 for task in tmp_tasks])
        TOD_METRICS['overall_failure_people'] = sum([1 if TOD_METRICS[task]['result'] == -1 else 0 for task in tmp_tasks])
        TOD_METRICS['overall_failure_critical_battery'] = sum([1 if TOD_METRICS[task]['result'] == -2 else 0 for task in tmp_tasks])
        TOD_METRICS['overall_failure'] = TOD_METRICS['overall_failure_people'] + TOD_METRICS['overall_failure_critical_battery']

        TOD_METRICS['overall_path_length'] = float(np.sum([TOD_METRICS[task]['path_length'] for task in tmp_tasks]))
        TOD_METRICS['overall_path_length_only_success'] = float(np.sum([TOD_METRICS[task]['path_length'] for task in tmp_tasks if TOD_METRICS[task]['result'] == 1]))
        TOD_METRICS['overall_planned_battery_consumption'] = float(np.sum([TOD_METRICS[task]['planned_battery_consumption'] for task in tmp_tasks]))
        TOD_METRICS['overall_planned_battery_consumption_only_success'] = float(np.sum([TOD_METRICS[task]['planned_battery_consumption'] for task in tmp_tasks if TOD_METRICS[task]['result'] == 1]))
        TOD_METRICS['mean_path_length'] = float(np.mean([TOD_METRICS[task]['path_length'] for task in tmp_tasks]))
        TOD_METRICS['mean_path_length_only_success'] = float(np.mean([TOD_METRICS[task]['path_length'] for task in tmp_tasks if TOD_METRICS[task]['result'] == 1]))
        TOD_METRICS['mean_planned_battery_consumption'] = float(np.mean([TOD_METRICS[task]['planned_battery_consumption'] for task in tmp_tasks]))
        TOD_METRICS['mean_planned_battery_consumption_only_success'] = float(np.mean([TOD_METRICS[task]['planned_battery_consumption'] for task in tmp_tasks if TOD_METRICS[task]['result'] == 1]))

        TOD_METRICS['overall_travelled_distance'] = float(np.sum([TOD_METRICS[task]['travelled_distance'] for task in tmp_tasks]))
        TOD_METRICS['overall_battery_consumption'] = float(np.sum([TOD_METRICS[task]['battery_consumption'] for task in tmp_tasks]))
        TOD_METRICS['overall_wasted_travelled_distance'] = float(np.sum([TOD_METRICS[task]['wasted_travelled_distance'] for task in tmp_tasks]))
        TOD_METRICS['overall_wasted_battery_consumption'] = float(np.sum([TOD_METRICS[task]['wasted_battery_consumption'] for task in tmp_tasks]))
        TOD_METRICS['mean_travelled_distance'] = float(np.mean([TOD_METRICS[task]['travelled_distance'] for task in tmp_tasks if TOD_METRICS[task]['result'] == 1]))
        TOD_METRICS['mean_battery_consumption'] = float(np.mean([TOD_METRICS[task]['battery_consumption'] for task in tmp_tasks if TOD_METRICS[task]['result'] == 1]))
        TOD_METRICS['mean_wasted_travelled_distance'] = float(np.mean([TOD_METRICS[task]['wasted_travelled_distance'] for task in tmp_tasks if TOD_METRICS[task]['result'] in [-1, -2]]))
        TOD_METRICS['mean_wasted_battery_consumption'] = float(np.mean([TOD_METRICS[task]['wasted_battery_consumption'] for task in tmp_tasks if TOD_METRICS[task]['result'] in [-1, -2]]))
        
        TOD_METRICS['overall_stalled_time'] = float(np.sum([TOD_METRICS[task]['stalled_time'] for task in tmp_tasks]))
        TOD_METRICS['overall_time_to_reach_goal'] = float(np.sum([TOD_METRICS[task]['time_to_reach_goal'] for task in tmp_tasks]))
        TOD_METRICS['overall_wasted_time_to_reach_goal'] = float(np.sum([TOD_METRICS[task]['wasted_time_to_reach_goal'] for task in tmp_tasks]))
        TOD_METRICS['overall_task_time'] = TOD_METRICS['overall_stalled_time'] + TOD_METRICS['overall_time_to_reach_goal'] + TOD_METRICS['overall_wasted_time_to_reach_goal']
        TOD_METRICS['mean_stalled_time'] = float(np.mean([TOD_METRICS[task]['stalled_time'] for task in tmp_tasks if TOD_METRICS[task]['result'] == 1]))
        TOD_METRICS['mean_time_to_reach_goal'] = float(np.mean([TOD_METRICS[task]['time_to_reach_goal'] for task in tmp_tasks if TOD_METRICS[task]['result'] == 1]))
        TOD_METRICS['mean_wasted_time_to_reach_goal'] = float(np.mean([TOD_METRICS[task]['wasted_time_to_reach_goal'] for task in tmp_tasks if TOD_METRICS[task]['result'] in [-1, -2]]))
        TOD_METRICS['mean_task_time'] = TOD_METRICS['mean_stalled_time'] + TOD_METRICS['mean_time_to_reach_goal'] + TOD_METRICS['mean_wasted_time_to_reach_goal']
        
        TOD_METRICS['overall_robot_fallen'] = sum([TOD_METRICS[task]['robot_fallen'] for task in tmp_tasks])
        TOD_METRICS['overall_human_collision'] = sum([TOD_METRICS[task]['human_collision'] for task in tmp_tasks])
        TOD_METRICS['mean_min_velocity'] = float(np.mean([TOD_METRICS[task]['min_velocity'] for task in tmp_tasks]))
        TOD_METRICS['mean_max_velocity'] = float(np.mean([TOD_METRICS[task]['max_velocity'] for task in tmp_tasks]))
        TOD_METRICS['mean_average_velocity'] = float(np.mean([TOD_METRICS[task]['average_velocity'] for task in tmp_tasks]))
        TOD_METRICS['mean_min_clearing_distance'] = float(np.mean([TOD_METRICS[task]['min_clearing_distance'] for task in tmp_tasks]))
        TOD_METRICS['mean_max_clearing_distance'] = float(np.mean([TOD_METRICS[task]['max_clearing_distance'] for task in tmp_tasks]))
        TOD_METRICS['mean_average_clearing_distance'] = float(np.mean([TOD_METRICS[task]['average_clearing_distance'] for task in tmp_tasks]))
        TOD_METRICS['mean_min_distance_to_humans'] = float(np.mean([TOD_METRICS[task]['min_distance_to_humans'] for task in tmp_tasks]))
        TOD_METRICS['overall_space_compliance'] = {proxemic: None for proxemic in PROXEMIC_THRESHOLDS.keys()}
        TOD_METRICS['mean_space_compliance'] = {proxemic: None for proxemic in PROXEMIC_THRESHOLDS.keys()}
        for proxemic in PROXEMIC_THRESHOLDS.keys():
            TOD_METRICS['overall_space_compliance'][proxemic] = float(np.sum([TOD_METRICS[task]['space_compliance'][proxemic] for task in tmp_tasks]))
            TOD_METRICS['mean_space_compliance'][proxemic] = float(np.mean([TOD_METRICS[task]['space_compliance'][proxemic] for task in tmp_tasks]))

        METRICS[tod.value] = TOD_METRICS
    
    
    
    METRICS['task_count'] = sum([METRICS[tod]['task_count'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)])
    METRICS['overall_success'] = sum([METRICS[tod]['overall_success'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)])
    METRICS['overall_failure_people'] = sum([METRICS[tod]['overall_failure_people'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)])
    METRICS['overall_failure_critical_battery'] = sum([METRICS[tod]['overall_failure_critical_battery'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)])
    METRICS['overall_failure'] = METRICS['overall_failure_people'] + METRICS['overall_failure_critical_battery']

    METRICS['overall_path_length'] = float(np.sum([METRICS[tod]['overall_path_length'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_path_length_only_success'] = float(np.sum([METRICS[tod]['overall_path_length_only_success'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_planned_battery_consumption'] = float(np.sum([METRICS[tod]['overall_planned_battery_consumption'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_planned_battery_consumption_only_success'] = float(np.sum([METRICS[tod]['overall_planned_battery_consumption_only_success'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['mean_path_length'] = float(np.mean([METRICS[tod][task]['path_length'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int)]))
    METRICS['mean_path_length_only_success'] = float(np.mean([METRICS[tod][task]['path_length'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int) and METRICS[tod][task]['result'] == 1]))
    METRICS['mean_planned_battery_consumption'] = float(np.mean([METRICS[tod][task]['planned_battery_consumption'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int)]))
    METRICS['mean_planned_battery_consumption_only_success'] = float(np.mean([METRICS[tod][task]['planned_battery_consumption'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int) and METRICS[tod][task]['result'] == 1]))

    METRICS['overall_travelled_distance'] = float(np.sum([METRICS[tod]['overall_travelled_distance'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_battery_consumption'] = float(np.sum([METRICS[tod]['overall_battery_consumption'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_wasted_travelled_distance'] = float(np.sum([METRICS[tod]['overall_wasted_travelled_distance'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_wasted_battery_consumption'] = float(np.sum([METRICS[tod]['overall_wasted_battery_consumption'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['mean_travelled_distance'] = float(np.mean([METRICS[tod][task]['travelled_distance'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int) and METRICS[tod][task]['result'] == 1]))
    METRICS['mean_battery_consumption'] = float(np.mean([METRICS[tod][task]['battery_consumption'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int) and METRICS[tod][task]['result'] == 1]))
    METRICS['mean_wasted_travelled_distance'] = float(np.mean([METRICS[tod][task]['wasted_travelled_distance'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int) and METRICS[tod][task]['result'] in [-1, -2]]))
    METRICS['mean_wasted_battery_consumption'] = float(np.mean([METRICS[tod][task]['wasted_battery_consumption'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int) and METRICS[tod][task]['result'] in [-1, -2]]))
        
    METRICS['overall_stalled_time'] = float(np.sum([METRICS[tod]['overall_stalled_time'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_time_to_reach_goal'] = float(np.sum([METRICS[tod]['overall_time_to_reach_goal'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_wasted_time_to_reach_goal'] = float(np.sum([METRICS[tod]['overall_wasted_time_to_reach_goal'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_task_time'] = METRICS['overall_stalled_time'] + METRICS['overall_time_to_reach_goal'] + METRICS['overall_wasted_time_to_reach_goal']
    METRICS['mean_stalled_time'] = float(np.mean([METRICS[tod][task]['stalled_time'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int) and METRICS[tod][task]['result'] == 1]))
    METRICS['mean_time_to_reach_goal'] = float(np.mean([METRICS[tod][task]['time_to_reach_goal'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int) and METRICS[tod][task]['result'] == 1]))
    METRICS['mean_wasted_time_to_reach_goal'] = float(np.sum([METRICS[tod][task]['wasted_time_to_reach_goal'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int) and METRICS[tod][task]['result'] in [-1, -2]]))
    METRICS['mean_task_time'] = METRICS['mean_stalled_time'] + METRICS['mean_time_to_reach_goal'] + METRICS['mean_wasted_time_to_reach_goal']
        

    METRICS['overall_robot_fallen'] = sum([METRICS[tod]['overall_robot_fallen'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)])
    METRICS['overall_human_collision'] = sum([METRICS[tod]['overall_human_collision'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)])
    METRICS['mean_min_velocity'] = float(np.mean([METRICS[tod][task]['min_velocity'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int)]))
    METRICS['mean_max_velocity'] = float(np.mean([METRICS[tod][task]['max_velocity'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int)]))
    METRICS['mean_average_velocity'] = float(np.mean([METRICS[tod][task]['average_velocity'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int)]))
    METRICS['mean_average_clearing_distance'] = float(np.mean([METRICS[tod][task]['average_clearing_distance'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int)]))
    METRICS['overall_space_compliance'] = {proxemic: None for proxemic in PROXEMIC_THRESHOLDS.keys()}
    METRICS['mean_space_compliance'] = {proxemic: None for proxemic in PROXEMIC_THRESHOLDS.keys()}
    for proxemic in PROXEMIC_THRESHOLDS.keys():
        METRICS['overall_space_compliance'][proxemic] = float(np.sum([METRICS[tod][task]['space_compliance'][proxemic] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int)]))
        METRICS['mean_space_compliance'][proxemic] = float(np.mean([METRICS[tod][task]['space_compliance'][proxemic] for tod in METRICS.keys() if isinstance(METRICS[tod], dict) for task in METRICS[tod].keys() if isinstance(task, int)]))

    json_to_save = make_serializable(METRICS)
    with open(os.path.join(INDIR, f"{bag}", "metrics.json"), 'w') as json_file:
        json.dump(json_to_save, json_file)
