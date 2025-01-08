import copy
import json
import math
import os
import numpy as np
import pandas as pd
from utils import *
from metrics_utils import *

INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
BAGNAME= ['causal-07012025']
# BAGNAME= ['noncausal-03012025', 'causal-04012025']
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
PROXEMIC_THRESHOLDS =  {'intimate': 0.5, 
                        'personal': 1.2, 
                        'social': 3.6, 
                        'public': 7.6}

for bag in BAGNAME:
    print(f"analysing {bag}")
    with open(os.path.join(INDIR, bag, 'tasks.json')) as json_file:
        TASKS = json.load(json_file)

    METRICS_ALL = {}
    tasks_to_continue = {}  # Dictionary to store tasks that continue to the next DF

    for tod in TOD:
        print(f"\t- {tod.value}")
        DF = pd.read_csv(os.path.join(INDIR, f"{bag}", f"{bag}_{tod.value}.csv"))
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
            PATH_LENGTH = np.sum([
                math.sqrt(
                    (WPS_COORD[TASKS[str(task_id)]['path'][wp_idx]]['x'] - WPS_COORD[TASKS[str(task_id)]['path'][wp_idx+1]]['x'])**2 +
                    (WPS_COORD[TASKS[str(task_id)]['path'][wp_idx]]['y'] - WPS_COORD[TASKS[str(task_id)]['path'][wp_idx+1]]['y'])**2
                )
                for wp_idx in range(len(TASKS[str(task_id)]['path'])-1)
            ])
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
            HUMAN_COLLISION = compute_human_collision(task_df)
            MIN_VELOCITY = task_df['R_V'].min()
            MAX_VELOCITY = task_df['R_V'].max()
            AVERAGE_VELOCITY = task_df['R_V'].mean()
            MIN_CLEARING_DISTANCE = task_df['R_CD'].min()
            MAX_CLEARING_DISTANCE = task_df['R_CD'].max()
            AVERAGE_CLEARING_DISTANCE = task_df['R_CD'].mean()
            MIN_DISTANCE_TO_HUMANS = compute_min_h_distance(task_df)
            SPACE_COMPLIANCE = compute_sc_for_zones(task_df, PROXEMIC_THRESHOLDS)
        
            METRICS[task_id]['result'] = SUCCESS
            METRICS[task_id]['path_length'] = float(PATH_LENGTH)
            METRICS[task_id]['planned_battery_consumption'] = float(PLANNED_BATTERY_CONSUMPTION)
            METRICS[task_id]['time_to_reach_goal'] = float(TIME_TO_GOAL)
            METRICS[task_id]['travelled_distance'] = float(TRAVELLED_DISTANCE)
            METRICS[task_id]['battery_consumption'] = float(BATTERY_CONSUMPTION)
            METRICS[task_id]['wasted_time_to_reach_goal'] = float(WASTED_TIME_TO_GOAL)
            METRICS[task_id]['wasted_travelled_distance'] = float(WASTED_TRAVELLED_DISTANCE)
            METRICS[task_id]['wasted_battery_consumption'] = float(WASTED_BATTERY_CONSUMPTION)
            METRICS[task_id]['robot_fallen'] = int(ROBOT_FALLS)
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
        
        tmp_tasks = list(METRICS.keys())
        METRICS['task_count'] = len(tmp_tasks)
        METRICS['overall_success'] = sum([1 if METRICS[task]['result'] == 1 else 0 for task in tmp_tasks])
        METRICS['overall_failure_people'] = sum([1 if METRICS[task]['result'] == -1 else 0 for task in tmp_tasks])
        METRICS['overall_failure_critical_battery'] = sum([1 if METRICS[task]['result'] == -2 else 0 for task in tmp_tasks])
        METRICS['overall_failure'] = METRICS['overall_failure_people'] + METRICS['overall_failure_critical_battery']

        METRICS['overall_path_length'] = float(np.sum([METRICS[task]['path_length'] for task in tmp_tasks]))
        METRICS['overall_path_length_only_success'] = float(np.sum([METRICS[task]['path_length'] for task in tmp_tasks if METRICS[task]['result'] == 1]))
        METRICS['overall_planned_battery_consumption'] = float(np.sum([METRICS[task]['planned_battery_consumption'] for task in tmp_tasks]))
        METRICS['overall_planned_battery_consumption_only_success'] = float(np.sum([METRICS[task]['planned_battery_consumption'] for task in tmp_tasks if METRICS[task]['result'] == 1]))
        METRICS['mean_path_length'] = float(np.mean([METRICS[task]['path_length'] for task in tmp_tasks]))
        METRICS['mean_path_length_only_success'] = float(np.mean([METRICS[task]['path_length'] for task in tmp_tasks if METRICS[task]['result'] == 1]))
        METRICS['mean_planned_battery_consumption'] = float(np.mean([METRICS[task]['planned_battery_consumption'] for task in tmp_tasks]))
        METRICS['mean_planned_battery_consumption_only_success'] = float(np.mean([METRICS[task]['planned_battery_consumption'] for task in tmp_tasks if METRICS[task]['result'] == 1]))

        METRICS['overall_travelled_distance'] = float(np.sum([METRICS[task]['travelled_distance'] for task in tmp_tasks]))
        METRICS['overall_battery_consumption'] = float(np.sum([METRICS[task]['battery_consumption'] for task in tmp_tasks]))
        METRICS['mean_travelled_distance'] = float(np.sum([METRICS[task]['travelled_distance'] for task in tmp_tasks])/METRICS['overall_success']) if METRICS['overall_success'] > 0 else 0
        METRICS['mean_battery_consumption'] = float(np.sum([METRICS[task]['battery_consumption'] for task in tmp_tasks])/METRICS['overall_success']) if METRICS['overall_success'] > 0 else 0

        METRICS['overall_wasted_travelled_distance'] = float(np.sum([METRICS[task]['wasted_travelled_distance'] for task in tmp_tasks]))
        METRICS['overall_wasted_battery_consumption'] = float(np.sum([METRICS[task]['wasted_battery_consumption'] for task in tmp_tasks]))
        METRICS['mean_wasted_travelled_distance'] = float(np.sum([METRICS[task]['wasted_travelled_distance'] for task in tmp_tasks])/METRICS['overall_failure']) if METRICS['overall_failure'] > 0 else 0
        METRICS['mean_wasted_battery_consumption'] = float(np.sum([METRICS[task]['wasted_battery_consumption'] for task in tmp_tasks])/METRICS['overall_failure']) if METRICS['overall_failure'] > 0 else 0
        
        METRICS['mean_stalled_time'] = float(np.sum([METRICS[task]['stalled_time'] for task in tmp_tasks])/METRICS['overall_success']) if METRICS['overall_success'] > 0 else 0
        METRICS['mean_time_to_reach_goal'] = float(np.sum([METRICS[task]['time_to_reach_goal'] for task in tmp_tasks])/METRICS['overall_success']) if METRICS['overall_success'] > 0 else 0
        METRICS['mean_wasted_time_to_reach_goal'] = float(np.sum([METRICS[task]['wasted_time_to_reach_goal'] for task in tmp_tasks])/METRICS['overall_failure']) if METRICS['overall_failure'] > 0 else 0
        METRICS['mean_task_time'] = METRICS['mean_stalled_time'] + METRICS['mean_time_to_reach_goal'] + METRICS['mean_wasted_time_to_reach_goal']
        
        METRICS['overall_stalled_time'] = float(np.sum([METRICS[task]['stalled_time'] for task in tmp_tasks]))
        METRICS['overall_time_to_reach_goal'] = float(np.sum([METRICS[task]['time_to_reach_goal'] for task in tmp_tasks]))
        METRICS['overall_wasted_time_to_reach_goal'] = float(np.sum([METRICS[task]['wasted_time_to_reach_goal'] for task in tmp_tasks]))
        METRICS['overall_task_time'] = METRICS['overall_stalled_time'] + METRICS['overall_time_to_reach_goal'] + METRICS['overall_wasted_time_to_reach_goal']

        METRICS['overall_robot_fallen'] = sum([METRICS[task]['robot_fallen'] for task in tmp_tasks])
        METRICS['overall_human_collision'] = sum([METRICS[task]['human_collision'] for task in tmp_tasks])
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

    with open(os.path.join(INDIR, f"{bag}", "metrics_timesplit.json"), 'w') as json_file:
        json.dump(METRICS_ALL, json_file, indent=4)

