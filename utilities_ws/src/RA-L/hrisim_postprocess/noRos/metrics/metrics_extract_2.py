import pickle
import json
import math
import os
import numpy as np
import pandas as pd
from utils import *
from metrics_utils import *
from tqdm import tqdm


INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
BAGNAME= ['base', 'causal']
# BAGNAME= ['causal']
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
Kobs = 3

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
    # for tod in [TOD.H1, TOD.H2, TOD.H3]:
        with open(os.path.join(INDIR, bag, f'tasks-{tod.value}.json')) as pkl_file:
            TASKS = json.load(pkl_file)
        TASK_IDs = list(range(TASKS['n_tasks']))
        
        DF = pd.read_csv(os.path.join(INDIR, f"{bag}", f"{bag}-{tod.value}.csv"))
        r = get_initrow(DF)
        DF = DF[r:]
            
        TOD_METRICS = {task_id: {} for task_id in TASK_IDs}
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
            NO_OBS_PLANNED_BATTERY_CONSUMPTION = compute_planned_battery_consumption(PATH_LENGTH, len(TASKS[str(task_id)]['path'])-1,
                                                ROBOT_MAX_VEL, Ks, Kd, Astar_time)
            OBS_PLANNED_BATTERY_CONSUMPTION = compute_planned_battery_consumption(PATH_LENGTH, len(TASKS[str(task_id)]['path'])-1,
                                                                                  ROBOT_MAX_VEL, Ks*Kobs, Kd*Kobs, Astar_time)
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
            SPACE_COMPLIANCE, AGENT_DISTANCES = compute_hall_count(task_df, PROXEMIC_THRESHOLDS)
        
            
            TOD_METRICS[task_id]['result'] = SUCCESS
            TOD_METRICS[task_id]['travelled_distance_planned'] = float(PATH_LENGTH)
            TOD_METRICS[task_id]['travelled_distance_actual'] = float(TRAVELLED_DISTANCE)
            TOD_METRICS[task_id]['travelled_distance_wasted'] = float(WASTED_TRAVELLED_DISTANCE)
            TOD_METRICS[task_id]['battery_consumption_planned_noobs'] = float(NO_OBS_PLANNED_BATTERY_CONSUMPTION)
            TOD_METRICS[task_id]['battery_consumption_planned_obs'] = float(OBS_PLANNED_BATTERY_CONSUMPTION)
            TOD_METRICS[task_id]['battery_consumption_planned'] = (float(OBS_PLANNED_BATTERY_CONSUMPTION) + float(NO_OBS_PLANNED_BATTERY_CONSUMPTION))/2
            TOD_METRICS[task_id]['battery_consumption_actual'] = float(BATTERY_CONSUMPTION)
            TOD_METRICS[task_id]['battery_consumption_wasted'] = float(WASTED_BATTERY_CONSUMPTION)
            TOD_METRICS[task_id]['time_to_reach_goal_actual'] = float(TIME_TO_GOAL)
            TOD_METRICS[task_id]['time_to_reach_goal_wasted'] = float(WASTED_TIME_TO_GOAL)
            TOD_METRICS[task_id]['time_to_reach_goal_stalled'] = float(STALLED_TIME)
            TOD_METRICS[task_id]['robot_fallen'] = int(ROBOT_FALLS)
            TOD_METRICS[task_id]['human_collision'] = int(HUMAN_COLLISION)
            TOD_METRICS[task_id]['agent_distances'] = AGENT_DISTANCES
            

        # Add results to metrics dictionary
        tmp_tasks = list(TOD_METRICS.keys())
        TOD_METRICS['task_count'] = len(tmp_tasks)
        TOD_METRICS['overall_success'] = sum([1 if TOD_METRICS[task]['result'] == 1 else 0 for task in tmp_tasks])
        TOD_METRICS['overall_failure_people'] = sum([1 if TOD_METRICS[task]['result'] == -1 else 0 for task in tmp_tasks])
        TOD_METRICS['overall_failure_critical_battery'] = sum([1 if TOD_METRICS[task]['result'] == -2 else 0 for task in tmp_tasks])
        TOD_METRICS['overall_failure'] = TOD_METRICS['overall_failure_people'] + TOD_METRICS['overall_failure_critical_battery']

        TOD_METRICS['overall_travelled_distance_planned'] = float(np.sum([TOD_METRICS[task]['travelled_distance_planned'] for task in tmp_tasks]))
        TOD_METRICS['overall_travelled_distance_planned_only_success'] = float(np.sum([TOD_METRICS[task]['travelled_distance_planned'] for task in tmp_tasks if TOD_METRICS[task]['result'] == 1]))
        TOD_METRICS['overall_travelled_distance_actual'] = float(np.sum([TOD_METRICS[task]['travelled_distance_actual'] for task in tmp_tasks]))
        TOD_METRICS['overall_travelled_distance_wasted'] = float(np.sum([TOD_METRICS[task]['travelled_distance_wasted'] for task in tmp_tasks]))

        TOD_METRICS['overall_battery_consumption_planned_noobs'] = float(np.sum([TOD_METRICS[task]['battery_consumption_planned_noobs'] for task in tmp_tasks]))
        TOD_METRICS['overall_battery_consumption_planned_noobs_only_success'] = float(np.sum([TOD_METRICS[task]['battery_consumption_planned_noobs'] for task in tmp_tasks if TOD_METRICS[task]['result'] == 1]))
        TOD_METRICS['overall_battery_consumption_planned_obs'] = float(np.sum([TOD_METRICS[task]['battery_consumption_planned_obs'] for task in tmp_tasks]))
        TOD_METRICS['overall_battery_consumption_planned_obs_only_success'] = float(np.sum([TOD_METRICS[task]['battery_consumption_planned_obs'] for task in tmp_tasks if TOD_METRICS[task]['result'] == 1]))
        TOD_METRICS['overall_battery_consumption_planned'] = float(np.sum([TOD_METRICS[task]['battery_consumption_planned'] for task in tmp_tasks]))
        TOD_METRICS['overall_battery_consumption_planned_only_success'] = float(np.sum([TOD_METRICS[task]['battery_consumption_planned'] for task in tmp_tasks if TOD_METRICS[task]['result'] == 1]))
        TOD_METRICS['overall_battery_consumption_actual'] = float(np.sum([TOD_METRICS[task]['battery_consumption_actual'] for task in tmp_tasks]))
        TOD_METRICS['overall_battery_consumption_wasted'] = float(np.sum([TOD_METRICS[task]['battery_consumption_wasted'] for task in tmp_tasks]))
        
        TOD_METRICS['overall_time_to_reach_goal_stalled'] = float(np.sum([TOD_METRICS[task]['time_to_reach_goal_stalled'] for task in tmp_tasks]))
        TOD_METRICS['overall_time_to_reach_goal_actual'] = float(np.sum([TOD_METRICS[task]['time_to_reach_goal_actual'] for task in tmp_tasks]))
        TOD_METRICS['overall_time_to_reach_goal_wasted'] = float(np.sum([TOD_METRICS[task]['time_to_reach_goal_wasted'] for task in tmp_tasks]))
        TOD_METRICS['overall_task_time'] = TOD_METRICS['overall_time_to_reach_goal_stalled'] + TOD_METRICS['overall_time_to_reach_goal_actual'] + TOD_METRICS['overall_time_to_reach_goal_wasted']
        
        TOD_METRICS['overall_robot_fallen'] = sum([TOD_METRICS[task]['robot_fallen'] for task in tmp_tasks])
        TOD_METRICS['overall_human_collision'] = sum([TOD_METRICS[task]['human_collision'] for task in tmp_tasks])

        METRICS[tod.value] = TOD_METRICS
    
    
    
    METRICS['task_count'] = sum([METRICS[tod]['task_count'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)])
    METRICS['overall_success'] = sum([METRICS[tod]['overall_success'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)])
    METRICS['overall_failure_people'] = sum([METRICS[tod]['overall_failure_people'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)])
    METRICS['overall_failure_critical_battery'] = sum([METRICS[tod]['overall_failure_critical_battery'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)])
    METRICS['overall_failure'] = METRICS['overall_failure_people'] + METRICS['overall_failure_critical_battery']

    METRICS['overall_travelled_distance_planned'] = float(np.sum([METRICS[tod]['overall_travelled_distance_planned'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_travelled_distance_planned_only_success'] = float(np.sum([METRICS[tod]['overall_travelled_distance_planned_only_success'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_travelled_distance_actual'] = float(np.sum([METRICS[tod]['overall_travelled_distance_actual'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_travelled_distance_wasted'] = float(np.sum([METRICS[tod]['overall_travelled_distance_wasted'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    
    METRICS['overall_battery_consumption_planned_noobs'] = float(np.sum([METRICS[tod]['overall_battery_consumption_planned_noobs'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_battery_consumption_planned_noobs_only_success'] = float(np.sum([METRICS[tod]['overall_battery_consumption_planned_noobs_only_success'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_battery_consumption_planned_obs'] = float(np.sum([METRICS[tod]['overall_battery_consumption_planned_obs'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_battery_consumption_planned_obs_only_success'] = float(np.sum([METRICS[tod]['overall_battery_consumption_planned_obs_only_success'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_battery_consumption_planned'] = float(np.sum([METRICS[tod]['overall_battery_consumption_planned'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_battery_consumption_planned_only_success'] = float(np.sum([METRICS[tod]['overall_battery_consumption_planned_only_success'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_battery_consumption_actual'] = float(np.sum([METRICS[tod]['overall_battery_consumption_actual'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_battery_consumption_wasted'] = float(np.sum([METRICS[tod]['overall_battery_consumption_wasted'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
      
    METRICS['overall_time_to_reach_goal_stalled'] = float(np.sum([METRICS[tod]['overall_time_to_reach_goal_stalled'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_time_to_reach_goal_actual'] = float(np.sum([METRICS[tod]['overall_time_to_reach_goal_actual'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_time_to_reach_goal_wasted'] = float(np.sum([METRICS[tod]['overall_time_to_reach_goal_wasted'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)]))
    METRICS['overall_task_time'] = METRICS['overall_time_to_reach_goal_stalled'] + METRICS['overall_time_to_reach_goal_actual'] + METRICS['overall_time_to_reach_goal_wasted']
  
    METRICS['overall_robot_fallen'] = sum([METRICS[tod]['overall_robot_fallen'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)])
    METRICS['overall_human_collision'] = sum([METRICS[tod]['overall_human_collision'] for tod in METRICS.keys() if isinstance(METRICS[tod], dict)])
   
    pkl_to_save = make_serializable(METRICS)
    with open(os.path.join(INDIR, f"{bag}", "metrics.pkl"), 'wb') as pkl_file:
        pickle.dump(pkl_to_save, pkl_file)
