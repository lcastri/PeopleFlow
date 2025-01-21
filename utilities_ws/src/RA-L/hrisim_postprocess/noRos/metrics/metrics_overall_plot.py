import json
import os
from metrics_utils import *

INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
BAGNAMES = ['noncausal-11012025', 'causal-12012025']
CATEGORIES = {'noncausal-11012025': 'Non-Causal', 'causal-12012025': 'Causal'}
OUTDIR = os.path.join('/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/results', 'comparison', '__'.join(BAGNAMES), 'overall')
os.makedirs(OUTDIR, exist_ok=True)

# Initialize aggregated data structures
success_failure_metrics = {}
working_time_metrics = {}
path_metrics = {}
battery_metrics = {}
velocity_metrics = {}
battery_charging_metrics = {}
charging_time_metrics = {}
collision_metrics = {}
clearance_metrics = {}
proxemics_metrics = {}

# Load metrics for each bag
for bagname in BAGNAMES:
    metrics_path = os.path.join(INDIR, bagname, "metrics.json")
    with open(metrics_path, 'r') as json_file:
        METRICS = json.load(json_file)
    
    # EFFICIENCY 
    success_failure_metrics[bagname] = {
        "N. Success": {"value": METRICS['overall_success'],
                       "%": METRICS['overall_success']*100/METRICS['task_count'],
                       "100%": METRICS['task_count'],
                       "color": "tab:blue"},
        "N. Failure (People)": {"value": METRICS['overall_failure_people'],
                                "%": METRICS['overall_failure_people']*100/METRICS['task_count'], 
                                "100%": METRICS['task_count'],
                                "color": "tab:orange"},
        "N. Failure (Critical Battery)": {"value": METRICS['overall_failure_critical_battery'], 
                                          "%": METRICS['overall_failure_critical_battery']*100/METRICS['task_count'], 
                                          "100%": METRICS['task_count'],
                                          "color": "tab:red"}
    }
    working_time_metrics[bagname] = {
        "Active Time": {"value": METRICS['overall_time_to_reach_goal'], 
                        "%": METRICS['overall_time_to_reach_goal']*100/METRICS['overall_task_time'],
                        "100%": METRICS['overall_task_time'],
                        "color": "tab:blue"},
        "Stalled Time": {"value": METRICS['overall_stalled_time'], 
                         "%": METRICS['overall_stalled_time']*100/METRICS['overall_task_time'], 
                        "100%": METRICS['overall_task_time'],
                         "color": "tab:orange"},
        "Wasted Time": {"value": METRICS['overall_wasted_time_to_reach_goal'], 
                        "%": METRICS['overall_wasted_time_to_reach_goal']*100/METRICS['overall_task_time'], 
                        "100%": METRICS['overall_task_time'],
                        "color": "tab:red"},
    }
    path_metrics[bagname] = {
        "Planned Travelled Distance": {"value": METRICS['overall_path_length_only_success'], 
                                       "%": METRICS['overall_path_length_only_success']*100/(METRICS['overall_travelled_distance']+METRICS['overall_wasted_travelled_distance']),
                                       "100%": METRICS['overall_travelled_distance']+METRICS['overall_wasted_travelled_distance'],
                                       "color": "tab:blue"},
        "Extra Travelled Distance": {"value": METRICS['overall_travelled_distance'] - METRICS['overall_path_length_only_success'],
                                     "%": (METRICS['overall_travelled_distance'] - METRICS['overall_path_length_only_success'])*100/(METRICS['overall_travelled_distance']+METRICS['overall_wasted_travelled_distance']), 
                                     "100%": METRICS['overall_travelled_distance']+METRICS['overall_wasted_travelled_distance'],
                                     "color": "tab:orange"},
        "Wasted Travelled Distance": {"value": METRICS['overall_wasted_travelled_distance'],
                                      "%": METRICS['overall_wasted_travelled_distance']*100/(METRICS['overall_travelled_distance']+METRICS['overall_wasted_travelled_distance']),  
                                      "100%": METRICS['overall_travelled_distance']+METRICS['overall_wasted_travelled_distance'],
                                      "color": "tab:red"},
    }
    battery_metrics[bagname] = {
        "Planned Battery Usage": {"value": METRICS['overall_planned_battery_consumption_only_success'],
                                  "%": METRICS['overall_planned_battery_consumption_only_success']*100/(METRICS['overall_battery_consumption']+METRICS['overall_wasted_battery_consumption']),
                                  "100%": METRICS['overall_battery_consumption']+METRICS['overall_wasted_battery_consumption'],
                                  "color": "tab:blue"},
        "Extra Battery Usage": {"value": METRICS['overall_battery_consumption'] - METRICS['overall_planned_battery_consumption_only_success'],
                                "%": (METRICS['overall_battery_consumption'] - METRICS['overall_planned_battery_consumption_only_success'])*100/(METRICS['overall_battery_consumption']+METRICS['overall_wasted_battery_consumption']),  
                                "100%": METRICS['overall_battery_consumption']+METRICS['overall_wasted_battery_consumption'],
                                "color": "tab:orange"},
        "Wasted Battery Usage": {"value": METRICS['overall_wasted_battery_consumption'], 
                                 "%": METRICS['overall_wasted_battery_consumption']*100/(METRICS['overall_battery_consumption']+METRICS['overall_wasted_battery_consumption']), 
                                 "100%": METRICS['overall_battery_consumption']+METRICS['overall_wasted_battery_consumption'],
                                 "color": "tab:red"},
    }
    velocity_metrics[bagname] = {
        "Avg Velocity": {"value": METRICS['mean_average_velocity'], 
                         "color": "tab:blue"},
    }
    
    # SAFETY 
    collision_metrics[bagname] = {
        "Human Collisions": {"value": METRICS['overall_human_collision'], 
                             "color": "tab:blue"},
    }

    clearance_metrics[bagname] = {
        "Avg Clearing Distance": {"value": METRICS['mean_average_clearing_distance'], 
                                  "color": "tab:blue"},
    }
    denominator = METRICS['overall_space_compliance']['public'] + METRICS['overall_space_compliance']['social'] + METRICS['overall_space_compliance']['personal'] + METRICS['overall_space_compliance']['intimate']
    proxemics_metrics[bagname] = {
        "Public Proxemics": {"value": METRICS['overall_space_compliance']['public']*100/denominator, 
                             "color": "tab:green"},
        "Social Proxemics": {"value": METRICS['overall_space_compliance']['social']*100/denominator, 
                             "color": "tab:blue"},
        "Personal Proxemics": {"value": METRICS['overall_space_compliance']['personal']*100/denominator, 
                               "color": "tab:orange"},
        "Intimate Proxemics": {"value": METRICS['overall_space_compliance']['intimate']*100/denominator, 
                               "color": "tab:red"},
    }
    

# Plot all metrics
plot_stacked_bar(success_failure_metrics, "Success-Failure", "Count", CATEGORIES, outdir=OUTDIR)
plot_stacked_bar(working_time_metrics, "Task Time", "s", CATEGORIES, outdir=OUTDIR)
plot_stacked_bar(path_metrics, "Path Length", "m", CATEGORIES, outdir=OUTDIR)
plot_stacked_bar(battery_metrics, "Battery Usage", "%", CATEGORIES, outdir=OUTDIR)
# plot_stacked_bar(velocity_metrics, "Velocity", "m/s", CATEGORIES, figsize=(8, 4), outdir=OUTDIR)
# plot_stacked_bar(collision_metrics, "Collision", "Count", CATEGORIES, figsize=(8, 4), outdir=OUTDIR)
# plot_stacked_bar(clearance_metrics, "Clearance Distance to Obstacles", "m", CATEGORIES, figsize=(8, 4), outdir=OUTDIR)
# plot_stacked_bar(battery_charging_metrics, "Battery Level at the Start of Charging", "%", CATEGORIES, figsize=(8, 4), outdir=OUTDIR)
# plot_stacked_bar(charging_time_metrics, "Charging Time", "s", CATEGORIES, figsize=(8, 4), outdir=OUTDIR)
# plot_stacked_bar(proxemics_metrics, "Proxemics", "%", CATEGORIES, figsize=(8, 4), outdir=OUTDIR)