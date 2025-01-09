import json
import os
from metrics_utils import *
from utils import *


INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
BAGNAMES = ['noncausal-03012025', 'causal-08012025']
CATEGORIES = {'noncausal-03012025': 'Non-Causal', 'causal-08012025': 'Causal'}
OUTDIR = os.path.join('/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/results', 'comparison', '__'.join(BAGNAMES), 'trends')
os.makedirs(OUTDIR, exist_ok=True)

# Initialize aggregated data structures
success_failure_trends = {CATEGORIES[bag]: {"N. Success": {"value": [], "color": "tab:blue"}, 
                                            "N. Failure (People)": {"value": [], "color": "tab:orange"}, 
                                            "N. Failure (Critical Battery)": {"value": [], "color": "tab:red"}} 
                          for bag in BAGNAMES}

working_time_trends = {CATEGORIES[bag]: {"Active Time": {"value": [], "color": "tab:blue"}, 
                                         "Stalled Time": {"value": [], "color": "tab:orange"},
                                         "Wasted Time": {"value": [], "color": "tab:red"}} 
                       for bag in BAGNAMES}

path_trends = {CATEGORIES[bag]: {"Planned Travelled Distance": {"value": [], "color": "tab:blue"}, 
                                 "Extra Travelled Distance": {"value": [], "color": "tab:orange"}, 
                                 "Wasted Travelled Distance": {"value": [], "color": "tab:red"}} 
               for bag in BAGNAMES}

battery_trends = {CATEGORIES[bag]: {"Planned Battery Usage": {"value": [], "color": "tab:blue"}, 
                                    "Extra Battery Usage": {"value": [], "color": "tab:orange"}, 
                                    "Wasted Battery Usage": {"value": [], "color": "tab:red"}} 
                  for bag in BAGNAMES}

velocity_trends = {CATEGORIES[bag]: {"Avg Velocity": {"value": [], "color": "tab:blue"}} for bag in BAGNAMES}
collision_trends = {CATEGORIES[bag]: {"Human Collisions": {"value": [], "color": "tab:blue"}} for bag in BAGNAMES}
clearance_trends = {CATEGORIES[bag]: {"Avg Clearing Distance": {"value": [], "color": "tab:blue"}} for bag in BAGNAMES}
proxemics_trends = {
    CATEGORIES[bag]: {
    "Public": {"value": [], "color": "tab:green"},
    "Social": {"value": [], "color": "tab:blue"},
    "Personal": {"value": [], "color": "tab:orange"},
    "Intimate": {"value": [], "color": "tab:red"},
    } for bag in BAGNAMES
}
# Load metrics for each bag
for tod in TOD:
    for bag in BAGNAMES:
        metrics_path = os.path.join(INDIR, bag, "metrics_timesplit.json")
        with open(metrics_path, 'r') as json_file:
            METRICS = json.load(json_file)
        
        METRICS = METRICS[tod.value]
            
        # EFFICIENCY
        success_failure_trends[CATEGORIES[bag]]["N. Success"]["value"].append(METRICS['overall_success']*100/METRICS['task_count'] if METRICS['task_count'] > 0 else 0)
        success_failure_trends[CATEGORIES[bag]]["N. Failure (People)"]["value"].append(METRICS['overall_failure_people']*100/METRICS['task_count'] if METRICS['task_count'] > 0 else 0)
        success_failure_trends[CATEGORIES[bag]]["N. Failure (Critical Battery)"]["value"].append(METRICS['overall_failure_critical_battery']*100/METRICS['task_count'] if METRICS['task_count'] > 0 else 0)

        working_time_trends[CATEGORIES[bag]]["Active Time"]["value"].append(METRICS['overall_time_to_reach_goal']*100/METRICS['overall_task_time'] if METRICS['overall_task_time'] > 0 else 0)
        working_time_trends[CATEGORIES[bag]]["Stalled Time"]["value"].append(METRICS['overall_stalled_time']*100/METRICS['overall_task_time'] if METRICS['overall_task_time'] > 0 else 0)
        working_time_trends[CATEGORIES[bag]]["Wasted Time"]["value"].append(METRICS['overall_wasted_time_to_reach_goal']*100/METRICS['overall_task_time'] if METRICS['overall_task_time'] > 0 else 0)

        path_trends[CATEGORIES[bag]]["Planned Travelled Distance"]["value"].append(METRICS['overall_path_length_only_success']*100/(METRICS['overall_travelled_distance'] + METRICS['overall_wasted_travelled_distance']) if (METRICS['overall_travelled_distance'] + METRICS['overall_wasted_travelled_distance']) > 0 else 0)
        path_trends[CATEGORIES[bag]]["Extra Travelled Distance"]["value"].append((METRICS['overall_travelled_distance'] - METRICS['overall_path_length_only_success'])*100/(METRICS['overall_travelled_distance'] + METRICS['overall_wasted_travelled_distance']) if (METRICS['overall_travelled_distance'] + METRICS['overall_wasted_travelled_distance']) > 0 else 0)
        path_trends[CATEGORIES[bag]]["Wasted Travelled Distance"]["value"].append(METRICS['overall_wasted_travelled_distance']*100/(METRICS['overall_travelled_distance'] + METRICS['overall_wasted_travelled_distance']) if (METRICS['overall_travelled_distance'] + METRICS['overall_wasted_travelled_distance']) > 0 else 0)

        battery_trends[CATEGORIES[bag]]["Planned Battery Usage"]["value"].append(METRICS['overall_planned_battery_consumption_only_success']*100/(METRICS['overall_battery_consumption'] + METRICS['overall_wasted_battery_consumption']) if (METRICS['overall_battery_consumption'] + METRICS['overall_wasted_battery_consumption']) > 0 else 0)
        battery_trends[CATEGORIES[bag]]["Extra Battery Usage"]["value"].append((METRICS['overall_battery_consumption'] - METRICS['overall_planned_battery_consumption_only_success'])*100/(METRICS['overall_battery_consumption'] + METRICS['overall_wasted_battery_consumption']) if (METRICS['overall_battery_consumption'] + METRICS['overall_wasted_battery_consumption']) > 0 else 0)
        battery_trends[CATEGORIES[bag]]["Wasted Battery Usage"]["value"].append(METRICS['overall_wasted_battery_consumption']*100/(METRICS['overall_battery_consumption'] + METRICS['overall_wasted_battery_consumption']) if (METRICS['overall_battery_consumption'] + METRICS['overall_wasted_battery_consumption']) > 0 else 0)
        
        velocity_trends[CATEGORIES[bag]]["Avg Velocity"]["value"].append(METRICS['mean_average_velocity'])
        
        # SAFETY
        collision_trends[CATEGORIES[bag]]["Human Collisions"]["value"].append(METRICS['overall_human_collision'] + METRICS['overall_robot_fallen'])
        clearance_trends[CATEGORIES[bag]]["Avg Clearing Distance"]["value"].append(METRICS['mean_average_clearing_distance'])
        
        denominator = METRICS['overall_space_compliance']['public'] + METRICS['overall_space_compliance']['social'] + METRICS['overall_space_compliance']['personal'] + METRICS['overall_space_compliance']['intimate']
        proxemics_trends[CATEGORIES[bag]]["Public"]["value"].append(METRICS['overall_space_compliance']['public']*100/denominator if denominator > 0 else 0)
        proxemics_trends[CATEGORIES[bag]]["Social"]["value"].append(METRICS['overall_space_compliance']['social']*100/denominator if denominator > 0 else 0)
        proxemics_trends[CATEGORIES[bag]]["Personal"]["value"].append(METRICS['overall_space_compliance']['personal']*100/denominator if denominator > 0 else 0)
        proxemics_trends[CATEGORIES[bag]]["Intimate"]["value"].append(METRICS['overall_space_compliance']['intimate']*100/denominator if denominator > 0 else 0)

time_labels = [tod.value for tod in TOD]

plot_grouped_stacked_bars(success_failure_trends, time_labels, "Success-Failure", "Percentage (%)", outdir=OUTDIR)
plot_grouped_stacked_bars(working_time_trends, time_labels, "Task Time", "Percentage (%)", outdir=OUTDIR)
plot_grouped_stacked_bars(path_trends, time_labels, "Path Length", "Percentage (%)", outdir=OUTDIR)
plot_grouped_stacked_bars(battery_trends, time_labels, "Battery Usage", "Percentage (%)", outdir=OUTDIR)
plot_grouped_stacked_bars(velocity_trends, time_labels, "Velocity", "m/s", label_bar_pos=0.01, percentage=False, outdir=OUTDIR)
plot_grouped_stacked_bars(collision_trends, time_labels, "Collision", "Count", label_bar_pos=0.5, percentage=False, outdir=OUTDIR)
plot_grouped_stacked_bars(clearance_trends, time_labels, "Clearance Distance to Obstacles", "m", label_bar_pos=0.025, percentage=False, outdir=OUTDIR)
plot_grouped_stacked_bars(proxemics_trends, time_labels, "Proxemics", "Percentage (%)", outdir=OUTDIR)