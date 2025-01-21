import json
import os
from metrics_utils import *
from utils import *


INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
BAGNAMES = ['noncausal-11012025', 'causal-12012025']
CATEGORIES = {'noncausal-11012025': 'Non-Causal', 'causal-12012025': 'Causal'}
OUTDIR = os.path.join('/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/results', 'comparison', '__'.join(BAGNAMES), 'trends')
os.makedirs(OUTDIR, exist_ok=True)

# Initialize aggregated data structures
success_failure_trends = {CATEGORIES[bag]: {"N. Success": {"value": [], "%": [], "100%": [], "color": "tab:blue"}, 
                                            "N. Failure (People)": {"value": [], "%": [], "100%": [], "color": "tab:orange"}, 
                                            "N. Failure (Critical Battery)": {"value": [], "%": [], "100%": [], "color": "tab:red"}} 
                          for bag in BAGNAMES}

working_time_trends = {CATEGORIES[bag]: {"Active Time": {"value": [], "%": [], "100%": [], "color": "tab:blue"}, 
                                         "Stalled Time": {"value": [], "%": [], "100%": [], "color": "tab:orange"},
                                         "Wasted Time": {"value": [], "%": [], "100%": [], "color": "tab:red"}} 
                       for bag in BAGNAMES}

path_trends = {CATEGORIES[bag]: {"Planned Travelled Distance": {"value": [], "%": [], "100%": [], "color": "tab:blue"}, 
                                 "Extra Travelled Distance": {"value": [], "%": [], "100%": [], "color": "tab:orange"}, 
                                 "Wasted Travelled Distance": {"value": [], "%": [], "100%": [], "color": "tab:red"}} 
               for bag in BAGNAMES}

battery_trends = {CATEGORIES[bag]: {"Planned Battery Usage": {"value": [], "%": [], "100%": [], "color": "tab:blue"}, 
                                    "Extra Battery Usage": {"value": [], "%": [], "100%": [], "color": "tab:orange"}, 
                                    "Wasted Battery Usage": {"value": [], "%": [], "100%": [], "color": "tab:red"}} 
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
        metrics_path = os.path.join(INDIR, bag, "metrics.json")
        with open(metrics_path, 'r') as json_file:
            METRICS = json.load(json_file)
        
        METRICS = METRICS[tod.value]
            
        # EFFICIENCY
        success_failure_trends[CATEGORIES[bag]]["N. Success"]["value"].append(METRICS['overall_success'])
        success_failure_trends[CATEGORIES[bag]]["N. Failure (People)"]["value"].append(METRICS['overall_failure_people'])
        success_failure_trends[CATEGORIES[bag]]["N. Failure (Critical Battery)"]["value"].append(METRICS['overall_failure_critical_battery'])
        success_failure_trends[CATEGORIES[bag]]["N. Success"]["%"].append(METRICS['overall_success']*100/METRICS['task_count'] if METRICS['task_count'] > 0 else 0)
        success_failure_trends[CATEGORIES[bag]]["N. Failure (People)"]["%"].append(METRICS['overall_failure_people']*100/METRICS['task_count'] if METRICS['task_count'] > 0 else 0)
        success_failure_trends[CATEGORIES[bag]]["N. Failure (Critical Battery)"]["%"].append(METRICS['overall_failure_critical_battery']*100/METRICS['task_count'] if METRICS['task_count'] > 0 else 0)
        success_failure_trends[CATEGORIES[bag]]["N. Success"]["100%"].append(METRICS['task_count'])
        success_failure_trends[CATEGORIES[bag]]["N. Failure (People)"]["100%"].append(METRICS['task_count'])
        success_failure_trends[CATEGORIES[bag]]["N. Failure (Critical Battery)"]["100%"].append(METRICS['task_count'])

        working_time_trends[CATEGORIES[bag]]["Active Time"]["value"].append(METRICS['overall_time_to_reach_goal'])
        working_time_trends[CATEGORIES[bag]]["Stalled Time"]["value"].append(METRICS['overall_stalled_time'])
        working_time_trends[CATEGORIES[bag]]["Wasted Time"]["value"].append(METRICS['overall_wasted_time_to_reach_goal'])
        working_time_trends[CATEGORIES[bag]]["Active Time"]["%"].append(METRICS['overall_time_to_reach_goal']*100/METRICS['overall_task_time'] if METRICS['overall_task_time'] > 0 else 0)
        working_time_trends[CATEGORIES[bag]]["Stalled Time"]["%"].append(METRICS['overall_stalled_time']*100/METRICS['overall_task_time'] if METRICS['overall_task_time'] > 0 else 0)
        working_time_trends[CATEGORIES[bag]]["Wasted Time"]["%"].append(METRICS['overall_wasted_time_to_reach_goal']*100/METRICS['overall_task_time'] if METRICS['overall_task_time'] > 0 else 0)
        working_time_trends[CATEGORIES[bag]]["Active Time"]["100%"].append(METRICS['overall_task_time'])
        working_time_trends[CATEGORIES[bag]]["Stalled Time"]["100%"].append(METRICS['overall_task_time'])
        working_time_trends[CATEGORIES[bag]]["Wasted Time"]["100%"].append(METRICS['overall_task_time'])

        path_trends[CATEGORIES[bag]]["Planned Travelled Distance"]["value"].append(METRICS['overall_path_length_only_success'])
        path_trends[CATEGORIES[bag]]["Extra Travelled Distance"]["value"].append(METRICS['overall_travelled_distance'] - METRICS['overall_path_length_only_success'])
        path_trends[CATEGORIES[bag]]["Wasted Travelled Distance"]["value"].append(METRICS['overall_wasted_travelled_distance'])
        path_trends[CATEGORIES[bag]]["Planned Travelled Distance"]["%"].append(METRICS['overall_path_length_only_success']*100/(METRICS['overall_travelled_distance'] + METRICS['overall_wasted_travelled_distance']) if (METRICS['overall_travelled_distance'] + METRICS['overall_wasted_travelled_distance']) > 0 else 0)
        path_trends[CATEGORIES[bag]]["Extra Travelled Distance"]["%"].append((METRICS['overall_travelled_distance'] - METRICS['overall_path_length_only_success'])*100/(METRICS['overall_travelled_distance'] + METRICS['overall_wasted_travelled_distance']) if (METRICS['overall_travelled_distance'] + METRICS['overall_wasted_travelled_distance']) > 0 else 0)
        path_trends[CATEGORIES[bag]]["Wasted Travelled Distance"]["%"].append(METRICS['overall_wasted_travelled_distance']*100/(METRICS['overall_travelled_distance'] + METRICS['overall_wasted_travelled_distance']) if (METRICS['overall_travelled_distance'] + METRICS['overall_wasted_travelled_distance']) > 0 else 0)
        path_trends[CATEGORIES[bag]]["Planned Travelled Distance"]["100%"].append(METRICS['overall_travelled_distance'] + METRICS['overall_wasted_travelled_distance'])
        path_trends[CATEGORIES[bag]]["Extra Travelled Distance"]["100%"].append(METRICS['overall_travelled_distance'] + METRICS['overall_wasted_travelled_distance'])
        path_trends[CATEGORIES[bag]]["Wasted Travelled Distance"]["100%"].append(METRICS['overall_travelled_distance'] + METRICS['overall_wasted_travelled_distance'])

        battery_trends[CATEGORIES[bag]]["Planned Battery Usage"]["value"].append(METRICS['overall_planned_battery_consumption_only_success'])
        battery_trends[CATEGORIES[bag]]["Extra Battery Usage"]["value"].append(METRICS['overall_battery_consumption'] - METRICS['overall_planned_battery_consumption_only_success'])
        battery_trends[CATEGORIES[bag]]["Wasted Battery Usage"]["value"].append(METRICS['overall_wasted_battery_consumption'])
        battery_trends[CATEGORIES[bag]]["Planned Battery Usage"]["%"].append(METRICS['overall_planned_battery_consumption_only_success']*100/(METRICS['overall_battery_consumption'] + METRICS['overall_wasted_battery_consumption']) if (METRICS['overall_battery_consumption'] + METRICS['overall_wasted_battery_consumption']) > 0 else 0)
        battery_trends[CATEGORIES[bag]]["Extra Battery Usage"]["%"].append((METRICS['overall_battery_consumption'] - METRICS['overall_planned_battery_consumption_only_success'])*100/(METRICS['overall_battery_consumption'] + METRICS['overall_wasted_battery_consumption']) if (METRICS['overall_battery_consumption'] + METRICS['overall_wasted_battery_consumption']) > 0 else 0)
        battery_trends[CATEGORIES[bag]]["Wasted Battery Usage"]["%"].append(METRICS['overall_wasted_battery_consumption']*100/(METRICS['overall_battery_consumption'] + METRICS['overall_wasted_battery_consumption']) if (METRICS['overall_battery_consumption'] + METRICS['overall_wasted_battery_consumption']) > 0 else 0)
        battery_trends[CATEGORIES[bag]]["Planned Battery Usage"]["100%"].append(METRICS['overall_battery_consumption'] + METRICS['overall_wasted_battery_consumption'])
        battery_trends[CATEGORIES[bag]]["Extra Battery Usage"]["100%"].append(METRICS['overall_battery_consumption'] + METRICS['overall_wasted_battery_consumption'])
        battery_trends[CATEGORIES[bag]]["Wasted Battery Usage"]["100%"].append(METRICS['overall_battery_consumption'] + METRICS['overall_wasted_battery_consumption'])
        
        velocity_trends[CATEGORIES[bag]]["Avg Velocity"]["value"].append(METRICS['mean_average_velocity'])
        
        # SAFETY
        collision_trends[CATEGORIES[bag]]["Human Collisions"]["value"].append(METRICS['overall_human_collision'])
        clearance_trends[CATEGORIES[bag]]["Avg Clearing Distance"]["value"].append(METRICS['mean_average_clearing_distance'])
        
        denominator = METRICS['overall_space_compliance']['public'] + METRICS['overall_space_compliance']['social'] + METRICS['overall_space_compliance']['personal'] + METRICS['overall_space_compliance']['intimate']
        proxemics_trends[CATEGORIES[bag]]["Public"]["value"].append(METRICS['overall_space_compliance']['public']*100/denominator if denominator > 0 else 0)
        proxemics_trends[CATEGORIES[bag]]["Social"]["value"].append(METRICS['overall_space_compliance']['social']*100/denominator if denominator > 0 else 0)
        proxemics_trends[CATEGORIES[bag]]["Personal"]["value"].append(METRICS['overall_space_compliance']['personal']*100/denominator if denominator > 0 else 0)
        proxemics_trends[CATEGORIES[bag]]["Intimate"]["value"].append(METRICS['overall_space_compliance']['intimate']*100/denominator if denominator > 0 else 0)

time_labels = [tod.value for tod in TOD]

plot_stacked_bars_over_time(success_failure_trends, time_labels, "Success-Failure", "Count", outdir=OUTDIR)
plot_stacked_bars_over_time(working_time_trends, time_labels, "Task Time", "s", outdir=OUTDIR)
plot_stacked_bars_over_time(path_trends, time_labels, "Path Length", "m", outdir=OUTDIR)
plot_stacked_bars_over_time(battery_trends, time_labels, "Battery Usage", "%", outdir=OUTDIR)
plot_bars_over_time(velocity_trends, time_labels, "Velocity", "m/s", label_bar_pos=0.01, percentage=False, outdir=OUTDIR)
plot_bars_over_time(collision_trends, time_labels, "Collision", "Count", label_bar_pos=0.5, percentage=False, outdir=OUTDIR)
plot_bars_over_time(clearance_trends, time_labels, "Clearance Distance to Obstacles", "m", label_bar_pos=0.025, percentage=False, outdir=OUTDIR)
plot_bars_over_time(proxemics_trends, time_labels, "Proxemics", "Percentage (%)", outdir=OUTDIR)