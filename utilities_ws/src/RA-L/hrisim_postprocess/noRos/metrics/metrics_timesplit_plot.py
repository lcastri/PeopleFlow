import json
import os
import pickle
from utils import *
from metrics_utils import *

INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
BAGNAMES = ['base', 'causal']
CATEGORIES = {'base': 'Baseline', 'causal': 'Causal'}
OUTDIR = os.path.join('/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/results', 'comparison', '__'.join(BAGNAMES), 'timesplit')
os.makedirs(OUTDIR, exist_ok=True)

# Initialize aggregated data structures
success_failure_metrics = {}
working_time_metrics = {}
travelled_distance_metrics = {}
battery_metrics = {}
velocity_metrics = {}
collision_metrics = {}
clearance_metrics = {}
proxemics_metrics = {}

pvalues_path = os.path.join(OUTDIR, "pvalues.pkl")
with open(pvalues_path, 'rb') as pkl_file:
    PVALUES = pickle.load(pkl_file)

# Load metrics for each bag
for tod in TOD:
    for bagname in BAGNAMES:
        metrics_path = os.path.join(INDIR, bagname, "metrics.pkl")
        with open(metrics_path, 'rb') as pkl_file:
            METRICS = pickle.load(pkl_file)
        
        METRICS = METRICS[tod.value]
            
        # EFFICIENCY 
        #! Success & Failures
        success_failure_metrics[bagname] = {
            "Success": {"value": METRICS['overall_success'],
                        "%": METRICS['overall_success']*100/METRICS['task_count'],
                        "100%": METRICS['task_count'],
                        "color": "tab:blue",
                        "p-value": PVALUES["N. Success"]["p_value"]},
            "Failures~(D)": {"value": METRICS['overall_failure_people'],
                                    "%": METRICS['overall_failure_people']*100/METRICS['task_count'], 
                                    "100%": METRICS['task_count'],
                                    "color": "tab:orange",
                                    "p-value": PVALUES["N. Failures (People)"]["p_value"]},
            "Failures~(L)": {"value": METRICS['overall_failure_critical_battery'], 
                                            "%": METRICS['overall_failure_critical_battery']*100/METRICS['task_count'], 
                                            "100%": METRICS['task_count'],
                                            "color": "tab:red",
                                            "p-value": PVALUES["N. Failures (Critical Battery)"]["p_value"]}
        }
        
        #! Task Time
        total_time = METRICS['overall_task_time']/3600
        active_time = METRICS['overall_time_to_reach_goal_actual']/3600
        stalled_time = METRICS['overall_time_to_reach_goal_stalled']/3600
        wasted_time = METRICS['overall_time_to_reach_goal_wasted']/3600
        working_time_metrics[bagname] = {
            "Active": {"value": active_time, 
                            "%": active_time*100/total_time,
                            "100%": total_time,
                            "color": "tab:blue",
                            "p-value": PVALUES["Active Time"]["p_value"]},
            "Stalled": {"value": stalled_time, 
                            "%": stalled_time*100/total_time, 
                            "100%": total_time,
                            "color": "tab:orange",
                            "p-value": PVALUES["Stalled Time"]["p_value"]},
            "Wasted": {"value": wasted_time, 
                            "%": wasted_time*100/total_time, 
                            "100%": total_time,
                            "color": "tab:red",
                            "p-value": PVALUES["Wasted Time"]["p_value"]},
        }
        
        #! Travelled Distance
        total_distance = (METRICS['overall_travelled_distance_actual'] + METRICS['overall_travelled_distance_wasted'])/1000
        planned_distance = METRICS['overall_travelled_distance_planned_only_success']/1000
        extra_distance = (METRICS['overall_travelled_distance_actual'] - METRICS['overall_travelled_distance_planned_only_success'])/1000
        wasted_distance = METRICS['overall_travelled_distance_wasted']/1000
        travelled_distance_metrics[bagname] = {
            "Planned": {"value": planned_distance, 
                                        "%": planned_distance*100/total_distance,
                                        "100%": total_distance,
                                        "color": "tab:blue",
                                        "p-value": PVALUES["Planned Travelled Distance"]["p_value"]},
            "Extra": {"value": extra_distance,
                                        "%": extra_distance*100/total_distance, 
                                        "100%": total_distance,
                                        "color": "tab:orange",
                                        "p-value": PVALUES["Extra Travelled Distance"]["p_value"]},
            "Wasted": {"value": wasted_distance,
                                        "%": wasted_distance*100/total_distance,  
                                        "100%": total_distance,
                                        "color": "tab:red",
                                        "p-value": PVALUES["Wasted Travelled Distance"]["p_value"]},
        }
        
        #! Battery
        # Define total battery reference for normalization
        total_battery = METRICS['overall_battery_consumption_actual'] + METRICS['overall_battery_consumption_wasted']

        # Get planned and actual battery consumption
        planned_battery = METRICS['overall_battery_consumption_planned_only_success']/100
        actual_battery = METRICS['overall_battery_consumption_actual']/100
        wasted_battery = METRICS['overall_battery_consumption_wasted']/100

        # Compute absolute deviation (ignoring sign)
        absolute_deviation = abs(METRICS['overall_battery_consumption_actual'] -  METRICS['overall_battery_consumption_planned_only_success'])

        # Use max(planned, actual) as the reference to normalize percentages
        reference_total = (METRICS['overall_battery_consumption_actual'] + METRICS['overall_battery_consumption_wasted'])/100

        # Update battery metrics dictionary
        battery_metrics[bagname] = {
            "Effective": {
                "value": actual_battery,
                "%": actual_battery * 100 / reference_total,
                "100%": reference_total,
                "color": "tab:blue",
                "p-value": PVALUES["Effective Battery Usage"]["p_value"]
            },
            "Wasted": {
                "value": wasted_battery,
                "%": wasted_battery * 100 / reference_total,
                "100%": reference_total,
                "color": "tab:red",
                "p-value": PVALUES["Wasted Battery Usage"]["p_value"]
            },
        }
        
        collision_metrics[bagname] = METRICS['overall_human_collision']


    # Plot all metrics
    plot_stacked_bar(success_failure_metrics, "Success-Failure", "Count", CATEGORIES,tod=tod.value, outdir=OUTDIR, step=300)
    plot_stacked_bar(working_time_metrics, "Task Time", "h", CATEGORIES, tod=tod.value, outdir=OUTDIR, step=5)
    plot_stacked_bar(travelled_distance_metrics, "Travelled Distance", "km", CATEGORIES, tod=tod.value, outdir=OUTDIR, step=5)
    plot_stacked_bar(battery_metrics, "Battery Usage", "Battery Cycles", CATEGORIES, tod=tod.value, outdir=OUTDIR, step=2)
    
    plt.figure(figsize=(10, 8))
    x_pos = [0.1, 0.21]
    colors = ["tab:blue", "tab:orange"]
    fontsize = 15  # Define fontsize

    # Create bars and store them in a variable
    bars = plt.bar(x_pos, collision_metrics.values(), width=0.1, color=colors)

    # Set labels and title
    plt.xticks(x_pos, CATEGORIES.values(), fontsize=fontsize)
    plt.yticks(fontsize=fontsize)
    plt.grid(axis='y', linestyle='--', alpha=0.6)
    plt.title(f"{tod.value} -- Human Collision", fontsize=fontsize)
    plt.ylim(0, max(collision_metrics.values()) * 1.2)  # Give some space above bars
    plt.tight_layout()

    # Annotate bars
    for bar, value in zip(bars, collision_metrics.values()):
        plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height() / 2,
                f"{value:.2f}", ha='center', va='center', fontsize=fontsize)
    plt.savefig(f"{OUTDIR}/{tod.value}-Human_Collision.png", dpi=300, bbox_inches="tight")
    plt.close()