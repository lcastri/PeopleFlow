import matplotlib.pyplot as plt
import numpy as np
import json
import os

INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
BAGNAMES = ['noncausal-03012025', 'causal-04012025']
OUTDIR = os.path.join('/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original', 'comparison', '__'.join(BAGNAMES), 'overall')
os.makedirs(OUTDIR, exist_ok=True)

# Initialize aggregated data structures
success_failure_metrics = {}
mean_time_metrics = {}
overall_time_metrics = {}
mean_path_metrics = {}
mean_battery_metrics = {}
mean_battery_charging_metrics = {}
velocity_metrics = {}
collision_metrics = {}
clearance_metrics = {}
space_compliance_metrics = {}

# Load metrics for each bag
for bagname in BAGNAMES:
    metrics_path = os.path.join(INDIR, bagname, "metrics.json")
    with open(metrics_path, 'r') as json_file:
        METRICS = json.load(json_file)
    
    # EFFICIENCY 
    success_failure_metrics[bagname] = {
        "N. Tasks": METRICS['task_count'],
        "Overall Success": METRICS['overall_success'],
        "Overall Failure": METRICS['overall_failure'],
        "Overall Failure (People)": METRICS['overall_failure_people'],
        "Overall Failure (Critical Battery)": METRICS['overall_failure_critical_battery']
    }
    mean_time_metrics[bagname] = {
        "Mean Stalled Time (s)": METRICS['mean_stalled_time'],
        "Mean Time to Goal (s)": METRICS['mean_time_to_reach_goal'],
        "Mean Wasted Time (s)": METRICS['mean_wasted_time_to_reach_goal'],
    }
    overall_time_metrics[bagname] = {
        "Overall Stalled Time (s)": METRICS['overall_stalled_time'],
        "Overall Time to Goal (s)": METRICS['overall_time_to_reach_goal'],
        "Overall Wasted Time (s)": METRICS['overall_wasted_time_to_reach_goal'],
    }
    mean_path_metrics[bagname] = {
        "Mean Path Length (m)": METRICS['mean_path_length'],
        "Mean Travelled Distance (m)": METRICS['mean_travelled_distance'],
        "Mean Wasted Travelled Distance (m)": METRICS['mean_wasted_travelled_distance'],
    }
    mean_battery_metrics[bagname] = {
        "Mean Planned Battery Consumption (%)": METRICS['mean_planned_battery_consumption'],
        "Mean Battery Consumption (%)": METRICS['mean_battery_consumption'],
        "Mean Wasted Battery Consumption (%)": METRICS['mean_wasted_battery_consumption'],
    }
    mean_battery_charging_metrics[bagname] = {
        "Mean Battery Level at Start Charging (%)": METRICS['mean_battery_at_start_charging'],
        "Mean Charging Time (s)": METRICS['mean_battery_charging_time'],
    }
    velocity_metrics[bagname] = {
        "Mean Min Velocity (m/s)": METRICS['mean_min_velocity'],
        "Mean Avg Velocity (m/s)": METRICS['mean_average_velocity'],
        "Mean Max Velocity (m/s)": METRICS['mean_max_velocity'],
    }
    
    # SAFETY 
    collision_metrics[bagname] = {
        "Human Collisions": METRICS['overall_human_collision'],
        "Robot Falls": METRICS['overall_robot_fallen'],
    }
    clearance_metrics[bagname] = {
        "Mean Min Clearing Distance (m)": METRICS['mean_min_clearing_distance'],
        "Mean Avg Clearing Distance (m)": METRICS['mean_average_clearing_distance'],
        "Mean Max Clearing Distance (m)": METRICS['mean_max_clearing_distance'],
    }

# Helper function for grouped bar plots
def plot_grouped_bar(metrics_dict, title, ylabel, figsize=(14, 8), outdir=None):
    metric_categories = list(metrics_dict[BAGNAMES[0]].keys())
    x = np.arange(len(metric_categories))
    width = 0.35

    plt.figure(figsize=figsize)
    for i, bagname in enumerate(BAGNAMES):
        values = list(metrics_dict[bagname].values())
        # values = [val if val is not None else 0 for val in metrics_dict[bagname].values()]  # Replace None with 0
        plt.bar(x + i * width, values, width, label=bagname)

    plt.xticks(x + width / 2, metric_categories)
    plt.title(title)
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid()
    plt.tight_layout()

    if outdir is not None:
        plt.savefig(os.path.join(outdir, f"{title}.png"), dpi=300, bbox_inches='tight')
    else:
        plt.show()


# Plot all metrics
plot_grouped_bar(success_failure_metrics, "Success Failure Metrics", "Count", outdir=OUTDIR)
plot_grouped_bar(mean_time_metrics, "(Mean) Time Metrics", "s", outdir=OUTDIR)
plot_grouped_bar(overall_time_metrics, "(Overall) Time Metrics", "s", outdir=OUTDIR)
plot_grouped_bar(mean_path_metrics, "Path Metrics", "m", outdir=OUTDIR)
plot_grouped_bar(mean_battery_metrics, "Battery Metrics", "%", outdir=OUTDIR)
plot_grouped_bar(mean_battery_charging_metrics, "Battery Metrics", "%", outdir=OUTDIR)
plot_grouped_bar(velocity_metrics, "Velocity Metrics", "m/s", outdir=OUTDIR)
plot_grouped_bar(collision_metrics, "Collision Metrics", "Count", outdir=OUTDIR)
plot_grouped_bar(clearance_metrics, "Clearance Metrics", "m", outdir=OUTDIR)