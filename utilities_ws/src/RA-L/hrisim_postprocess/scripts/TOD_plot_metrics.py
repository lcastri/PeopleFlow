import matplotlib.pyplot as plt
import numpy as np
import json
import os

INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/TOD/original'
BAGNAMES = ['noncausal_13122024', 'causal_18122024']
# BAGNAMES = ['noncausal_13122024']

# Initialize aggregated data structures
aggregated_metrics = {}
time_metrics = {}
velocity_metrics = {}
distance_metrics = {}
human_related_metrics = {}
space_compliance_metrics = {}

# Load metrics for each bag
for bagname in BAGNAMES:
    metrics_path = os.path.join(INDIR, bagname, "metrics.json")
    with open(metrics_path, 'r') as json_file:
        METRICS = json.load(json_file)
        
    aggregated_metrics[bagname] = {
        "Overall Success": METRICS['overall_success'],
        "Overall Failure": METRICS['overall_failure'],
        "Mean Battery Charging Time (s)": METRICS['mean_battery_charging_time'],
        "Mean Battery Level at Start Charging (%)": METRICS['mean_battery_at_start_charging'],
    }

    time_metrics[bagname] = {
        "Mean Stalled Time (s)": METRICS['mean_stalled_time'],
        "Mean Time to Goal (s)": METRICS['mean_time_to_reach_goal'],
        "Mean Path Length (m)": METRICS['mean_path_length']
    }

    velocity_metrics[bagname] = {
        "Mean Min Velocity (m/s)": METRICS['mean_min_velocity'],
        "Mean Max Velocity (m/s)": METRICS['mean_max_velocity'],
        "Mean Avg Velocity (m/s)": METRICS['mean_average_velocity']
    }

    distance_metrics[bagname] = {
        "Mean Min Clearing Distance (m)": METRICS['mean_min_clearing_distance'],
        "Mean Max Clearing Distance (m)": METRICS['mean_max_clearing_distance'],
        "Mean Avg Clearing Distance (m)": METRICS['mean_average_clearing_distance'],
    }

    human_related_metrics[bagname] = {
        "Human Collisions": METRICS['overall_human_collision'],
        "Mean Min Distance to Humans (m)": METRICS['mean_min_distance_to_humans'],
    }

    space_compliance_metrics[bagname] = {
        "Mean Intimate Space Compliance": METRICS['mean_space_compliance']['intimate'],
        "Mean Personal Space Compliance": METRICS['mean_space_compliance']['personal'],
        "Mean Social Space Compliance": METRICS['mean_space_compliance']['social'],
        "Mean Public Space Compliance": METRICS['mean_space_compliance']['public'],
    }

# Helper function for grouped bar plots
def plot_grouped_bar(metrics_dict, title, ylabel, figsize=(10, 6), palette="Set2"):
    metric_categories = list(metrics_dict[BAGNAMES[0]].keys())
    x = np.arange(len(metric_categories))
    width = 0.35

    plt.figure(figsize=figsize)
    for i, bagname in enumerate(BAGNAMES):
        values = list(metrics_dict[bagname].values())
        plt.bar(x + i * width, values, width, label=bagname)

    plt.xticks(x + width / 2, metric_categories)
    plt.title(title)
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid()
    plt.tight_layout()

# Plot all metrics
plot_grouped_bar(aggregated_metrics, "Overall Aggregated Metrics", "Count")
plot_grouped_bar(time_metrics, "Time-Related Metrics", "Seconds / Meters")
plot_grouped_bar(velocity_metrics, "Velocity Metrics", "m/s")
plot_grouped_bar(distance_metrics, "Distance Metrics", "Meters")
plot_grouped_bar(space_compliance_metrics, "Space Compliance Metrics", "Percentage")
plt.show()
