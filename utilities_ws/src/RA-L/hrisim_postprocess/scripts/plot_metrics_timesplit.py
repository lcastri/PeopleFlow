import matplotlib.pyplot as plt
import numpy as np
import json
import os
from utils import *

# Helper function for grouped bar plots
def plot_grouped_bar(metrics_dict, title, ylabel, figsize=(14, 8), palette="Set2", outdir=None):
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

INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
OUTDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
BAGNAMES = ['noncausal_27122024']

# Initialize aggregated data structures
aggregated_metrics = {}
battery_metrics = {}
time_metrics = {}
velocity_metrics = {}
distance_metrics = {}
human_related_metrics = {}
space_compliance_metrics = {}

# Load metrics for each bag
for tod in TOD:
    for bagname in BAGNAMES:
        metrics_path = os.path.join(INDIR, bagname, "metrics_timesplit.json")
        with open(metrics_path, 'r') as json_file:
            METRICS = json.load(json_file)
        
        METRICS = METRICS[tod.value]
            
        aggregated_metrics[bagname] = {
            "Overall Success": METRICS['overall_success'],
            "Overall Failure": METRICS['overall_failure']
        }
        
        # battery_metrics[bagname] = {
        #     "Mean Battery Charging Time (s)": METRICS['mean_battery_charging_time'],
        #     "Mean Battery Level at Start Charging (%)": METRICS['mean_battery_at_start_charging'],
        # }

        time_metrics[bagname] = {
            "Mean Stalled Time (s)": METRICS['mean_stalled_time'],
            "Mean Time to Goal (s)": METRICS['mean_time_to_reach_goal'],
            "Mean Path Length (m)": METRICS['mean_path_length'],
            "Mean Avg Velocity (m/s)": METRICS['mean_average_velocity'],
            "Mean Avg Clearing Distance (m)": METRICS['mean_average_clearing_distance'],
        }

        # velocity_metrics[bagname] = {
        #     "Mean Min Velocity (m/s)": METRICS['mean_min_velocity'],
        #     "Mean Max Velocity (m/s)": METRICS['mean_max_velocity'],
        # }

        # distance_metrics[bagname] = {
        #     "Mean Min Clearing Distance (m)": METRICS['mean_min_clearing_distance'],
        #     "Mean Max Clearing Distance (m)": METRICS['mean_max_clearing_distance'],
        # }

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

    #   Plot all metrics
    plot_grouped_bar(aggregated_metrics, f"{tod.value.capitalize()} -- Overall Aggregated Metrics", "Count", outdir=OUTDIR)
    plot_grouped_bar(time_metrics, f"{tod.value.capitalize()} -- Time-Related Metrics", "Seconds / Meters", outdir=OUTDIR)
    # plot_grouped_bar(velocity_metrics, f"{tod.value.capitalize()} -- Velocity Metrics", "m/s", outdir=OUTDIR)
    # plot_grouped_bar(distance_metrics, f"{tod.value.capitalize()} -- Distance Metrics", "Meters", outdir=OUTDIR)
    plot_grouped_bar(human_related_metrics, f"{tod.value.capitalize()} -- Human Distance Metrics", "Count", outdir=OUTDIR)
    plot_grouped_bar(space_compliance_metrics, f"{tod.value.capitalize()} -- Space Compliance Metrics", "Percentage", outdir=OUTDIR)
