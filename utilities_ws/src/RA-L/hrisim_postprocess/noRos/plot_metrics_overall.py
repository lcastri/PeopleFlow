import matplotlib.pyplot as plt
import numpy as np
import json
import os

INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
BAGNAMES = ['noncausal-test-02012025']
# BAGNAMES = ['noncausal_27122024', 'causal_30122024']
OUTDIR = os.path.join('/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original', 'comparison', '__'.join(BAGNAMES), 'overall')
os.makedirs(OUTDIR, exist_ok=True)

# Initialize aggregated data structures
aggregated_metrics = {}
battery_metrics = {}
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
        "Mean Travelled Distance (m)": METRICS['mean_travelled_distance'],
    }

    velocity_metrics[bagname] = {
        "Mean Min Velocity (m/s)": METRICS['mean_min_velocity'],
        "Mean Avg Velocity (m/s)": METRICS['mean_average_velocity'],
        "Mean Max Velocity (m/s)": METRICS['mean_max_velocity'],
    }

    distance_metrics[bagname] = {
        "Mean Min Clearing Distance (m)": METRICS['mean_min_clearing_distance'],
        "Mean Avg Clearing Distance (m)": METRICS['mean_average_clearing_distance'],
        "Mean Max Clearing Distance (m)": METRICS['mean_max_clearing_distance'],
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
plot_grouped_bar(aggregated_metrics, "Success Failure Metrics", "Count", outdir=OUTDIR)
plot_grouped_bar(time_metrics, "Time Distance-Related Metrics", "s / m ", outdir=OUTDIR)
plot_grouped_bar(velocity_metrics, "Velocity-Related Metrics", "m/s ", outdir=OUTDIR)
plot_grouped_bar(distance_metrics, "Clearence-Related Metrics", "m ", outdir=OUTDIR)
plot_grouped_bar(human_related_metrics, "Human-related Metrics", "Count / m", outdir=OUTDIR)
plot_grouped_bar(space_compliance_metrics, "Space Compliance Metrics", "Percentage", outdir=OUTDIR)
