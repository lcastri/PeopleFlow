import itertools
import pickle
import os
import matplotlib.pyplot as plt
from utils import *
from metrics_utils import *


# Function to plot boxplots with p-value annotations
def plot_boxplot_with_p_value(data, title, ylabel, categories, p_value, background = False, outdir = None):
    fontsize = 20
    plt.figure(figsize=(10, 6))
    ax = plt.gca()

    plt.boxplot([data[bag][title] for bag in BAGNAMES], labels=[categories[bag] for bag in BAGNAMES])
    # plt.title(f"{title}\nP-Value: {p_value:.3e}")
    plt.title(f"{title}", fontdict={"fontsize": fontsize})
    # plt.title(f"{title} -- P-value: {p_value}", fontdict={"fontsize": fontsize})
    plt.ylabel(ylabel, fontdict={"fontsize": fontsize})
    plt.grid(axis='y', linestyle='--', alpha=0.6)
    plt.xticks(fontsize=fontsize)
    # plt.yticks(fontsize=fontsize)
    plt.ylim(0, 7.6 * 1.05)
    # plt.yticks(np.round(np.linspace(0, 7.6, num=6), 2), fontsize=fontsize)  # Adjust `num=6` as needed
    plt.yticks([0, 1, 2, 3, 4, 5, 6, 7, 8], fontsize=fontsize)  # Adjust `num=6` as needed
    # plt.yticks([0, 0.5, 1.2, 3.6, 7.6], fontsize=fontsize)  # Adjust `num=6` as needed

    if background:
        # Define bands and labels
        bands = [
            (0, 0.5, "Intimate", "tab:red"),
            (0.5, 1.2, "Personal", "tab:orange"),
            (1.2, 3.6, "Social", "tab:blue"),
            (3.6, 7.6, "Public", "tab:green"),
        ]

        # Add colored bands and labels
        for lower, upper, label, color in bands:
            ax.axhspan(lower, upper, color=color, alpha=0.3)
            ax.text(plt.xlim()[0] + 0.05, (lower + upper) / 2, label, 
                    va="center", ha="left", fontsize=fontsize-1, color="black")  # Left-aligned
        
    if outdir is not None:
        plt.savefig(os.path.join(outdir, f'{title.lower().replace(" ", "_")}_boxplot.png'))
    else:
        plt.show()
    plt.tight_layout()
    plt.close()

INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
BAGNAMES = ['base', 'causal']
CATEGORIES = {'base': 'Baseline', 'causal': 'Causal'}
OUTDIR = os.path.join('/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/results', 'comparison', '__'.join(BAGNAMES), 'boxplot')
os.makedirs(OUTDIR, exist_ok=True)

# Initialize aggregated data structures
working_time_metrics = {bagname: {"Active Time": [], "Stalled Time": [], "Wasted Time": []} for bagname in BAGNAMES}
path_metrics = {bagname: {"Planned Travelled Distance": [], "Extra Travelled Distance": [], "Wasted Travelled Distance": []} for bagname in BAGNAMES}
battery_metrics = {bagname: {"Planned Battery Usage": [], "Extra Battery Usage": [], "Wasted Battery Usage": []} for bagname in BAGNAMES}
proxemics_metrics = {bagname: {"Human-Robot Proxemic Compliance": []} for bagname in BAGNAMES}

# Load metrics for each bag
for bagname in BAGNAMES:
    metrics_path = os.path.join(INDIR, bagname, "metrics.pkl")
    with open(metrics_path, 'rb') as pkl_file:
        METRICS = pickle.load(pkl_file)
    for tod in TOD:
        metrics_tod = METRICS[tod.value]
        for task in metrics_tod.keys():
            try:
                if isinstance(int(task), int):
                    # path_metrics[bagname]["Planned Travelled Distance"].append(metrics_tod[task]['path_length'])
                    # battery_metrics[bagname]["Planned Battery Usage"].append(metrics_tod[task]['planned_battery_consumption'])
                    # if metrics_tod[task]['result'] == 1:
                    #     working_time_metrics[bagname]["Active Time"].append(metrics_tod[task]['time_to_reach_goal'])
                    #     working_time_metrics[bagname]["Stalled Time"].append(metrics_tod[task]['stalled_time'])
                    #     path_metrics[bagname]["Extra Travelled Distance"].append(metrics_tod[task]['travelled_distance'] - metrics_tod[task]['path_length'])
                    #     battery_metrics[bagname]["Extra Battery Usage"].append(metrics_tod[task]['battery_consumption'] - metrics_tod[task]['planned_battery_consumption'])
                    # else:
                    #     working_time_metrics[bagname]["Wasted Time"].append(metrics_tod[task]['wasted_time_to_reach_goal'])
                    #     path_metrics[bagname]["Wasted Travelled Distance"].append(metrics_tod[task]['wasted_travelled_distance'])
                    #     battery_metrics[bagname]["Wasted Battery Usage"].append(metrics_tod[task]['wasted_battery_consumption'])
                    proxemics_metrics[bagname]["Human-Robot Proxemic Compliance"].extend(
                        value for value in itertools.chain(*metrics_tod[task]['agent_distances'].values()) if value < 7.6
                    )
            except ValueError:
                continue

# Plot all metrics as boxplot with p-values
metrics = {
    # "working_time_metrics": ["Active Time", "Stalled Time", "Wasted Time"],
    # "path_metrics": ["Planned Travelled Distance", "Extra Travelled Distance", "Wasted Travelled Distance"],
    # "battery_metrics": ["Planned Battery Usage", "Extra Battery Usage", "Wasted Battery Usage"],
    "proxemics_metrics": ["Human-Robot Proxemic Compliance"],
}

for metric_type, metric_list in metrics.items():
    print(metric_type)
    for metric_name in metric_list:
        data = eval(metric_type)  # Get the data dictionary dynamically
        # p_value, test_used = compute_p_values(data, metric_name)
        plot_boxplot_with_p_value(data, metric_name, "m", CATEGORIES, "****", background = metric_type == 'proxemics_metrics', outdir=OUTDIR)