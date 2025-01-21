import itertools
import json
import os
import matplotlib.pyplot as plt
import seaborn as sns
from utils import *
from metrics_utils import *
from scipy.stats import shapiro, mannwhitneyu, ttest_ind, normaltest
import pandas as pd
import numpy as np

def is_normal(data, alpha=0.05):
    # Automatically choose test based on sample size
    if len(data) <= 5000:
        # Shapiro-Wilk test for small datasets
        stat, p = shapiro(data)
    else:
        # D'Agostino and Pearson's test for larger datasets
        stat, p = normaltest(data)

    # Plot density for visual inspection
    # plt.figure(figsize=(8, 5))
    # sns.kdeplot(data, fill=True, color='skyblue', alpha=0.5)
    # plt.title(f'Density Plot (p={p:.3f}, Normal: {p > alpha})')
    # plt.xlabel('Values')
    # plt.ylabel('Density')
    # plt.grid(True)
    # plt.show()

    return p > alpha

# Function to compute p-values with the appropriate test
def compute_p_values_with_test(data, metric_name, alpha=0.05):
    noncausal_data = np.array(data['noncausal-11012025'][metric_name])
    causal_data = np.array(data['causal-12012025'][metric_name])

    # Check normality for both groups
    noncausal_normal = is_normal(noncausal_data, alpha)
    causal_normal = is_normal(causal_data, alpha)

    # Select appropriate test
    if noncausal_normal and causal_normal:
        stat, p_value = ttest_ind(noncausal_data, causal_data, equal_var=False)
        test_used = "T-Test"
    else:
        stat, p_value = mannwhitneyu(noncausal_data, causal_data, alternative='two-sided')
        test_used = "Mann-Whitney U Test"

    return p_value, test_used

# Function to plot boxplots with p-value annotations
def plot_boxplot_with_p_value(data, title, ylabel, categories, p_value, background = False, outdir = None):
    plt.figure(figsize=(10, 6))
    ax = plt.gca()

    plt.boxplot([data[bag][title] for bag in BAGNAMES], labels=[categories[bag] for bag in BAGNAMES])
    plt.title(f"{title}\nP-Value: {p_value:.3e}")
    plt.ylabel(ylabel)
    plt.grid(True)
    if background:  
        ax.axhspan(0, 0.5, color="tab:red", alpha=0.3)
        ax.axhspan(0.5, 1.2, color="tab:orange", alpha=0.3)
        ax.axhspan(1.2, 3.6, color="tab:blue", alpha=0.3)
        ax.axhspan(3.6, 7.6, color="tab:green", alpha=0.3)
        
    if outdir is not None:
        plt.savefig(os.path.join(outdir, f'{title.lower().replace(" ", "_")}_boxplot.png'))
    else:
        plt.show()
    plt.close()

INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
BAGNAMES = ['noncausal-11012025', 'causal-12012025']
CATEGORIES = {'noncausal-11012025': 'Non-Causal', 'causal-12012025': 'Causal'}
OUTDIR = os.path.join('/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/results', 'comparison', '__'.join(BAGNAMES), 'boxplot')
os.makedirs(OUTDIR, exist_ok=True)

# Initialize aggregated data structures
working_time_metrics = {bagname: {"Active Time": [], "Stalled Time": [], "Wasted Time": []} for bagname in BAGNAMES}
path_metrics = {bagname: {"Planned Travelled Distance": [], "Extra Travelled Distance": [], "Wasted Travelled Distance": []} for bagname in BAGNAMES}
battery_metrics = {bagname: {"Planned Battery Usage": [], "Extra Battery Usage": [], "Wasted Battery Usage": []} for bagname in BAGNAMES}
proxemics_metrics = {bagname: {"Distances": []} for bagname in BAGNAMES}

# Load metrics for each bag
for bagname in BAGNAMES:
    metrics_path = os.path.join(INDIR, bagname, "metrics.json")
    with open(metrics_path, 'r') as json_file:
        METRICS = json.load(json_file)
    for tod in TOD:
        metrics_tod = METRICS[tod.value]
        for task in metrics_tod.keys():
            try:
                if isinstance(int(task), int):
                    path_metrics[bagname]["Planned Travelled Distance"].append(metrics_tod[task]['path_length'])
                    battery_metrics[bagname]["Planned Battery Usage"].append(metrics_tod[task]['planned_battery_consumption'])
                    if metrics_tod[task]['result'] == 1:
                        working_time_metrics[bagname]["Active Time"].append(metrics_tod[task]['time_to_reach_goal'])
                        working_time_metrics[bagname]["Stalled Time"].append(metrics_tod[task]['stalled_time'])
                        path_metrics[bagname]["Extra Travelled Distance"].append(metrics_tod[task]['travelled_distance'] - metrics_tod[task]['path_length'])
                        battery_metrics[bagname]["Extra Battery Usage"].append(metrics_tod[task]['battery_consumption'] - metrics_tod[task]['planned_battery_consumption'])
                    else:
                        working_time_metrics[bagname]["Wasted Time"].append(metrics_tod[task]['wasted_time_to_reach_goal'])
                        path_metrics[bagname]["Wasted Travelled Distance"].append(metrics_tod[task]['wasted_travelled_distance'])
                        battery_metrics[bagname]["Wasted Battery Usage"].append(metrics_tod[task]['wasted_battery_consumption'])
                    proxemics_metrics[bagname]["Distances"].extend(
                        value for value in itertools.chain(*metrics_tod[task]['agent_distances'].values()) if value < 7.6
                    )
            except ValueError:
                continue

# Plot all metrics as boxplot with p-values
metrics = {
    "working_time_metrics": ["Active Time", "Stalled Time", "Wasted Time"],
    "path_metrics": ["Planned Travelled Distance", "Extra Travelled Distance", "Wasted Travelled Distance"],
    "battery_metrics": ["Planned Battery Usage", "Extra Battery Usage", "Wasted Battery Usage"],
    "proxemics_metrics": ["Distances"],
}

for metric_type, metric_list in metrics.items():
    print(metric_type)
    for metric_name in metric_list:
        data = eval(metric_type)  # Get the data dictionary dynamically
        p_value, test_used = compute_p_values_with_test(data, metric_name)
        plot_boxplot_with_p_value(data, metric_name, "Value", CATEGORIES, p_value, background = metric_type == 'proxemics_metrics', outdir=OUTDIR)