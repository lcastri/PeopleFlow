import matplotlib.pyplot as plt
import numpy as np
import json
import os

INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
BAGNAMES = ['noncausal-03012025', 'causal-04012025']
CATEGORIES = {'noncausal-03012025': 'Non-Causal', 'causal-04012025': 'Causal'}
OUTDIR = os.path.join('/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/results', 'comparison', '__'.join(BAGNAMES), 'overall')
os.makedirs(OUTDIR, exist_ok=True)

# Initialize aggregated data structures
success_failure_metrics = {}
working_time_metrics = {}
path_metrics = {}
battery_metrics = {}
velocity_metrics = {}

# Load metrics for each bag
for bagname in BAGNAMES:
    metrics_path = os.path.join(INDIR, bagname, "metrics.json")
    with open(metrics_path, 'r') as json_file:
        METRICS = json.load(json_file)
    
    # EFFICIENCY 
    success_failure_metrics[bagname] = {
        "N. Success": {"value": METRICS['overall_success']*100/METRICS['task_count'], "color": "tab:blue"},
        "N. Failure (People)": {"value": METRICS['overall_failure_people']*100/METRICS['task_count'], "color": "tab:orange"},
        "N. Failure (Critical Battery)": {"value": METRICS['overall_failure_critical_battery']*100/METRICS['task_count'], "color": "tab:red"}
    }
    working_time_metrics[bagname] = {
        "Active Time": {"value": METRICS['overall_time_to_reach_goal']*100/METRICS['overall_task_time'], "color": "tab:blue"},
        "Stalled Time": {"value": METRICS['overall_stalled_time']*100/METRICS['overall_task_time'], "color": "tab:orange"},
        "Wasted Time": {"value": METRICS['overall_wasted_time_to_reach_goal']*100/METRICS['overall_task_time'], "color": "tab:red"},
    }
    path_metrics[bagname] = {
        "Planned Travelled Distance": {"value": METRICS['overall_path_length_only_success']*100/(METRICS['overall_travelled_distance']+METRICS['overall_wasted_travelled_distance']), "color": "tab:blue"},
        "Extra Travelled Distance": {"value": (METRICS['overall_travelled_distance'] - METRICS['overall_path_length_only_success'])*100/(METRICS['overall_travelled_distance']+METRICS['overall_wasted_travelled_distance']), "color": "tab:orange"},
        "Wasted Travelled Distance": {"value": METRICS['overall_wasted_travelled_distance']*100/(METRICS['overall_travelled_distance']+METRICS['overall_wasted_travelled_distance']), "color": "tab:red"},
    }
    battery_metrics[bagname] = {
        "Planned Battery Usage": {"value": METRICS['overall_planned_battery_consumption_only_success']*100/(METRICS['overall_battery_consumption']+METRICS['overall_wasted_battery_consumption']), "color": "tab:blue"},
        "Extra Battery Usage": {"value": (METRICS['overall_battery_consumption'] - METRICS['overall_planned_battery_consumption_only_success'])*100/(METRICS['overall_battery_consumption']+METRICS['overall_wasted_battery_consumption']), "color": "tab:orange"},
        "Wasted Battery Usage": {"value": METRICS['overall_wasted_battery_consumption']*100/(METRICS['overall_battery_consumption']+METRICS['overall_wasted_battery_consumption']), "color": "tab:red"},
    }
    # mean_battery_charging_metrics[bagname] = {
    #     "Mean Battery Level at Start Charging (%)": METRICS['mean_battery_at_start_charging'],
    #     "Mean Charging Time (s)": METRICS['mean_battery_charging_time'],
    # }
    velocity_metrics[bagname] = {
        # "Mean Min Velocity (m/s)": METRICS['mean_min_velocity'],
        "Avg Velocity": {"value": METRICS['mean_average_velocity'], "color": "tab:blue"},
        # "Mean Max Velocity (m/s)": METRICS['mean_max_velocity'],
    }
    
    # # SAFETY 
    # collision_metrics[bagname] = {
    #     "Human Collisions": METRICS['overall_human_collision'],
    #     "Robot Falls": METRICS['overall_robot_fallen'],
    # }
    # clearance_metrics[bagname] = {
    #     "Mean Min Clearing Distance (m)": METRICS['mean_min_clearing_distance'],
    #     "Mean Avg Clearing Distance (m)": METRICS['mean_average_clearing_distance'],
    #     "Mean Max Clearing Distance (m)": METRICS['mean_max_clearing_distance'],
    # }

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
        
        
def plot_stacked_bar(metrics_dict, title, ylabel, bar_width=0.2, offset = 0.01, yticks = None, figsize=(14, 9), outdir=None):
    # Categories and components
    categories = list(metrics_dict.keys())
    components = list(metrics_dict[categories[0]].keys())

    # Extracting values
    values = {component: [metrics_dict[cat][component]["value"] for cat in categories] for component in components}
    colors = {component: [metrics_dict[cat][component]["color"] for cat in categories] for component in components}

    # Create the bar chart
    x = [(bar_width + offset) * i for i in range(len(categories))]  # the label locations

    # Initialize bottom for stacking
    bottom = np.zeros(len(categories))

    # Plot each component
    plt.figure(figsize=figsize)
    for component, value in values.items():
        plt.bar(x, value, bar_width, label=component, bottom=bottom, color=colors[component])
        bottom += np.array(value)

    # Adjust x-axis and other labels
    plt.xticks(x, list(CATEGORIES.values()), fontsize=12)
    if yticks is not None: plt.yticks(yticks, fontsize=12)  # y ticks every 10
    plt.ylabel(ylabel, fontsize=14)
    plt.title(title, fontsize=16)

    # Move legend below the chart
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), fancybox=True, shadow=False, ncol=len(components))

    # Add grid for clarity
    plt.grid(axis='y', linestyle='--', alpha=0.5)

    # Display or save the plot
    if outdir is not None:
        plt.savefig(os.path.join(outdir, f"{title}.png"), dpi=300, bbox_inches='tight')
    else:
        plt.show()
        
        
def plot_efficiency(metrics_list, titles, ylabel, figsize=(20, 12), outdir=None):
    """
    Plots multiple stacked bar charts side by side in the same figure.
    
    Parameters:
    - metrics_list: List of dictionaries containing the metrics for each subplot.
    - titles: List of titles for the subplots.
    - ylabel: Common y-axis label.
    - figsize: Tuple representing the figure size.
    - outdir: Directory to save the figure, if specified.
    """
    n_plots = len(metrics_list)
    fig, axs = plt.subplots(1, n_plots, figsize=figsize, sharey=True)

    for i, (ax, metrics, title) in enumerate(zip(axs, metrics_list, titles)):
        # Categories and components
        categories = list(metrics.keys())
        components = list(metrics[categories[0]].keys())

        # Extract values and colors
        values = {component: [metrics[cat][component]["value"] for cat in categories] for component in components}
        colors = {component: [metrics[cat][component]["color"] for cat in categories] for component in components}

        # Define label positions
        bar_width = 0.1
        offset = 0.01
        x = [(bar_width + offset) * i for i in range(len(categories))]

        # Initialize bottom for stacking
        bottom = np.zeros(len(categories))

        # Plot each component
        for component, value in values.items():
            bars = ax.bar(x, value, bar_width, label=component, bottom=bottom, color=colors[component])
            bottom += np.array(value)
            
            # Annotate each bar segment with its percentage value
            for bar in bars:
                height = bar.get_height()
                if height > 0:
                    ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() / 2 + bar.get_y(),
                            f'{height:.2f}%', ha='center', va='center', fontsize=10, color='black')

        # Set x-axis labels
        ax.set_xticks(x)
        ax.set_xticklabels([CATEGORIES[cat] for cat in categories], fontsize=14)
        ax.set_title(title, fontsize=14)

        # Add grid for clarity
        ax.grid(axis='y', linestyle='--', alpha=0.5)
        ax.legend(fontsize=14, loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=1)
        ax.set_ylim(0, 105)
        
    # Set common y-axis and title
    fig.supylabel(ylabel, fontsize=16)
    fig.suptitle("Efficiency Metrics", fontsize=16)

    # Adjust layout
    fig.tight_layout(rect=[0, 0.05, 1, 0.95])

    # Save or display the plot
    if outdir is not None:
        output_path = os.path.join(outdir, "Efficiency.png")
        plt.savefig(output_path, dpi=300, bbox_inches="tight")
    else:
        plt.show()



# Plot all metrics
plot_stacked_bar(success_failure_metrics, "Success-Failure", "%", yticks=range(0, 105, 10), outdir=OUTDIR)
plot_stacked_bar(working_time_metrics, "Task Time", "%", yticks=range(0, 105, 10), outdir=OUTDIR)
plot_stacked_bar(path_metrics, "Path Length", "%", yticks=range(0, 105, 10), outdir=OUTDIR)
plot_stacked_bar(battery_metrics, "Battery Usage", "%", yticks=range(0, 105, 10), outdir=OUTDIR)
plot_stacked_bar(velocity_metrics, "Velocity", "m/s", outdir=OUTDIR)

metrics_list = [
    success_failure_metrics,
    working_time_metrics,
    path_metrics,
    battery_metrics,
]
titles = [
    "Success-Failure",
    "Task Time",
    "Path Length",
    "Battery Usage",
]

plot_efficiency(metrics_list, titles, ylabel="Percentage (%)", outdir=OUTDIR)