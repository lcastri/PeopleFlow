import itertools
import pickle
import os
import json  # --- NEW CODE ---
from matplotlib import pyplot as plt
import numpy as np
from utils import *


# --- NEW CODE ---
def calculate_boxplot_stats(data_list):
    """
    Calculates descriptive statistics for a list of proxemics data.
    Handles empty lists by returning NaNs.
    """
    if not data_list:
        return {
            'mean': np.nan,
            'min': np.nan,
            'q1_25perc': np.nan,
            'median_50perc': np.nan,
            'q3_75perc': np.nan,
            'max': np.nan,
            'count': 0
        }
    
    data_array = np.asarray(data_list, dtype=float)
    q1, median, q3 = np.percentile(data_array, [25, 50, 75])
    
    return {
        'mean': np.mean(data_array),
        'min': np.min(data_array),
        'q1_25perc': q1,
        'median_50perc': median,
        'q3_75perc': q3,
        'max': np.max(data_array),
        'count': len(data_array)
    }
# --- END NEW CODE ---


def plot_boxplot(data, title, ylabel, categories, background = False, outdir = None):
    fontsize = 20
    plt.figure(figsize=(22, 10))
    ax = plt.gca()

    # --- MODIFIED: Handle empty lists for plotting ---
    # Replace empty lists with [np.nan] to avoid ValueError
    box_data = [(np.asarray(data[bag][title], dtype=float).ravel().tolist() if data[bag][title] else [np.nan]) for bag in BAGNAMES]
    
    plt.boxplot(
        box_data,
        labels=[categories[bag] for bag in BAGNAMES],
        positions=list(range(2, len(BAGNAMES) + 2))
    )

    plt.title(f"{title}", fontdict={"fontsize": fontsize})
    plt.ylabel(ylabel, fontdict={"fontsize": fontsize})
    plt.grid(axis='y', linestyle='--', alpha=0.6)
    plt.xticks(fontsize=15, rotation=90)
    plt.xlim(0.5, len(BAGNAMES) + 1.5)
    plt.ylim(0, 7.6 * 1.05)
    plt.yticks([0, 1, 2, 3, 4, 5, 6, 7, 8], fontsize=fontsize)

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
            ax.text(ax.get_xlim()[0] + 0.05, (lower + upper) / 2, label, 
                    va="center", ha="left", fontsize=fontsize-1, color="black")
    plt.tight_layout()
    if outdir is not None:
        os.makedirs(outdir, exist_ok=True)
        plt.savefig(os.path.join(outdir, f'{title.lower().replace(" ", "_")}_boxplot.png'))
    else:
        plt.show()
        
    plt.close()

def plot_stacked_bar(metrics_dict, title, ylabel, xTickLabel, bar_width=0.1, offset=0.01, figsize=(22, 10),
                     fontsize=20, step=10, tod=None, outdir=None, noPerc=False):
    # Categories and components
    categories = list(metrics_dict.keys())
    components = list(metrics_dict[categories[0]].keys())

    # Extracting values and colors for stacked bars
    values = {component: [metrics_dict[cat][component]["value"] for cat in categories] for component in components}
    if not noPerc:
        percs = {component: [metrics_dict[cat][component]["%"] for cat in categories] for component in components}
    colors = {component: [metrics_dict[cat][component]["color"] for cat in categories] for component in components}


    # Define the bar positions
    x_stacked = [i*(bar_width + offset) for i in range(len(categories))]
    x_ticks = x_stacked
    fig, ax = plt.subplots(figsize=figsize)

    # Plot the stacked bars
    bottom = np.zeros(len(categories))
    for component in components:
        bars = ax.bar(x_stacked, values[component], bar_width, label=rf"${component}$", bottom=bottom, color=colors[component])
        bottom += np.array(values[component])
        # Add percentage annotation for stacked bars
        for i, bar in enumerate(bars):
            height = bar.get_height()
            if height > 0:
                if not noPerc:
                    ax.text(bar.get_x() + bar.get_width() / 2, bar.get_y() + height / 2,
                            f"{percs[component][i]:.1f}%", ha='center', va='center', fontsize=15, rotation=90)
                else:
                    ax.text(bar.get_x() + bar.get_width() / 2, bar.get_y() + height / 2,
                            f"{values[component][i]:.2f}", ha='center', va='center', fontsize=15, rotation=90)

    # Configure x-axis and labels
    ax.set_xticks(x_ticks)
    ax.set_xticklabels([xTickLabel[cat] for cat in categories], fontsize=15, rotation=90)
    ax.tick_params(axis='y', labelsize=fontsize)

    ax.set_ylabel(ylabel, fontsize=fontsize)
    ax.set_ylim(0, max(bottom) * 1.1)
    ax.set_yticks(np.arange(0, max(bottom) + step, step))
    ax.set_title(f"{tod} -- {title}" if tod is not None else title, fontsize=fontsize)

    # Add grid and legend
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=len(components), fontsize=fontsize,
        handletextpad=0.3,  # Reduce spacing between marker and text
        borderaxespad=0.3,  # Reduce padding between legend and plot
        columnspacing=0.9  # Reduce spacing between columns
    )

    # Adjust layout and save/display
    fig.tight_layout()
    if outdir is not None:
        plt.savefig(f"{outdir}/{f'{tod}-{title}' if tod is not None else f'{title}'}.pdf", dpi=300, bbox_inches="tight")
    else:
        plt.show()
    plt.close()

INDIR = '/home/hrisim/ros_ws/src/HRISim/hrisim_postprocess/csv/sensitivity'
BAGNAMES = [
    # KD = 0.1
    'KD01_KPD1_KBC05', 'KD01_KPD1_KBC5', 'KD01_KPD1_KBC50',
    'KD01_KPD10_KBC05', 'KD01_KPD10_KBC5', 'KD01_KPD10_KBC50',
    'KD01_KPD100_KBC05', 'KD01_KPD100_KBC5', 'KD01_KPD100_KBC50',
    # KD = 1
    'KD1_KPD1_KBC05', 'KD1_KPD1_KBC5', 'KD1_KPD1_KBC50',
    'KD1_KPD10_KBC05', 'KD1_KPD10_KBC5', 'KD1_KPD10_KBC50',
    'KD1_KPD100_KBC05', 'KD1_KPD100_KBC5', 'KD1_KPD100_KBC50',
    # KD = 10
    'KD10_KPD1_KBC05', 'KD10_KPD1_KBC5', 'KD10_KPD1_KBC50',
    'KD10_KPD10_KBC05', 'KD10_KPD10_KBC5', 'KD10_KPD10_KBC50',
    'KD10_KPD100_KBC05', 'KD10_KPD100_KBC5', 'KD10_KPD100_KBC50',
]
CATEGORIES = {
    # KD = 0.1
    'KD01_KPD1_KBC05': '($\lambda_D:0.1$, $\lambda_{PD}:1$, $\lambda_{BC}:0.5$)', 'KD01_KPD1_KBC05': '($\lambda_D:0.1$, $\lambda_{PD}:1$, $\lambda_{BC}:0.5$)', 'KD01_KPD1_KBC5': '($\lambda_D:0.1$, $\lambda_{PD}:1$, $\lambda_{BC}:5$)', 'KD01_KPD1_KBC50': '($\lambda_D:0.1$, $\lambda_{PD}:1$, $\lambda_{BC}:50$)',
    'KD01_KPD10_KBC05': '($\lambda_D:0.1$, $\lambda_{PD}:10$, $\lambda_{BC}:0.5$)', 'KD01_KPD10_KBC5': '($\lambda_D:0.1$, $\lambda_{PD}:10$, $\lambda_{BC}:5$)', 'KD01_KPD10_KBC50': '($\lambda_D:0.1$, $\lambda_{PD}:10$, $\lambda_{BC}:50$)',
    'KD01_KPD100_KBC05': '($\lambda_D:0.1$, $\lambda_{PD}:100$, $\lambda_{BC}:0.5$)', 'KD01_KPD100_KBC5': '($\lambda_D:0.1$, $\lambda_{PD}:100$, $\lambda_{BC}:5$)', 'KD01_KPD100_KBC50': '($\lambda_D:0.1$, $\lambda_{PD}:100$, $\lambda_{BC}:50$)',
    # KD = 1
    'KD1_KPD1_KBC05': '($\lambda_D:1$, $\lambda_{PD}:1$, $\lambda_{BC}:0.5$)', 'KD1_KPD1_KBC5': '($\lambda_D:1$, $\lambda_{PD}:1$, $\lambda_{BC}:5$)', 'KD1_KPD1_KBC50': '($\lambda_D:1$, $\lambda_{PD}:1$, $\lambda_{BC}:50$)',
    'KD1_KPD10_KBC05': '($\lambda_D:1$, $\lambda_{PD}:10$, $\lambda_{BC}:0.5$)', 'KD1_KPD10_KBC5': '($\lambda_D:1$, $\lambda_{PD}:10$, $\lambda_{BC}:5$)', 'KD1_KPD10_KBC50': '($\lambda_D:1$, $\lambda_{PD}:10$, $\lambda_{BC}:50$)',
    'KD1_KPD100_KBC05': '($\lambda_D:1$, $\lambda_{PD}:100$, $\lambda_{BC}:0.5$)', 'KD1_KPD100_KBC5': '($\lambda_D:1$, $\lambda_{PD}:100$, $\lambda_{BC}:5$)', 'KD1_KPD100_KBC50': '($\lambda_D:1$, $\lambda_{PD}:100$, $\lambda_{BC}:50$)',
    # KD = 10
    'KD10_KPD1_KBC05': '($\lambda_D:10$, $\lambda_{PD}:1$, $\lambda_{BC}:0.5$)', 'KD10_KPD1_KBC5': '($\lambda_D:10$, $\lambda_{PD}:1$, $\lambda_{BC}:5$)', 'KD10_KPD1_KBC50': '($\lambda_D:10$, $\lambda_{PD}:1$, $\lambda_{BC}:50$)',
    'KD10_KPD10_KBC05': '($\lambda_D:10$, $\lambda_{PD}:10$, $\lambda_{BC}:0.5$)', 'KD10_KPD10_KBC5': '($\lambda_D:10$, $\lambda_{PD}:10$, $\lambda_{BC}:5$)', 'KD10_KPD10_KBC50': '($\lambda_D:10$, $\lambda_{PD}:10$, $\lambda_{BC}:50$)',
    'KD10_KPD100_KBC05': '($\lambda_D:10$, $\lambda_{PD}:100$, $\lambda_{BC}:0.5$)', 'KD10_KPD100_KBC5': '($\lambda_D:10$, $\lambda_{PD}:100$, $\lambda_{BC}:5$)', 'KD10_KPD100_KBC50': '($\lambda_D:10$, $\lambda_{PD}:100$, $\lambda_{BC}:50$)',
}
OUTDIR = os.path.join('/home/hrisim/ros_ws/src/HRISim/hrisim_postprocess/results/sensitivity/')
os.makedirs(OUTDIR, exist_ok=True)

# Initialize aggregated data structures
success_failure_metrics = {}
working_time_metrics = {}
travelled_distance_metrics = {}
causal_battery_metrics = {}
battery_metrics = {}
velocity_metrics = {}
battery_charging_metrics = {}
charging_time_metrics = {}
collision_metrics = {}
clearance_metrics = {}
proxemics_metrics = {}

pval_working_time = {bagname: {"Active Time": [], "Stalled Time": [], "Wasted Time": []} for bagname in BAGNAMES}
pval_travelled_distance = {bagname: {"Planned Travelled Distance": [], "Extra Travelled Distance": [], "Wasted Travelled Distance": []} for bagname in BAGNAMES}
pval_battery = {bagname: {"Effective Battery Usage": [], "Wasted Battery Usage": []} for bagname in BAGNAMES}
pval_proxemics = {bagname: {"Distances": []} for bagname in BAGNAMES}
   
# Load metrics for each bag
for bagname in BAGNAMES:
    metrics_path = os.path.join(INDIR, bagname, "metrics.pkl")
    with open(metrics_path, 'rb') as pkl_file:
        METRICS = pickle.load(pkl_file)
        
    for tod in [TOD.H2, TOD.H6]:
        metrics_tod = METRICS[tod.value]
        for task in metrics_tod.keys():
            if isinstance(task, int):
                # Ensure data is flattened and numeric before extending
                flat_distances = itertools.chain(*metrics_tod[task]['agent_distances'].values())
                pval_proxemics[bagname]["Distances"].extend(
                    value for value in flat_distances 
                    if isinstance(value, (int, float)) and value < 7.6
                )

    
    # EFFICIENCY 
    #! Success & Failures
    success_failure_metrics[bagname] = {
        "Success": {"value": METRICS['overall_success'],
                       "%": METRICS['overall_success']*100/METRICS['task_count'],
                       "100%": METRICS['task_count'],
                       "color": "tab:blue",},
        "Failures~(D)": {"value": METRICS['overall_failure_people'],
                                "%": METRICS['overall_failure_people']*100/METRICS['task_count'], 
                                "100%": METRICS['task_count'],
                                "color": "tab:orange",},
        "Failures~(L)": {"value": METRICS['overall_failure_critical_battery'], 
                                          "%": METRICS['overall_failure_critical_battery']*100/METRICS['task_count'], 
                                          "100%": METRICS['task_count'],
                                          "color": "tab:red",},
    }
    
    #! Task Time
    total_time = METRICS['overall_task_time']/3600
    active_time = METRICS['overall_time_to_reach_goal_actual']/3600
    stalled_time = METRICS['overall_time_to_reach_goal_stalled']/3600
    wasted_time = METRICS['overall_time_to_reach_goal_wasted']/3600
    
    # Handle potential division by zero if total_time is 0
    total_time_safe = total_time if total_time > 0 else 1 
    
    working_time_metrics[bagname] = {
        "Active": {"value": active_time, 
                        "%": active_time*100/total_time_safe,
                        "100%": total_time,
                        "color": "tab:blue",},
        "Stalled": {"value": stalled_time, 
                         "%": stalled_time*100/total_time_safe, 
                        "100%": total_time,
                         "color": "tab:orange",},
        "Wasted": {"value": wasted_time, 
                        "%": wasted_time*100/total_time_safe, 
                        "100%": total_time,
                        "color": "tab:red",},
        "Total": {"value": total_time_safe}
    }
    
    #! Travelled Distance
    total_distance = (METRICS['overall_travelled_distance_actual'] + METRICS['overall_travelled_distance_wasted'])/1000
    planned_distance = METRICS['overall_travelled_distance_planned_only_success']/1000
    extra_distance = (METRICS['overall_travelled_distance_actual'] - METRICS['overall_travelled_distance_planned_only_success'])/1000
    wasted_distance = METRICS['overall_travelled_distance_wasted']/1000
    
    # Handle potential division by zero if total_distance is 0
    total_distance_safe = total_distance if total_distance > 0 else 1
    
    travelled_distance_metrics[bagname] = {
        "Planned": {"value": planned_distance, 
                                       "%": planned_distance*100/total_distance_safe,
                                       "100%": total_distance,
                                       "color": "tab:blue",},
        "Extra": {"value": extra_distance,
                                     "%": extra_distance*100/total_distance_safe, 
                                     "100%": total_distance,
                                     "color": "tab:orange",},
        "Wasted": {"value": wasted_distance,
                                      "%": wasted_distance*100/total_distance_safe,  
                                      "100%": total_distance,
                                      "color": "tab:red",},
        "Total": {"value": total_distance_safe}
    }
    
    
    #! Battery
    # Get planned and actual battery consumption
    actual_battery = METRICS['overall_battery_consumption_actual']/100
    wasted_battery = METRICS['overall_battery_consumption_wasted']/100
    
    # Use max(planned, actual) as the reference to normalize percentages
    reference_total = actual_battery + wasted_battery
    
    # Handle potential division by zero if reference_total is 0
    reference_total_safe = reference_total if reference_total > 0 else 1

    # Update battery metrics dictionary
    battery_metrics[bagname] = {
        "Effective": {
            "value": actual_battery,
            "%": actual_battery * 100 / reference_total_safe,
            "100%": reference_total,
            "color": "tab:blue",},
        "Wasted": {
            "value": wasted_battery,
            "%": wasted_battery * 100 / reference_total_safe,
            "100%": reference_total,
            "color": "tab:red",},
        "Total": {"value": reference_total_safe}
    }
    
    # SAFETY 
    collision_metrics[bagname] = METRICS['overall_human_collision']


print("Aggregating data for analysis table...")

# 1. Calculate proxemics statistics
proxemics_stats = {}
for bagname in BAGNAMES:
    proxemics_stats[bagname] = calculate_boxplot_stats(pval_proxemics[bagname]["Distances"])
    proxemics_stats[bagname]["label"] = CATEGORIES[bagname]

# 2. Create a master dictionary
analysis_data_table = {
    'labels': CATEGORIES,
    'success_failure': success_failure_metrics,
    'task_time': working_time_metrics,
    'travelled_distance': travelled_distance_metrics,
    'battery_usage': battery_metrics,
    'collisions': collision_metrics,
    'proxemics_statistics': proxemics_stats
}

# 3. Save the master dictionary to a JSON file
json_output_path = os.path.join(OUTDIR, 'sensitivity_analysis_data.json')
try:
    with open(json_output_path, 'w') as f:
        json.dump(analysis_data_table, f, indent=4)
    print(f"Successfully saved analysis data to: {json_output_path}")
except TypeError as e:
    print(f"Error saving JSON (possibly due to non-serializable data like np.nan): {e}")
    # Fallback for np.nan issues
    def convert(o):
        if isinstance(o, np.generic): return o.item()
        raise TypeError
    with open(json_output_path, 'w') as f:
        json.dump(analysis_data_table, f, indent=4, default=convert)
    print(f"Successfully saved analysis data to: {json_output_path} (with custom converter)")
except Exception as e:
    print(f"An unexpected error occurred while saving JSON: {e}")


# # --- Start of Plotting ---
# print("Generating plots...")

# plot_stacked_bar(success_failure_metrics, "Success-Failure", "Count", CATEGORIES, outdir=OUTDIR, step=1)
# plot_stacked_bar(working_time_metrics, "Task Time", "h", CATEGORIES, outdir=OUTDIR, step=0.1)
# plot_stacked_bar(travelled_distance_metrics, "Travelled Distance", "km", CATEGORIES, outdir=OUTDIR, step=1)
# plot_stacked_bar(battery_metrics, "Battery Usage", "Battery Cycles", CATEGORIES, outdir=OUTDIR, step=.02)

# plt.figure(figsize=(10, 6))
# x_pos = np.arange(len(BAGNAMES)) 
# colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"] * (len(BAGNAMES) // 4 + 1)
# colors = colors[:len(BAGNAMES)]
# fontsize = 20 

# # Create bars and store them in a variable
# bars = plt.bar(x_pos, collision_metrics.values(), width=0.4, color=colors) # Reduced width a bit

# # Set labels and title
# plt.ylabel("Count", fontsize=fontsize)
# plt.xticks(x_pos, CATEGORIES.values(), fontsize=fontsize - 8, rotation=90)
# plt.yticks(fontsize=fontsize)
# plt.grid(axis='y', linestyle='--', alpha=0.6)
# plt.title(f"Number of Human-Robot Collisions", fontsize=fontsize)
# max_val = max(collision_metrics.values()) if collision_metrics.values() else 1
# plt.ylim(0, max_val * 1.1)  # Give some space above bars
# plt.yticks(np.arange(0, max_val + 1, 2))

# plt.tight_layout()

# # Annotate bars
# for bar, value in zip(bars, collision_metrics.values()):
#     plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height() / 2,
#              f"{int(value)}", ha='center', va='center', fontsize=10)
# plt.savefig(f"{OUTDIR}/Dangerous_Interaction.pdf", dpi=300, bbox_inches="tight")
# plt.close()


# # --- Call to plot_boxplot ---
# print("Generating Proxemics Box Plot...")
# plot_boxplot(pval_proxemics, "Distances", "m", CATEGORIES, background=True, outdir=OUTDIR)
# print("All plots generated.")