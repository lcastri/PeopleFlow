import math
import os
from matplotlib import pyplot as plt
import numpy as np
import xml.etree.ElementTree as ET
from scipy.stats import shapiro, mannwhitneyu, ttest_ind, normaltest


def readScenario(scenario):
    # Load and parse the XML file
    
    tree = ET.parse('/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/scenarios/' + scenario + '.xml')
    root = tree.getroot()
    
    tmp = {}
    for waypoint in root.findall('waypoint'):
        waypoint_id = waypoint.get('id')
        x = float(waypoint.get('x'))
        y = float(waypoint.get('y'))
        r = float(waypoint.get('r'))
        tmp[waypoint_id] = {'x': x, 'y': y, 'r': r}
    return tmp
        
        
def get_initrow(df):
    for r in range(len(df)):
        if (df.iloc[r]["G_X"] != -1000 and df.iloc[r]["G_Y"] != -1000 and df.iloc[r].notnull().all()):
            return r
        
        
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

def compute_p_values(data, metric_name, alpha=0.05):
    noncausal_data = np.array(data['base'][metric_name])
    causal_data = np.array(data['causal'][metric_name])

    # Check normality for both groups
    noncausal_normal = False
    causal_normal = False
    # noncausal_normal = is_normal(noncausal_data, alpha)
    # causal_normal = is_normal(causal_data, alpha)

    # Select appropriate test
    if noncausal_normal and causal_normal:
        stat, p_value = ttest_ind(noncausal_data, causal_data, equal_var=False)
        test_used = "T-Test"
    else:
        stat, p_value = mannwhitneyu(noncausal_data, causal_data, alternative='two-sided')
        test_used = "Mann-Whitney U Test"

    return p_value, test_used
        
        
def compute_min_h_distance(df):
    robot_coords = df[['R_X', 'R_Y']].to_numpy()
    min_distance_to_humans = []
    for col in df.columns:
        if col.startswith('a') and col.endswith('_X'):
            human_coords = df[[col, col.replace('_X', '_Y')]].to_numpy()
            min_distance_to_humans.append(np.min(np.sqrt(np.sum((robot_coords - human_coords)**2, axis=1))))
    return np.min(min_distance_to_humans)


def compute_stalled_time(df, stalled_threshold):
    # Filter the data to only consider rows where the robot velocity is below the stall threshold
    stalled_df = df[df['R_V'] <= stalled_threshold]

    stalled_time = 0

    # Iterate through the index of the filtered dataframe (stalled_df)
    for t in range(1, len(stalled_df)):
        # Compare the current and previous indices to check if they are consecutive
        if stalled_df.index[t] == stalled_df.index[t-1] + 1:
            # Add the time difference between consecutive timestamps
            stalled_time += stalled_df.loc[stalled_df.index[t], 'ros_time'] - stalled_df.loc[stalled_df.index[t-1], 'ros_time']

    return stalled_time


def compute_travelled_distance(task_df):
    """Compute the actual distance travelled by the robot"""
    coords = task_df[['R_X', 'R_Y']].to_numpy()
    diffs = np.diff(coords, axis=0)
    distances = np.sqrt((diffs ** 2).sum(axis=1))
    return np.sum(distances)


def compute_planned_battery_consumption(dist, n_wp, robot_max_vel, ks, kd, astar_time):
    time_to_goal = math.ceil(dist/robot_max_vel)
    return time_to_goal * (ks + kd * robot_max_vel) + astar_time * n_wp * ks


def compute_actual_battery_consumption(task_df):
    battery = task_df['R_B'].to_numpy()
    return battery[0] - battery[-1]


def compute_human_collision(df, robot_diameter):
    """
    Calculate the number of unique human collisions using vectorized operations with NumPy.
    A collision is counted when a human enters the robot's base diameter and then exits.

    Parameters:
        df (pd.DataFrame): DataFrame containing robot and human positions over time.
        robot_diameter (float): Diameter of the robot base.

    Returns:
        int: Total number of unique collisions with humans.
    """
    # Convert the robot's coordinates to NumPy arrays
    r_x = df['R_X'].values
    r_y = df['R_Y'].values

    # Extract human columns dynamically
    human_x_cols = [col for col in df.columns if col.startswith('a') and col.endswith('_X')]
    human_y_cols = [col.replace('_X', '_Y') for col in human_x_cols]
    
    collision_count = 0
    radius = robot_diameter / 2
    
    # Iterate over human coordinate columns in pairs (x, y)
    for h_x_col, h_y_col in zip(human_x_cols, human_y_cols):
        h_x = df[h_x_col].values
        h_y = df[h_y_col].values

        # Compute the distance matrix using vectorized NumPy operations
        distances = np.sqrt((r_x - h_x)**2 + (r_y - h_y)**2)

        # Determine collision state (True for collision, False otherwise)
        collisions = distances < radius

        # Detect transitions into and out of collision state
        transitions = np.diff(collisions.astype(int))
        collision_entries = np.sum(transitions == 1)  # Count entries into the collision zone
        collision_count += collision_entries

    return collision_count


def compute_hall_count(df, proxemics_threshold):
    """
    Compute the Personal Space Compliance (PSC) metric for multiple humans across proxemic zones
    and save distances for each agent for further analysis.

    Parameters:
        df (pd.DataFrame): DataFrame containing robot and human positions over time.
        thresholds (list): Thresholds defining proxemic boundaries in meters.
            [intimate, personal, social, public]

    Returns:
        tuple: A dictionary of PSC scores for each proxemic zone and a dictionary of distances for each agent.
    """
    # Identify human columns dynamically
    human_columns = [(col, col.replace('_X', '_Y')) for col in df.columns if col.startswith('a') and col.endswith('_X')]

    # Initialize a dictionary for PSC scores by zone
    hall_count = {zone: 0 for zone in proxemics_threshold.keys()}

    # Initialize a dictionary to store distances for each agent
    agent_distances = {col: [] for col, _ in human_columns}

    robot_coords = df[['R_X', 'R_Y']].to_numpy()
    for ai_x_col, ai_y_col in human_columns:
        human_coords = df[[ai_x_col, ai_y_col]].to_numpy()

        # Calculate the Euclidean distance between robot and human
        distances = np.sqrt(np.sum((robot_coords - human_coords)**2, axis=1))
        agent_distances[ai_x_col] = distances.tolist()  # Save distances for each agent

        # Determine which zone the interaction falls into and add penalties
        for zone, (lower, upper) in proxemics_threshold.items():
            if zone == 'no-interaction':
                hall_count[zone] += np.sum(distances > lower)
            else:
                hall_count[zone] += np.sum((lower < distances) & (distances <= upper))

    return hall_count, agent_distances


# Function to convert numpy types to Python native types
def make_serializable(obj):
    if isinstance(obj, (np.int64, np.int32, np.integer)):
        return int(obj)
    elif isinstance(obj, (np.float64, np.float32, np.floating)):
        return float(obj)
    elif isinstance(obj, dict):
        return {key: make_serializable(value) for key, value in obj.items()}
    elif isinstance(obj, list):
        return [make_serializable(value) for value in obj]
    else:
        return obj


def plot_grouped_bar(bagnames, metrics_dict, title, ylabel, figsize=(14, 8), outdir=None):
    metric_categories = list(metrics_dict[bagnames[0]].keys())
    x = np.arange(len(metric_categories))
    width = 0.35

    plt.figure(figsize=figsize)
    for i, bagname in enumerate(bagnames):
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
        
        
# def plot_stacked_bar(metrics_dict, title, ylabel, xTickLabel, bar_width=0.1, offset = 0.01, yticks = None, figsize=(8, 6), tod = None, outdir=None):
#     # Categories and components
#     categories = list(metrics_dict.keys())
#     components = list(metrics_dict[categories[0]].keys())

#     # Extracting values
#     values = {component: [metrics_dict[cat][component]["value"] for cat in categories] for component in components}
#     percs = {component: [metrics_dict[cat][component]["%"] for cat in categories] for component in components}
#     colors = {component: [metrics_dict[cat][component]["color"] for cat in categories] for component in components}

#     # Create the bar chart
#     x = [(bar_width + offset) * i for i in range(len(categories))]  # the label locations

#     # Initialize bottom for stacking
#     bottom = np.zeros(len(categories))

#     # Plot each component
#     fig, ax = plt.subplots(figsize=figsize)
#     for component, value in values.items():
#         bars = plt.bar(x, value, bar_width, label=component, bottom=bottom, color=colors[component])
#         bottom += np.array(value)
#         # Annotate each bar segment with its percentage value
#         for bar in bars:
#             height = bar.get_height()
#             if height > 0:
#                 ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() / 2 + bar.get_y(),
#                         f'{percs[component][bars.index(bar)]:.2f}%', ha='center', va='center', fontsize=10, color='black')
                
#     # Adjust x-axis and other labels
#     plt.xticks(x, [xTickLabel[cat] for cat in categories], fontsize=12)
#     if yticks is not None: plt.yticks(yticks, fontsize=12)
#     plt.ylabel(ylabel, fontsize=14)
#     plt.title(f"{tod} -- {title}" if tod is not None else title, fontsize=16)
#     plt.ylim(0, max(bottom) * 1.1)
    
#     # Move legend below the chart
#     plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), fancybox=True, shadow=False, ncol=len(components))

#     # Add grid for clarity
#     plt.grid(axis='y', linestyle='--', alpha=0.5)

#     # Display or save the plot
#     if outdir is not None:
#         plt.savefig(os.path.join(outdir, f"{tod}-{title}.png" if tod is not None else f"{title}.png"), dpi=300, bbox_inches='tight')
#     else:
#         plt.show()

# Convert p-values into significance markers
def get_significance(p):
    if p < 0.0001:
        return "****"
    elif p < 0.001:
        return "***"
    elif p < 0.01:
        return "**"
    elif p < 0.05:
        return "*"
    return "n.s."  # Not significant

def plot_stacked_bar(metrics_dict, title, ylabel, xTickLabel, bar_width=0.1, offset=0.01, figsize=(10, 8), fontsize=20, showSubclass = False, step = 10, tod = None, outdir=None, noPerc=False):
    # Categories and components
    categories = list(metrics_dict.keys())
    components = list(metrics_dict[categories[0]].keys())

    # Extracting values and colors for stacked bars
    values = {component: [metrics_dict[cat][component]["value"] for cat in categories] for component in components}
    pvalues = {component: [metrics_dict[cat][component]["p-value"] for cat in categories] for component in components}
    if not noPerc:
        percs = {component: [metrics_dict[cat][component]["%"] for cat in categories] for component in components}
    colors = {component: [metrics_dict[cat][component]["color"] for cat in categories] for component in components}
    # total_values = [metrics_dict[cat][components[0]]["100%"] for cat in categories]  # Overall total for each category

    # Additional components
    if showSubclass:
        separated_values = {categories[i]: [values[component][i] for component in components] for i in range(len(categories))}
        separated_colors = {categories[i]: [colors[component][i] for component in components] for i in range(len(categories))}

    # Define the bar positions
    x_separated = []
    x_ticks = []
    x_stacked = []
    if showSubclass:
        for tick in range(len(categories)):
            tmp_x_stacked = tick/2
            x_stacked.append(tmp_x_stacked)
            pbar = tmp_x_stacked
            for sp in separated_values[categories[tick]]:
                x_separated.append(pbar + bar_width + offset)
                pbar = pbar + bar_width + offset
            x_ticks.append(tmp_x_stacked + (pbar - tmp_x_stacked)/2)
    else:
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
                    if showSubclass:
                        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_y() + height / 2,
                                f"{percs[component][i]:.1f}%", ha='center', va='center', fontsize=fontsize)
                    else:
                        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_y() + height / 2,
                                f"{percs[component][i]:.1f}%", ha='center', va='center', fontsize=fontsize)
                        # ax.text(bar.get_x() + bar.get_width() / 2, bar.get_y() + height / 2,
                        #         f"{values[component][i]:.2f} ({percs[component][i]:.1f}%)", ha='center', va='center', fontsize=fontsize)
                else:
                    ax.text(bar.get_x() + bar.get_width() / 2, bar.get_y() + height / 2,
                            f"{values[component][i]:.2f}", ha='center', va='center', fontsize=fontsize)
    # Add absolute annotation for stacked bars
    # for i, bar in enumerate(bars):
    #     ax.text(bar.get_x() + bar.get_width() / 2, total_values[i] + 1,
    #             f"{total_values[i]:.1f}", ha='center', va='bottom', fontsize=fontsize)
                

    if showSubclass:
        # Plot total bars
        tmp_separated_values = [sp for c in categories for sp in separated_values[c]]
        tmp_separated_separated_colors = [sp for c in categories for sp in separated_colors[c]]
        total_bars = ax.bar(x_separated, tmp_separated_values, bar_width, color=tmp_separated_separated_colors)
        for i, bar in enumerate(total_bars):
            if tmp_separated_values[i] != 0:
                ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 1,
                        f"{tmp_separated_values[i]:.1f}", ha='center', va='bottom', fontsize=fontsize)

    # Configure x-axis and labels
    ax.set_xticks(x_ticks)
    ax.set_xticklabels([xTickLabel[cat] for cat in categories], fontsize=fontsize)
    ax.tick_params(axis='y', labelsize=fontsize)

    ax.set_ylabel(ylabel, fontsize=fontsize)
    ax.set_ylim(0, max(bottom) * 1.1)
    ax.set_yticks(np.arange(0, max(bottom) + step, step))
    ax.set_title(f"{tod} -- {title}" if tod is not None else title, fontsize=fontsize)
    # ax.set_yticks(np.round(np.linspace(0, max(bottom), num=6), 2))  # Adjust `num=6` as needed

    # Add grid and legend
    ax.grid(axis='y', linestyle='--', alpha=0.6)
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
          
        
def plot_efficiency(metrics_list, titles, ylabel, xTickLabel, figsize=(20, 12), tod = None, outdir=None):
    n_plots = len(metrics_list)
    fig, axs = plt.subplots(1, n_plots, figsize=figsize, sharey=True)

    for i, (ax, metrics, title) in enumerate(zip(axs, metrics_list, titles)):
        # Categories and components
        categories = list(metrics.keys())
        components = list(metrics[categories[0]].keys())

        # Extract values and colors
        values = {component: [metrics[cat][component]["value"] for cat in categories] for component in components}
        percs = {component: [metrics[cat][component]["%"] for cat in categories] for component in components}
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
                            f'{percs[component][bars.index(bar)]:.2f}%', ha='center', va='center', fontsize=fontsize, color='black')

        # Set x-axis labels
        ax.set_xticks(x)
        ax.set_xticklabels([xTickLabel[cat] for cat in categories], fontsize=14)
        ax.set_title(title, fontsize=14)

        # Add grid for clarity
        ax.grid(axis='y', linestyle='--', alpha=0.5)
        ax.legend(fontsize=14, loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=len(components))
        
    # Set common y-axis and title
    fig.supylabel(ylabel, fontsize=16)
    if tod is not None: 
        fig.suptitle(f"{tod} -- Efficiency Metrics", fontsize=16)
    else:
        fig.suptitle("Overall -- Efficiency Metrics", fontsize=16)

    # Adjust layout
    fig.tight_layout(rect=[0, 0.05, 1, 0.95])

    # Save or display the plot
    if outdir is not None:
        if tod is not None: 
            output_path = os.path.join(outdir, f"{tod}-Efficiency.png")
        else:
            output_path = os.path.join(outdir, f"Efficiency.png")
        plt.savefig(output_path, dpi=300, bbox_inches="tight")
    else:
        plt.show()
        
        
def plot_trend(metric_trends, time_labels, title, ylabel, outdir = None):
    plt.figure(figsize=(12, 6))
    for category, metrics in metric_trends.items():
        for metric_name, values in metrics.items():
            plt.plot(time_labels, values, label=f"{category} - {metric_name}")
    
    plt.title(title)
    plt.xlabel("Time Splits")
    plt.ylabel(ylabel)
    plt.xticks()
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    
    # Save or display the plot
    if outdir is not None:
        output_path = os.path.join(outdir, f"{title.replace(' ', '_')}_trend.png")
        plt.savefig(output_path, dpi=300, bbox_inches="tight")
    else:
        plt.show()
        
        
def plot_stacked_bars_over_time(trends, time_labels, metric_name, ylabel, bar_width=0.1, group_offset=0.125, label_bar_pos = 2, outdir=None):
    categories = list(trends.keys())  # E.g., ["Non-Causal", "Causal"]
    components = list(trends[categories[0]].keys())  # Metric components, e.g., ["N. Success", "N. Failure"]

    num_time_splits = len(time_labels)
    num_categories = len(categories)
    bar_width = bar_width / num_categories  # Width of bars inside one group
    # x_positions = np.arange(num_time_splits)  # Base x-axis positions for time splits
    x_positions = [group_offset * i for i in range(num_time_splits)]
    
    plt.figure(figsize=(12, 6))
    bar_offsets = np.linspace(-bar_width / 2, bar_width / 2, num_categories)  # Smaller space between bars in the same group

    # Loop through each category (e.g., Non-Causal/Causal)
    for i, category in enumerate(categories):
        bottom = np.zeros(len(time_labels))  # For stacking bars
        for component in components:
            values = np.array(trends[category][component]["value"])  # Trend values across time splits
            percs = np.array(trends[category][component]["%"])  # Trend values across time splits
            color = trends[category][component]["color"]  # Color for this component
            bars = plt.bar(
                x_positions + bar_offsets[i],  # Adjust for grouped categories and the offset between bars
                values,
                width=bar_width,
                bottom=bottom,
                label=f"{component}" if i == 0 else "",  # Only add label for the first bar in the group
                color=color,
                edgecolor='black',  # Tight black border around each bar
                linewidth=0.5  # Thin border
            )
            
                
            bottom += values  # Update stacking height
        # Adding text annotations at the bottom of each bar
        for j, rect in enumerate(bars):
            # Position text at the bottom of each bar (for the stacking, add half the bar's height)
            plt.text(
                rect.get_x() + rect.get_width() / 2,  # X position (center of the bar)
                label_bar_pos,  # Y position (fixed position at the bottom of the plot)
                f'{category}',  # Text value, formatting to 2 decimal places
                ha='center',  # Center alignment horizontally
                va='bottom',  # Center alignment vertically
                fontsize=9,  # Adjust font size as needed
                color='black',  # Color of the text
                rotation=90
            )

    # Set labels, title, and legend
    plt.xticks(x_positions, time_labels, fontsize=10)
    plt.xlabel("Time Splits", fontsize=12)
    plt.ylabel(ylabel, fontsize=12)
    plt.title(f"{metric_name} Trend", fontsize=14)
    # Move the legend to the bottom
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=len(components), fontsize=10)
    plt.tight_layout()
    plt.grid(axis='y', linestyle='--', alpha=0.5)
    plt.ylim(0, max(bottom) * 1.05)

    # Save or display the plot
    if outdir is not None:
        output_path = os.path.join(outdir, f"{metric_name.replace(' ', '_')}_trend.png")
        plt.savefig(output_path, dpi=300, bbox_inches="tight")
    else:
        plt.show()
        
        
def plot_bars_over_time(trends, time_labels, metric_name, ylabel, bar_width=0.1, group_offset=0.125, label_bar_pos = 2, percentage=True, outdir=None):
    categories = list(trends.keys())  # E.g., ["Non-Causal", "Causal"]
    components = list(trends[categories[0]].keys())  # Metric components, e.g., ["N. Success", "N. Failure"]

    num_time_splits = len(time_labels)
    num_categories = len(categories)
    bar_width = bar_width / num_categories  # Width of bars inside one group
    # x_positions = np.arange(num_time_splits)  # Base x-axis positions for time splits
    x_positions = [group_offset * i for i in range(num_time_splits)]
    
    plt.figure(figsize=(12, 6))
    bar_offsets = np.linspace(-bar_width / 2, bar_width / 2, num_categories)  # Smaller space between bars in the same group

    # Loop through each category (e.g., Non-Causal/Causal)
    for i, category in enumerate(categories):
        bottom = np.zeros(len(time_labels))  # For stacking bars
        for component in components:
            values = np.array(trends[category][component]["value"])  # Trend values across time splits
            color = trends[category][component]["color"]  # Color for this component
            bars = plt.bar(
                x_positions + bar_offsets[i],  # Adjust for grouped categories and the offset between bars
                values,
                width=bar_width,
                bottom=bottom,
                label=f"{component}" if i == 0 else "",  # Only add label for the first bar in the group
                color=color,
                edgecolor='black',  # Tight black border around each bar
                linewidth=0.5  # Thin border
            )
            
                
            bottom += values  # Update stacking height
        # Adding text annotations at the bottom of each bar
        for j, rect in enumerate(bars):
            # Position text at the bottom of each bar (for the stacking, add half the bar's height)
            plt.text(
                rect.get_x() + rect.get_width() / 2,  # X position (center of the bar)
                label_bar_pos,  # Y position (fixed position at the bottom of the plot)
                f'{category}',  # Text value, formatting to 2 decimal places
                ha='center',  # Center alignment horizontally
                va='bottom',  # Center alignment vertically
                fontsize=9,  # Adjust font size as needed
                color='black',  # Color of the text
                rotation=90
            )

    # Set labels, title, and legend
    plt.xticks(x_positions, time_labels, fontsize=10)
    plt.xlabel("Time Splits", fontsize=12)
    plt.ylabel(ylabel, fontsize=12)
    plt.title(f"{metric_name} Trend", fontsize=14)
    # Move the legend to the bottom
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=len(components), fontsize=10)
    plt.tight_layout()
    plt.grid(axis='y', linestyle='--', alpha=0.5)
    if percentage: plt.ylim(0, 101)

    # Save or display the plot
    if outdir is not None:
        output_path = os.path.join(outdir, f"{metric_name.replace(' ', '_')}_trend.png")
        plt.savefig(output_path, dpi=300, bbox_inches="tight")
    else:
        plt.show()
