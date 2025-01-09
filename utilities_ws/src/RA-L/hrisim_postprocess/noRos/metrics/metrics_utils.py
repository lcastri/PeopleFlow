import math
import os
from matplotlib import pyplot as plt
import numpy as np
import xml.etree.ElementTree as ET


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
    Calculate the number of unique human collisions.
    A collision is counted only when a human enters the robot's base diameter and then exits.

    Parameters:
        df (pd.DataFrame): DataFrame containing robot and human positions over time.

    Returns:
        int: Total number of unique collisions with humans.
    """
    robot_coords = df[['R_X', 'R_Y']].to_numpy()
    collision_count = 0
    
    # Track collision state for each human
    human_collision_states = {}

    # Identify human coordinate columns dynamically
    for col in df.columns:
        if col.startswith('a') and col.endswith('_X'):
            human_id = col[:-2]  # Extract human identifier (e.g., 'a1', 'a2')
            human_collision_states[human_id] = False  # Initialize collision state

    # Iterate over time steps
    for _, row in df.iterrows():
        r_x, r_y = row['R_X'], row['R_Y']

        for col in df.columns:
            if col.startswith('a') and col.endswith('_X'):
                human_id = col[:-2]  # Extract human identifier
                h_x, h_y = row[col], row[col.replace('_X', '_Y')]

                # Calculate distance
                distance = np.sqrt((r_x - h_x)**2 + (r_y - h_y)**2)

                # Check for collision entry
                if distance < (robot_diameter/2) and not human_collision_states[human_id]:
                    collision_count += 1
                    human_collision_states[human_id] = True  # Human enters collision zone

                # Check for collision exit
                elif distance >= (robot_diameter/2):
                    human_collision_states[human_id] = False  # Human exits collision zone

    return collision_count


def compute_hall_count(df, proxemics_threshold):
    """
    Compute the Personal Space Compliance (PSC) metric for multiple humans across proxemic zones.

    Parameters:
        df (pd.DataFrame): DataFrame containing robot and human positions over time.
        thresholds (list): Thresholds defining proxemic boundaries in meters.
            [intimate, personal, social, public]

    Returns:
        dict: PSC scores for each proxemic zone.
    """
    # Identify human columns dynamically
    human_columns = [(col, col.replace('_X', '_Y')) for col in df.columns if col.startswith('a') and col.endswith('_X')]

    # Initialize a dictionary for PSC scores by zone
    hall_count = {zone: 0 for zone in proxemics_threshold.keys()}

    robot_coords = df[['R_X', 'R_Y']].to_numpy()
    for ai_x_col, ai_y_col in human_columns:
        human_coords = df[[ai_x_col, ai_y_col]].to_numpy()

        # Calculate the Euclidean distance between robot and human
        distances = np.sqrt(np.sum((robot_coords - human_coords)**2, axis=1))

        # Determine which zone the interaction falls into and add penalties
        for zone, (lower, upper) in proxemics_threshold.items():
            if zone == 'no-interaction':
                hall_count[zone] += np.sum(distances > lower)
            else:
                hall_count[zone] += np.sum((lower < distances) & (distances <= upper))

    return hall_count


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
        
        
def plot_stacked_bar(metrics_dict, title, ylabel, xTickLabel, bar_width=0.2, offset = 0.01, yticks = None, figsize=(14, 9), tod = None, outdir=None):
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
    plt.xticks(x, [xTickLabel[cat] for cat in categories], fontsize=12)
    if yticks is not None: plt.yticks(yticks, fontsize=12)  # y ticks every 10
    plt.ylabel(ylabel, fontsize=14)
    if tod is not None:
        plt.title(f"{tod} -- {title}", fontsize=16)
    else:
        plt.title(title, fontsize=16)
        
    # Move legend below the chart
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05), fancybox=True, shadow=False, ncol=len(components))

    # Add grid for clarity
    plt.grid(axis='y', linestyle='--', alpha=0.5)

    # Display or save the plot
    if outdir is not None:
        if tod is not None:
            plt.savefig(os.path.join(outdir, f"{tod}-{title}.png"), dpi=300, bbox_inches='tight')
        else:
            plt.savefig(os.path.join(outdir, f"{title}.png"), dpi=300, bbox_inches='tight')
    else:
        plt.show()
        
        
def plot_efficiency(metrics_list, titles, ylabel, xTickLabel, figsize=(20, 12), tod = None, outdir=None):
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
        ax.set_xticklabels([xTickLabel[cat] for cat in categories], fontsize=14)
        ax.set_title(title, fontsize=14)

        # Add grid for clarity
        ax.grid(axis='y', linestyle='--', alpha=0.5)
        ax.legend(fontsize=14, loc='upper center', bbox_to_anchor=(0.5, -0.05), ncol=1)
        ax.set_ylim(0, 105)
        
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
        
        
def plot_grouped_stacked_bars(trends, time_labels, metric_name, ylabel, bar_width=0.1, group_offset=0.125, label_bar_pos = 2, percentage=True, outdir=None):
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
