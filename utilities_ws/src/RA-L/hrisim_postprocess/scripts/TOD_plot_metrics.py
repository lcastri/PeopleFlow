import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import json
import os

INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/TOD/original'
BAGNAME= 'noncausal_13122024'

with open(os.path.join(INDIR, f"{BAGNAME}", "metrics.json"), 'r') as json_file:
    METRICS = json.load(json_file)

# Extract metrics for plotting
aggregated_metrics = {
    "Overall Success": METRICS['overall_success'],
    "Overall Failure": METRICS['overall_failure'],
    "Human Collisions": METRICS['overall_human_collision']
}

time_metrics = {
    "Mean Stalled Time (s)": METRICS['mean_stalled_time'],
    "Mean Time to Goal (s)": METRICS['mean_time_to_reach_goal'],
    "Mean Path Length (m)": METRICS['mean_path_length']
}

velocity_metrics = {
    "Mean Min Velocity (m/s)": METRICS['mean_min_velocity'],
    "Mean Max Velocity (m/s)": METRICS['mean_max_velocity'],
    "Mean Avg Velocity (m/s)": METRICS['mean_average_velocity']
}

distance_metrics = {
    "Mean Min Clearing Distance (m)": METRICS['mean_min_clearing_distance'],
    "Mean Max Clearing Distance (m)": METRICS['mean_max_clearing_distance'],
    "Mean Avg Clearing Distance (m)": METRICS['mean_average_clearing_distance'],
    "Mean Min Distance to Humans (m)": METRICS['mean_min_distance_to_humans'],
}

space_compliance_metrics = {
    "Mean Intimate Space Compliance": METRICS['mean_space_compliance']['intimate'],
    "Mean Personal Space Compliance": METRICS['mean_space_compliance']['personal'],
    "Mean Social Space Compliance": METRICS['mean_space_compliance']['social'],
    "Mean Public Space Compliance": METRICS['mean_space_compliance']['public'],
}

# Bar chart for aggregated metrics
plt.figure(figsize=(8, 5))
plt.bar(aggregated_metrics.keys(), aggregated_metrics.values(), color=['green', 'red', 'orange'])
plt.title('Overall Aggregated Metrics')
plt.ylabel('Count')
plt.grid()

# Time-related metrics
plt.figure(figsize=(8, 5))
sns.barplot(x=list(time_metrics.keys()), y=list(time_metrics.values()), palette='Blues')
plt.title('Time-Related Metrics')
plt.ylabel('Seconds / Meters')
plt.grid()

# Velocity metrics
plt.figure(figsize=(8, 5))
sns.barplot(x=list(velocity_metrics.keys()), y=list(velocity_metrics.values()), palette='Purples')
plt.title('Velocity Metrics')
plt.ylabel('m/s')
plt.grid()

# Clearing and distance metrics
plt.figure(figsize=(10, 6))
sns.barplot(x=list(distance_metrics.keys()), y=list(distance_metrics.values()), palette='Greens')
plt.title('Clearing and Distance Metrics')
plt.ylabel('Meters')
plt.grid()

# Clearing and distance metrics
plt.figure(figsize=(10, 6))
sns.barplot(x=list(space_compliance_metrics.keys()), y=list(space_compliance_metrics.values()), palette='Greens')
plt.title('Clearing and Distance Metrics')
plt.ylabel('Meters')
plt.grid()
plt.show()
