import matplotlib.pyplot as plt
import numpy as np

# Data from the table
scenarios = ['A', 'B']

# ADE values: [A, B] as mean of rosbag1 and rosbag2
ade_sgan = [(12.4 + 16.32) / 2, (10.88 + 24.27) / 2]
ade_neurosym = [(7.06 + 2.52) / 2, (5.7 + 9.87) / 2]

# FDE values: [A, B] as mean of rosbag1 and rosbag2
fde_sgan = [(2.28 + 3.24) / 2, (2.67 + 5) / 2]
fde_neurosym = [(1.31 + 0.68) / 2, (1.4 + 1.8) / 2]

# Inference Time: [A, B]
time_sgan = [4.17, 5.19]
time_neurosym = [5.37, 7.36]

x = np.array([0, 0.3])  # label locations
width = 0.1  # width of the bars

fig, ax = plt.subplots(figsize=(5, 5))
bars1 = ax.bar(x - 0.5*width, ade_sgan, width, color='#ea9999')
bars2 = ax.bar(x + 0.5*width, ade_neurosym, width, color='#9ac07d')
# Labels and title
ax.set_xlabel('Scenario')
ax.set_ylabel('Error (m)')
ax.set_title('Average Displacement Error (ADE)')
ax.set_xticks(x)
ax.set_xticklabels(scenarios)
# Add labels at the base of each bar
# for bar in bars1:
#     ax.text(bar.get_x() + bar.get_width()/2, 0.2, 'SGAN', ha='center', va='bottom', fontsize=11, rotation=90)
# for bar in bars2:
#     ax.text(bar.get_x() + bar.get_width()/2, 0.2, 'NeuroSyM', ha='center', va='bottom', fontsize=11, rotation=90)
plt.tight_layout()
plt.show()


fig, ax = plt.subplots(figsize=(5, 5))
bars1 = ax.bar(x - 0.5*width, fde_sgan, width, color='#ea9999')
bars2 = ax.bar(x + 0.5*width, fde_neurosym, width, color='#9ac07d')
# Labels and title
ax.set_xlabel('Scenario')
ax.set_ylabel('Error (m)')
ax.set_title('Final Displacement Error (FDE)')
ax.set_xticks(x)
ax.set_xticklabels(scenarios)
# Add labels at the base of each bar
# for bar in bars1:
#     ax.text(bar.get_x() + bar.get_width()/2, 0.1, 'SGAN', ha='center', va='bottom', fontsize=11, rotation=90)
# for bar in bars2:
#     ax.text(bar.get_x() + bar.get_width()/2, 0.1, 'NeuroSyM', ha='center', va='bottom', fontsize=11, rotation=90)
plt.tight_layout()
plt.show()


fig, ax = plt.subplots(figsize=(5, 5))
bars1 = ax.bar(x - 0.5*width, time_sgan, width, color='#ea9999')
bars2 = ax.bar(x + 0.5*width, time_neurosym, width, color='#9ac07d')
# Labels and title
ax.set_xlabel('Scenario')
ax.set_ylabel('time (s)')
ax.set_title('Prediction Time')
ax.set_xticks(x)
ax.set_xticklabels(scenarios)
# Add labels at the base of each bar
# for bar in bars1:
#     ax.text(bar.get_x() + bar.get_width()/2, 0.2, 'SGAN', ha='center', va='bottom', fontsize=11, rotation=90)
# for bar in bars2:
#     ax.text(bar.get_x() + bar.get_width()/2, 0.2, 'NeuroSyM', ha='center', va='bottom', fontsize=11, rotation=90)
plt.tight_layout()
plt.show()
