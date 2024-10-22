import time
from matplotlib.collections import LineCollection
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as mpimg
from utils import TOD
import yaml
import os
from PIL import Image

def seconds_to_hhmmss(seconds):
    return time.strftime("%H:%M:%S", time.gmtime(seconds))

# Load map information
ANIM_DIR = os.path.expanduser('~/git/PeopleFlow/utilities_ws/src/RA-L/trajectory_plot/')
MAP_DIR = os.path.expanduser('~/git/PeopleFlow/utilities_ws/src/RA-L/peopledensity/maps/')
MAP_NAME = 'warehouse'
TRAJ_PATH= os.path.expanduser('utilities_ws/src/RA-L/hrisim_postprocess/csv/original/')
BAGNAME= 'BL100_21102024'
ROBOT_COLOR = 'tab:orange'
STEP = 600
MARKER_SIZE = 40

with open(MAP_DIR + MAP_NAME + '/map.yaml', 'r') as yaml_file:
    map_info = yaml.safe_load(yaml_file)

# Step 1: Load the PNG Image
map_image = mpimg.imread(MAP_DIR + MAP_NAME + '/map.pgm')

# Step 2: Plot the Map
# Get resolution and origin from YAML
resolution = map_info['resolution']
origin_x, origin_y = map_info['origin'][:2]

# Plot the map image
fig, ax = plt.subplots(figsize=(4.8, 4.8))
ax.imshow(map_image, extent=(origin_x, origin_x + len(map_image[0]) * resolution, 
                             origin_y, origin_y + len(map_image) * resolution),
           cmap='gray')

ax.set_xlim(-20, 11)  # Replace with your desired x-axis limits
ax.set_ylim(-11, 23)  # Replace with your desired y-axis limits

# Load trajectory data
dfs = []
for tod in TOD:
    df = pd.read_csv(os.path.join(TRAJ_PATH, f"{BAGNAME}", tod.value, f"{BAGNAME}_{tod.value}.csv"), index_col=0)
    selected_columns = ["pf_elapsed_time", "R_X", "R_Y"]
    for i in range(1, 20):
        selected_columns += [f'a{i}_X', f'a{i}_Y']
    df = df.loc[:, selected_columns]
    dfs.append(df.reset_index(drop=True))
trajectory_data = pd.concat(dfs, axis=0)

# Initialize the line objects for robot and human trajectories
robot_line, = ax.plot([], [], linestyle='-', color=ROBOT_COLOR)
robothead_point = ax.scatter([], [], s=MARKER_SIZE, color=ROBOT_COLOR, marker='D', zorder=3)
human_line = {}
humanhead_point = {}
for i in range(1, 20):
    human_line[i], = ax.plot([], [], linestyle='-')
    humanhead_point[i] = ax.scatter([], [], s=MARKER_SIZE, zorder=3)

time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, fontsize=12,
                    verticalalignment='top', horizontalalignment='left')

# Set the threshold for disappearing points (in number of frames)
SECS_TODISPLAY = 1 # [s]
DT = 0.1 # [s]
threshold = int(SECS_TODISPLAY / DT)

# Function to update the plot for animation
def update_plot(frame):
    # Calculate the time difference from the start time
    # current_time = pd.to_datetime(trajectory_data['pf_elapsed_time'].iloc[frame], unit='s')
    dt = seconds_to_hhmmss(8*3600 + trajectory_data['pf_elapsed_time'].iloc[frame])

    # Update the time label text
    time_text.set_text(f'Time: {dt}')

    start_index = max(0, frame - threshold)  # Starting index for the trajectory
    
    # Robot line
    robot_x = trajectory_data['R_X'].iloc[start_index:frame].values
    robot_y = trajectory_data['R_Y'].iloc[start_index:frame].values
    robot_line.set_data(robot_x, robot_y)
    
    # Human line
    human_x = {}
    human_y = {}
    for i in range(1, 20):
        human_x[i] = trajectory_data[f'a{i}_X'].iloc[start_index:frame].values
        human_y[i] = trajectory_data[f'a{i}_Y'].iloc[start_index:frame].values
        human_line[i].set_data(human_x[i], human_y[i])
    
    if frame != 0:
        # Robot head
        robot_line_segments = np.array([robot_x, robot_y]).T.reshape(-1, 1, 2)
        robot_line_collection = LineCollection(robot_line_segments, color=ROBOT_COLOR, linewidth=2)
        ax.add_collection(robot_line_collection)
        dot_x, dot_y = robot_x[-1], robot_y[-1]
        robothead_point.set_offsets([[dot_x, dot_y]])
        
        # Human head
        for i in range(1, 20):
            human_line_segments = np.array([human_x[i], human_y[i]]).T.reshape(-1, 1, 2)
            human_line_collection = LineCollection(human_line_segments, linewidth=2)
            ax.add_collection(human_line_collection)
            dot_x, dot_y = human_x[i][-1], human_y[i][-1]
            humanhead_point[i].set_offsets([[dot_x, dot_y]])

    return [robot_line, *human_line.values(), robothead_point, *humanhead_point.values(), time_text]

# Set labels and legend
plt.xlabel('X')
plt.ylabel('Y')
# plt.legend()

# Set axis limits
plt.xlim(-20, 11)
plt.ylim(-11, 23)

# Create the animation
ani = animation.FuncAnimation(fig, update_plot, frames=len(trajectory_data), interval=10, blit=True, repeat=False)

# Create a directory to save frames
if not os.path.exists(ANIM_DIR + f'frames/{BAGNAME}'):
    os.makedirs(ANIM_DIR + f'frames/{BAGNAME}')

# Save frames as images
for i in range(0, len(trajectory_data), STEP):
    update_plot(i)
    plt.savefig(ANIM_DIR + f'frames/{BAGNAME}/frame_{i:04d}.png')

# Convert frames to GIF using Pillow
images = []
for i in range(0, len(trajectory_data), STEP):
    images.append(Image.open(ANIM_DIR + f'frames/{BAGNAME}/frame_{i:04d}.png'))
images[0].save(ANIM_DIR + f'gif/{BAGNAME}.gif', save_all=True, append_images=images[1:], duration=100, loop=0)