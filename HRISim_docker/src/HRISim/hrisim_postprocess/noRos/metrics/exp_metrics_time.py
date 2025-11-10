import itertools
import json
import pickle
import os
from metrics_utils import *
from utils import *

INDIR = '/home/hrisim/ros_ws/src/HRISim/hrisim_postprocess/csv/experiments'
BAGNAMES = ['base', 'versionA', 'versionB', 'causal']
CATEGORIES = {'base': 'Baseline', 'versionA': 'Ablation A', 'versionB': 'Ablation B', 'causal': 'Causal'}
    
# Load metrics for each bag
for bagname in BAGNAMES:
    metrics_path = os.path.join(INDIR, bagname, "metrics.pkl")
    with open(metrics_path, 'rb') as pkl_file:
        METRICS = pickle.load(pkl_file)
    
    MEAN_EVALUATIONS = METRICS['mean_evaluations']
    MEAN_PLANNING_TIME = METRICS['mean_planning_time']
    MEAN_QUERY_TIME = METRICS['mean_query_inf_time']
    MEAN_TOT_QUERY_TIME = METRICS['tot_query_inf_time']/2000
    
    print("--------------------------------------------------")
    print(f"Metrics for {CATEGORIES[bagname]}:")
    print(f"Mean Evaluations: {MEAN_EVALUATIONS}")
    print(f"Mean Planning Time: {MEAN_PLANNING_TIME}")
    print(f"Mean Query Time: {MEAN_QUERY_TIME}")
    print(f"Mean Total Query Time: {MEAN_TOT_QUERY_TIME}")
    print("--------------------------------------------------")