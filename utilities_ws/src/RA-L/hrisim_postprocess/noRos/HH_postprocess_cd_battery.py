import math
import os
import pickle
import numpy as np
import pandas as pd
from metrics.utils import *
import networkx as nx
import xml.etree.ElementTree as ET

def get_initrow(df):
    for r in range(len(df)):
        if (df.iloc[r]["G_X"] != -1000 and df.iloc[r]["G_Y"] != -1000 and df.iloc[r].notnull().all()):
            return r  

# Load map information
INCSV_PATH= os.path.expanduser('utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/shrunk')
OUTCSV_PATH= os.path.expanduser(f'utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/my_noise/')
BAGNAME= ['test-obs-21022025']

static_duration = 5
dynamic_duration = 4
charging_time = 2
OBS_FACTOR = 5
ROBOT_MAX_VEL = 0.5
K_nl_s = 100 / (static_duration * 3600)
K_nl_d = (100 / (dynamic_duration * 3600) - K_nl_s)/(ROBOT_MAX_VEL)
K_l_s = K_nl_s * OBS_FACTOR
K_l_d = K_nl_d * OBS_FACTOR
charge_rate = 100 / (charging_time * 3600)

with open('/home/lcastri/git/PeopleFlow/HRISim_docker/HRISim/peopleflow/peopleflow_manager/res/warehouse/graph.pkl', 'rb') as f:
    G = pickle.load(f)

# Load data
for bag in BAGNAME:
    print(f"\n### Analysing rosbag: {bag}")
    for tod in TOD:
        print(f"- time: {tod.value}")
        tmp_bag = bag.replace('_', '-')
        DF = pd.read_csv(os.path.join(INCSV_PATH, f"{bag}", tod.value, "static", f"{tmp_bag}_{tod.value}.csv"))
        n_rows = len(DF)
        
        # RV
        RV = DF['R_V'] + np.random.normal(0, 0.01, size=n_rows)
        DF['R_V'] = RV
        
        # EC
        EC = np.full_like(DF['R_V'], 0)
        dt = np.diff(DF['pf_elapsed_time']).tolist()
        dt.insert(0, 5.0)
        dt = np.array(dt)
        EC = np.where(DF["OBS"] == 0, 
                      dt * (K_nl_s + K_nl_d * DF['R_V']), 
                      dt * (K_l_s + K_l_d * DF['R_V']))
        DF['EC'] = EC + np.random.uniform(0.025, 0.025, size=n_rows)
        
        # Create output directory if it doesn't exist
        out_path = os.path.join(OUTCSV_PATH, f'{bag}', f'{tod.value}')
        os.makedirs(out_path, exist_ok=True)
        
        # Save the updated DF
        DF.to_csv(os.path.join(out_path, f"{tmp_bag}_{tod.value}.csv"), index=False)