import os
import pandas as pd
from utils import *


INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/TOD/my_nonoise'
OUTDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/TOD/HH'
# BAGNAME= ['BL100_21102024', 'BL75_29102024', 'BL50_22102024', 'BL25_28102024']
BAGNAME= ['BL75_29102024']
dfs = []
for bag in BAGNAME:
    for wp in WP:
        if wp == WP.PARKING or wp == WP.CHARGING_STATION: continue
        for tod in TOD:
            if wp is WP.DOOR_ENTRANCE_CANTEEN:
                tmp_wp = "door_entrance-canteen"
            else:
                tmp_wp = wp.value.replace("-", "_")
            WPDF = pd.read_csv(os.path.join(INDIR, f"{bag}", tod.value, f"{bag}_{tod.value}_{tmp_wp}.csv"))
            WPDF.reset_index(drop=True, inplace=True)
            dfs.append(WPDF)
            
        WPDF = pd.concat(dfs, axis=0)
        WPDF.reset_index(drop=True, inplace=True)
        dfs = []
        time_slot = 1
        new_df = pd.DataFrame(columns=WPDF.columns)
        for t in range(WPDF.shape[0]):
            if WPDF["pf_elapsed_time"][t] > time_slot*3600:
                
                new_df["TOD"] = time_slot - 1
                output_dir = os.path.join(OUTDIR, f"{bag}")
                os.makedirs(output_dir, exist_ok=True)
                new_df.to_csv(os.path.join(output_dir, f"{bag.replace('_', '-')}_{time_slot-1}h_{wp.value}.csv"), index=False)
                
                time_slot += 1
                new_df = pd.DataFrame(columns=WPDF.columns)
            new_df = new_df.append(WPDF.loc[t])
        
        new_df["TOD"] = time_slot - 1
        output_dir = os.path.join(OUTDIR, f"{bag}")
        os.makedirs(output_dir, exist_ok=True)
        new_df.to_csv(os.path.join(output_dir, f"{bag.replace('_', '-')}_{time_slot-1}h_{wp.value}.csv"), index=False)