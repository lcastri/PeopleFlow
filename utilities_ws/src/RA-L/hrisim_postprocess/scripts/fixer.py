import os
import pandas as pd
from utils import *

INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/original'
BAGNAME = 'BL50_13102024'

for tod in TOD:
    print(f"analysing {tod.value}")
    DF = pd.read_csv(os.path.join(INDIR, f"{BAGNAME}", tod.value, f"{BAGNAME}_{tod.value}.csv"))
    
    # Drop rows that contain any NaN values
    # Reset the index so that it is consecutive after dropping rows
    DF.dropna(inplace=True)
    DF.reset_index(drop=True, inplace=True)
    
    for i in range(1, len(DF)):
        if (DF.loc[i-1, "G_X"], DF.loc[i-1, "G_Y"]) == (DF.loc[i, "G_X"], DF.loc[i, "G_Y"]) and DF.loc[i-1, "R_T"] != DF.loc[i, "R_T"] and DF.loc[i, "R_T"] == 1:
            print(f"{BAGNAME}_{tod.value}.csv ERROR row {i}")
            DF.loc[i, "R_T"] = 0
    DF.to_csv(os.path.join(INDIR, f"{BAGNAME}", tod.value, f"{BAGNAME}_{tod.value}.csv"), index=False)
   
for tod in TOD:
    DF = pd.read_csv(os.path.join(INDIR, f"{BAGNAME}", tod.value, f"{BAGNAME}_{tod.value}.csv"))
    for wp in WP:
        if wp == WP.PARKING or wp == WP.CHARGING_STATION: continue
        print(f"analysing {tod.value}-{wp.value}")
        WPDF = pd.read_csv(os.path.join(INDIR, f"{BAGNAME}", tod.value, f"{BAGNAME}_{tod.value}_{wp.value}.csv"))
        
        # Drop rows that contain any NaN values
        # Reset the index so that it is consecutive after dropping rows
        WPDF.dropna(inplace=True)
        WPDF.reset_index(drop=True, inplace=True)
    
        WPDF["R_T"] = DF["R_T"]
        WPDF.to_csv(os.path.join(INDIR, f"{BAGNAME}", tod.value, f"{BAGNAME}_{tod.value}_{wp.value}.csv"), index=False)
        
# for tod in TOD:
#     DF = pd.read_csv(os.path.join(INDIR, f"{BAGNAME}", tod.value, f"{BAGNAME}_{tod.value}.csv"))
#     if DF.iloc[0]["R_T"] != -1: continue
    
#     print(f"{BAGNAME}_{tod.value}.csv contains error")
    
#     DF.loc[0, "R_T"] = DF.iloc[1]["R_T"]
#     DF.to_csv(os.path.join(INDIR, f"{BAGNAME}_shrink", tod.value, f"{BAGNAME}_{tod.value}.csv"), index=False)
#     print(f"{BAGNAME}_{tod.value}.csv fixed!")
    
# for tod in TOD:
#     for wp in WP:
#         if wp == WP.PARKING or wp == WP.CHARGING_STATION: continue
#         WPDF = pd.read_csv(os.path.join(INDIR, f"{BAGNAME}", tod.value, f"{BAGNAME}_{tod.value}_{wp.value}.csv"))
#         if WPDF.iloc[0]["R_T"] != -1: continue
        
#         print(f"{BAGNAME}_{tod.value}_{wp.value}.csv contains error")
        
#         WPDF.loc[0, "R_T"] = WPDF.iloc[1]["R_T"]
#         WPDF.to_csv(os.path.join(INDIR, f"{BAGNAME}_shrink", tod.value, f"{BAGNAME}_{tod.value}_{wp.value}.csv"), index=False)
#         print(f"{BAGNAME}_{tod.value}_{wp.value}.csv fixed!")