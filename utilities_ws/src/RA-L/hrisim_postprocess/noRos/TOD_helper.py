from utils import *
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv'
BAGNAME= ['BL100_07112024']
USE_SUBSAMPLED = True

# def detrend(signal, window_size):
#     detrended_signal = np.copy(signal)
#     # Loop through the signal and subtract the window mean
#     for i in range(len(signal) - window_size + 1):
#         window = signal[i:i+window_size]
#         window_median = np.median(window)
#         detrended_signal[i:i+window_size] -= window_median
        
#     return detrended_signal
def detrend(signal, window_size):
    detrended_signal = np.copy(signal)
        
    return np.sin(detrended_signal)

for bagname in BAGNAME:
    for wp in WP:
        dfs = []
        if wp == WP.PARKING or wp == WP.CHARGING_STATION: continue
        print(f"### Loading : {bagname}-{wp.value}")
        for tod in TOD:
            print(f"- {tod.value}")
            if USE_SUBSAMPLED:
                filename = os.path.join(INDIR, "my_nonoise", f"{bagname}", tod.value, f"{bagname}_{tod.value}_{wp.value}.csv")
            else:
                filename = os.path.join(INDIR, "original", f"{bagname}", tod.value, f"{bagname}_{tod.value}_{wp.value}.csv")
            WPDF = pd.read_csv(filename)
            dfs.append(WPDF)
        concatenated_df = pd.concat(dfs, ignore_index=True)

        # Example: assuming "TOD" has a daily cycle, normalize to radians
        # Replace `max_tod` with the maximum value in the "TOD" variable (e.g., 24 for hours in a day).
        # concatenated_df['TOD'] = np.sin((concatenated_df['TOD'].values / np.max(concatenated_df['TOD'].values)) * 2 * np.pi)

        concatenated_df['TOD'] = detrend(concatenated_df['TOD'], 200)
        concatenated_df[['R_V', 'TOD']].plot()
        plt.show()
        break
    break


