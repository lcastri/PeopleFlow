from enum import Enum
import math
import os
from fpcmci.preprocessing.data import Data
from fpcmci.preprocessing.subsampling_methods.Static import Static
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

class TOD(Enum):
    STARTING = "starting"
    MORNING = "morning"
    LUNCH = "lunch"
    AFTERNOON = "afternoon"
    QUITTING = "quitting"
    OFF = "off"


class WP(Enum):
  PARKING = "parking"
  DOOR_ENTRANCE = "door_entrance"
  DOOR_ENTRANCE_CANTEEN = "door_entrance-canteen"
  CORRIDOR_ENTRANCE = "corridor_entrance"
  DOOR_CORRIDOR1 = "door_corridor1"
  DOOR_CORRIDOR2 = "door_corridor2"
  DOOR_CORRIDOR3 = "door_corridor3"
  SHELF12 = "shelf12"
  SHELF23 = "shelf23"
  SHELF34 = "shelf34"
  SHELF45 = "shelf45"
  SHELF56 = "shelf56"
  DOOR_OFFICE1 = "door_office1"
  DOOR_OFFICE2 = "door_office2"
  DOOR_TOILET1 = "door_toilet1"
  DOOR_TOILET2 = "door_toilet2"
  DELIVERY_POINT = "delivery_point"
  CORRIDOR0 = "corridor0"
  CORRIDOR1 = "corridor1"
  CORRIDOR2 = "corridor2"
  CORRIDOR3 = "corridor3"
  CORRIDOR4 = "corridor4"
  CORRIDOR5 = "corridor5"
  ENTRANCE = "entrance"
  OFFICE1 = "office1"
  OFFICE2 = "office2"
  TOILET1 = "toilet1"
  TOILET2 = "toilet2"
  TABLE2 = "table2"
  TABLE3 = "table3"
  TABLE4 = "table4"
  TABLE5 = "table5"
  TABLE6 = "table6"
  CORR_CANTEEN_1 = "corr_canteen_1"
  CORR_CANTEEN_2 = "corr_canteen_2"
  CORR_CANTEEN_3 = "corr_canteen_3"
  CORR_CANTEEN_4 = "corr_canteen_4"
  CORR_CANTEEN_5 = "corr_canteen_5"
  CORR_CANTEEN_6 = "corr_canteen_6"
  KITCHEN_1 = "kitchen1"
  KITCHEN_2 = "kitchen2"
  KITCHEN_3 = "kitchen3"
  CORRIDOR_CANTEEN = "corridor_canteen"
  SHELF1 = "shelf1"
  SHELF2 = "shelf2"
  SHELF3 = "shelf3"
  SHELF4 = "shelf4"
  SHELF5 = "shelf5"
  SHELF6 = "shelf6"
  CHARGING_STATION = "charging_station"
  
  
def compute_bandwidth_non_zero(df, sampling_time, small_threshold=5000):
    N, dim = df.shape
    bandwidths = []
    for i in range(dim):
        fft_signal = np.fft.rfft(df.iloc[:, i])
        fft_magnitude = np.abs(fft_signal)
    
        # Find the frequency axis
        freq_axis = np.fft.rfftfreq(N, sampling_time)
    
        # # Plotting the FFT magnitude
        # plt.figure(figsize=(12, 6))
        # plt.plot(freq_axis, fft_magnitude, label='FFT Magnitude Spectrum')
        # plt.axhline(y=small_threshold, color='red', linestyle='--', label='Small Magnitude Threshold')
        # plt.title('FFT Magnitude Spectrum')
        # plt.xlabel('Frequency (Hz)')
        # plt.ylabel('Magnitude')
        # plt.legend()
        # plt.grid()
        # plt.show()

        # Find frequencies where the magnitude is greater than the small threshold
        indices_above_threshold = np.where(fft_magnitude > small_threshold)[0]
        
        # Bandwidth is the range between the first and last indices above threshold
        if len(indices_above_threshold) > 0:
            bandwidths.append(freq_axis[indices_above_threshold[-1]] - freq_axis[indices_above_threshold[0]])
        else:
            bandwidths.append(0)
    
    return max(bandwidths)


FS = 10 #Hz
INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/original'
BAGNAME = 'BL100'

DF = pd.read_csv(os.path.join(INDIR, BAGNAME + "_shrink", TOD.STARTING.value, BAGNAME + "_" + TOD.STARTING.value))
for r in range(len(DF)):
    if (DF.iloc[r]["G_X"] != -1000 and DF.iloc[r]["G_Y"] != -1000 
        and DF.iloc[r].notnull().all()):
        break
DF = DF.loc[r:, ["R_X", "R_Y", "R_V", "G_X", "G_Y", "a1_X", "a1_Y", "a1_YAW", "a1_V"]]
df = Data(DF)
f = compute_bandwidth_non_zero(df.d, 1/FS)
print(f"Bandwidth fm: {f:.4f} Hz")
print(f"Subsampling frequency fs >= 2fm = {2*f:.4f} Hz")

# for tod in TOD:
#     DF = pd.read_csv(os.path.join(INDIR, BAGNAME + "_shrink", tod.value, BAGNAME + "_" + tod.value))
#     for r in range(len(DF)):
#         if (DF.iloc[r]["G_X"] != -1000 and DF.iloc[r]["G_Y"] != -1000 
#             and DF.iloc[r].notnull().all()):
#             break
#     DF = DF.loc[r:, ["R_X", "R_Y", "R_V", "G_X", "G_Y", "a1_X", "a1_Y", "a1_YAW", "a1_V"]]
#     df = Data(DF)
#     f = compute_bandwidth_non_zero(df.d, 1/FS)
#     print(f"Bandwidth fm: {f:.4f} Hz")
#     print(f"Subsampling frequency fs >= 2fm = {2*f:.4f} Hz")
#     for wp in WP:
#         WPDF = pd.read_csv(os.path.join(INDIR, BAGNAME + "_shrink", tod.value, BAGNAME + "_" + tod.value))




df.plot_timeseries()