import math
import os
from fpcmci.preprocessing.data import Data
from fpcmci.preprocessing.subsampling_methods.Static import Static
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import entropy
from scipy.fft import fft, fftfreq
from scipy.signal import butter, filtfilt
from metrics.utils import *
import copy
  
def fixer(indir, bags):

    for bag in bags:
        for tod in TOD:
            print(f"analysing {tod.value}")
            DF = pd.read_csv(os.path.join(indir, f"{bag}", tod.value, f"{bag}_{tod.value}.csv"))
            
            if "is_charging" in DF.columns: DF = DF.rename(columns={f"is_charging": "B_S"})

            # Drop rows that contain any NaN values
            # Reset the index so that it is consecutive after dropping rows
            DF.dropna(inplace=True)
            DF.reset_index(drop=True, inplace=True)
            
            for i in range(1, len(DF)):
                if (DF.loc[i-1, "G_X"], DF.loc[i-1, "G_Y"]) == (DF.loc[i, "G_X"], DF.loc[i, "G_Y"]) and DF.loc[i-1, "T_R"] != DF.loc[i, "T_R"] and DF.loc[i, "T_R"] == 1:
                    print(f"{bag}_{tod.value}.csv ERROR row {i}")
                    DF.loc[i, "T_R"] = 0
            DF.to_csv(os.path.join(indir, f"{bag}", tod.value, f"{bag}_{tod.value}.csv"), index=False)
            
            
            # Save WP-specific DataFrames
            general_columns_name = ['pf_elapsed_time', 'TOD', 'T', 'R_V', 'T_R', 'R_B', 'B_S', 'R_X', 'R_Y', 'G_X', 'G_Y']
            for wp in WP:
                if wp == WP.PARKING or wp == WP.CHARGING_STATION: continue
                WPDF = copy.deepcopy(DF[general_columns_name + [f"{wp.value}_NP", f"{wp.value}_PD", f"{wp.value}_BAC"]])
                
                # Rename the WP-specific columns
                WPDF = WPDF.rename(columns={f"{wp.value}_NP": "NP", f"{wp.value}_PD": "PD", f"{wp.value}_BAC": "BAC"})
                
                # Add the constant column "wp" with the value wp_id
                WPDF["WP"] = WPS[wp.value]
                
                wp_filename = f"{bag}_{tod.value}_{wp.value}.csv"
                WPDF.to_csv(f"{indir}/{bag}/{tod.value}/{wp_filename}", index=False)
                print(f"Saved {wp_filename}")
  
  
def plot_bandwidth(signal, varname, sampling_rate, bandwidth = None):
    fft_signal = np.fft.rfft(signal)
    fft_magnitude = np.abs(fft_signal)
    
    # Find the frequency axis
    freq_axis = np.fft.rfftfreq(len(signal), 1 / sampling_rate)
    
    # Plotting the FFT magnitude
    plt.figure(figsize=(12, 6))
    plt.plot(freq_axis, fft_magnitude, label='FFT Magnitude Spectrum')
    plt.axvline(x=0, color='red', linestyle='--')
    plt.axvline(x=bandwidth, color='red', linestyle='--')
    plt.title(f'{varname} - FFT Magnitude Spectrum')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude')
    plt.legend()
    plt.grid()
    # plt.show()


def low_pass_filter(data, cutoff_freq, sampling_rate, order=4):
    # Design a Butterworth low-pass filter
    nyquist = 0.5 * sampling_rate
    normal_cutoff = cutoff_freq / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    
    # Apply the filter
    filtered_data = filtfilt(b, a, data)
    
    return filtered_data


def get_bandwidth(df, sampling_rate, cutoff, energy_percentage, plot):
    N, dim = df.shape
    bandwidths = []
    
    for i in range(dim):
        varname = df.columns[i]
        
        # 1. Remove noise
        # filtered_signal = low_pass_filter(df.values[:,i], cutoff, sampling_rate)
        filtered_signal = df.values[:,i]
        # 2. Compute the FFT
        fft_values = fft(filtered_signal)
        frequencies = fftfreq(N, 1 / sampling_rate)

        # 3. Compute the power spectrum (magnitude squared)
        power_spectrum = np.abs(fft_values[:N // 2])**2
        positive_frequencies = frequencies[:N // 2]

        # 4. Calculate total energy (sum of power spectrum)
        total_energy = np.sum(power_spectrum)

        # 5. Set target cumulative energy (e.g., 95% of total energy)
        target_energy = energy_percentage * total_energy

        # 6. Cumulatively sum the power spectrum
        cumulative_energy = np.cumsum(power_spectrum)

        # 7. Find the indices where the cumulative energy meets the target percentage
        lower_index = np.where(cumulative_energy >= (1 - energy_percentage) * total_energy)[0][0]
        upper_index = np.where(cumulative_energy >= target_energy)[0][0]

        # 8. Calculate bandwidth between first and last significant frequencies
        bandwidth = positive_frequencies[upper_index] - positive_frequencies[lower_index]

        print(f"{varname} bandwidth ({energy_percentage*100}% energy): {bandwidth} Hz")
        
        bandwidths.append(bandwidth)
        
        if plot: plot_bandwidth(filtered_signal, varname, sampling_rate, bandwidth)
    
    return max(bandwidths)


def get_initrow(df):
    for r in range(len(df)):
        if (df.iloc[r]["G_X"] != -1000 and df.iloc[r]["G_Y"] != -1000 and df.iloc[r].notnull().all()):
            return r
        
        
def get_subsampling_step(cutoff = 1, energy_percentage=0.95, plot = True):
    wp = WP.DELIVERY_POINT.value
    rs = {}
    dfs = []
    for bag in BAGNAME:
        for tod in TOD:
            df = pd.read_csv(os.path.join(INDIR, f"{bag}", tod.value, f"{bag}_{tod.value}.csv"), index_col=0)
            r = get_initrow(df)
            tmp_wp = wp.replace("-", "_")
            df = df.loc[r:, ["R_X", "R_Y", "R_V", "R_B", 'B_S', f"{tmp_wp}_NP", f"{tmp_wp}_PD", f"{tmp_wp}_BAC"]]
            rs[tod] = r
            dfs.append(df.reset_index(drop=True))
        
    df = pd.concat(dfs, axis=0)

    _bw = get_bandwidth(df, SF, cutoff = cutoff, energy_percentage = energy_percentage, plot=plot)
    _ssf = 2*_bw
    _st = 1/_ssf
    _step = int(math.floor(_st * SF))
    
    del df
    return rs, _bw, _ssf, _st, _step


SF = 10 #Hz
INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/TOD/original'
OUTDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/TOD/shrunk'
# BAGNAME= ['BL100_07112024']
BAGNAME= ['BL100_21102024', 'BL75_29102024', 'BL50_22102024', 'BL25_28102024', 'BL100_07112024', 'BL20_06112024']
# BAGNAME= ['BL75_29102024']
# BAGNAME= ['BL100_21102024', 'BL75_29102024', 'BL50_22102024', 'BL25_28102024']


# fixer(INDIR, ['BL75_29102024'])

R, BW, SSF, ST, STEP = get_subsampling_step(cutoff = 1, energy_percentage=0.95, plot = True)
print(f"Bandwidth fm: {BW:.4f} Hz")
print(f"Subsampling frequency fs >= 2fm = {2*BW:.4f} Hz")
print(f"Subsampling time 1 sample every each {ST:.4f} s")
print(f"Subsampling step {STEP}")
print("")
        
for bag in ['BL75_29102024']:
    print(f"Subsampling {bag}")
    for tod in [TOD.MORNING]:
        print(f"- {tod.value}.csv")
        DF = pd.read_csv(os.path.join(INDIR, f"{bag}", tod.value, f"{bag}_{tod.value}.csv"))
        DF.reset_index(drop=True, inplace=True)
        df = Data(DF, vars = DF.columns, subsampling=Static(STEP))
        output_dir = os.path.join(OUTDIR, f"{bag}", "noRT", tod.value, "static")
        os.makedirs(output_dir, exist_ok=True)
        df.d.to_csv(os.path.join(output_dir, f"{bag}_{tod.value}.csv"), index=False)
        del df
        
        DF = DF.loc[R[tod]:, ["R_X", "R_Y", "G_X", "G_Y"]]
        for wp in WP:
            if wp == WP.PARKING or wp == WP.CHARGING_STATION: continue
            if wp == WP.DOOR_ENTRANCE_CANTEEN:
                tmp_wp = "door_entrance-canteen"
            else:
                tmp_wp = wp.value.replace("-", "_")
            print(f"- {tod.value}_{tmp_wp}.csv")
            WPDF = pd.read_csv(os.path.join(INDIR, f"{bag}", tod.value, f"{bag}_{tod.value}_{tmp_wp}.csv"))
            WPDF.reset_index(drop=True, inplace=True)
            WPDF = WPDF.loc[R[tod]:, ~WPDF.columns.str.contains('^Unnamed')]
            WPDF = WPDF.drop(columns=["T_R"])
            WPDF = pd.concat([DF, WPDF], axis=1)
            
            df = Data(WPDF, vars = WPDF.columns, subsampling=Static(STEP))
            
            output_dir = os.path.join(OUTDIR, f"{bag}", "noRT", tod.value, "static")
            os.makedirs(output_dir, exist_ok=True)
            df.d.to_csv(os.path.join(output_dir, f"{bag}_{tod.value}_{tmp_wp}.csv"), index=False)
            del df
