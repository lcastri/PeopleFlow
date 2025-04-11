import math
import os
from fpcmci.preprocessing.data import Data
from fpcmci.preprocessing.subsampling_methods.Static import Static
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
from scipy.signal import butter, filtfilt
from metrics.utils import *
import copy
   
  
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
    plt.show()
    
    
def moving_average_filter(data, window_size):
    return np.convolve(data, np.ones(window_size) / window_size, mode='same')


def low_pass_filter(data, cutoff_freq, sampling_rate, order=4):    
    # Validate cutoff frequency
    nyquist = 0.5 * sampling_rate
    if cutoff_freq >= nyquist:
        raise ValueError(f"Cutoff frequency ({cutoff_freq} Hz) must be less than Nyquist frequency ({nyquist} Hz).")
    
    # Normalize cutoff frequency
    normal_cutoff = cutoff_freq / nyquist
    
    # Check data length
    if len(data) < 3 * order:
        raise ValueError("Data length must be at least 3 times the filter order.")
    
    # Design Butterworth filter
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
        filtered_signal = moving_average_filter(df.values[:,i], 21)
        # filtered_signal = df.values[:,i]
        filtered_signal = filtered_signal[~np.isnan(filtered_signal)]
        
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
        if (df.iloc[r]["G_X"] != -1000 and df.iloc[r]["G_Y"] != -1000 and df.iloc[r]["R_B"] != 0 and df.iloc[r].notnull().all()):
            return r
        
        
def get_subsampling_step(cutoff = 0.5, energy_percentage=0.95, plot = True):
    wp = WP.WA_3_C.value
    rs = {}
    dfs = []
    for bag in BAGNAME:
        for tod in TOD:
            df = pd.read_csv(os.path.join(INDIR, f"{bag}", f"{bag}_{tod.value}.csv"), index_col=0)
            r = get_initrow(df)
            df = df.loc[r:, ["R_X", "R_Y", "R_V", "R_B", 'B_S', f"{wp}_NP", f"{wp}_PD", f"{wp}_BAC"]]
            rs[tod] = r
            dfs.append(df.reset_index(drop=True))
        
    df = pd.concat(dfs, axis=0)
    df.dropna(inplace=True)

    _bw = get_bandwidth(df, SF, cutoff = cutoff, energy_percentage = energy_percentage, plot=plot)
    _ssf = 2*_bw
    _st = 1/_ssf
    _step = int(math.floor(_st * SF))
    
    del df
    return rs, _bw, _ssf, _st, _step


SF = 10 #Hz
INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
OUTDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/shrunk'
# BAGNAME= ['test-obs-01', 'test-obs-02', 'test-obs-04', 'test-obs-05']
BAGNAME= ['noncausal-28022025']

# R, BW, SSF, ST, STEP = get_subsampling_step(cutoff = 0.25, energy_percentage=0.95, plot = False)
# print(f"Bandwidth fm: {BW:.4f} Hz")
# print(f"Subsampling frequency fs >= 2fm = {2*BW:.4f} Hz")
# print(f"Subsampling time 1 sample every each {ST:.4f} s")
# print(f"Subsampling step {STEP}")
# print("")
STEP = 50
        
for bag in BAGNAME:
    print(f"Subsampling {bag}")
    tmp_bag = bag.replace("_", "-")
    for tod in TOD:
        print(f"- {tod.value}.csv")
        DF = pd.read_csv(os.path.join(INDIR, f"{bag}", f"{bag}_{tod.value}.csv"))
        r = get_initrow(DF)
        # DF.dropna(inplace=True)
        # DF.reset_index(drop=True, inplace=True)
        DF = pd.DataFrame(DF.values[r:,:], columns=DF.columns)
        DF.reset_index(drop=True, inplace=True)
        df = Data(DF, vars = DF.columns, subsampling=Static(STEP))
        output_dir = os.path.join(OUTDIR, f"{bag}", tod.value, "static")
        os.makedirs(output_dir, exist_ok=True)
        df.d.to_csv(os.path.join(output_dir, f"{tmp_bag}_{tod.value}.csv"), index=False)
        
                
        # Save WP-specific DataFrames
        general_columns_name = ['pf_elapsed_time', 'TOD', 'T', 'R_V',  'R_B', 'C_S', 'R_X', 'R_Y', 'G_X', 'G_Y', 'OBS']
        for wp in WP:
            if wp == WP.PARKING or wp == WP.CHARGING_STATION: continue
            WPDF = copy.deepcopy(df.d[general_columns_name + [f"{wp.value}_NP", f"{wp.value}_PD"]])
            # WPDF = copy.deepcopy(df.d[general_columns_name + [f"{wp.value}_NP", f"{wp.value}_PD", f"{wp.value}_ELT"]])
                
            # Rename the WP-specific columns
            WPDF = WPDF.rename(columns={f"{wp.value}_NP": "NP", 
                                        f"{wp.value}_PD": "PD"})
            # WPDF = WPDF.rename(columns={f"{wp.value}_NP": "NP", 
            #                             f"{wp.value}_PD": "PD", 
            #                             f"{wp.value}_ELT": "ELT"})
                
            # Add the constant column "wp" with the value wp_id
            WPDF["WP"] = WPS[wp.value]
            
            wp_filename = f"{tmp_bag}_{tod.value}_{wp.value}.csv"
            WPDF.to_csv(os.path.join(output_dir, f"{wp_filename}"), index=False)
            print(f"Saved {wp_filename}")
        del df
