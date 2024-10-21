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
from utils import *

  
def get_pdf(df: Data):
    """
    Compute the probability distribution function from an array of data
    Returns:
        list: probability distribution function
    """
    counts = {}
    for i in range(0, df.T):
        t = tuple(df.d.iloc[i])
        if t in counts:
            counts[t] += 1
        else:
            counts[t] = 1
    pdf = {k: v / df.T for k, v in counts.items()}
    return list(pdf.values())


def get_entropy(df: Data):
    """
    Compute the entropy based on probability distribution function
    """
    return entropy(get_pdf(df), base = 2)
  
  
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
        
        
def get_subsampling_step(cutoff = 1, energy_percentage=0.95, plot = False):
    wp = WP.DELIVERY_POINT.value
    rs = {}
    dfs = []
    for tod in TOD:
        df = pd.read_csv(os.path.join(INDIR, f"{BAGNAME}", tod.value, f"{BAGNAME}_{tod.value}.csv"), index_col=0)
        r = get_initrow(df)
        df = df.loc[r:, ["R_X", "R_Y", "R_V", "R_B", f"{wp}_NP", f"{wp}_PD", f"{wp}_BAC", f"{wp}_pBAC"]]
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
INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/original'
OUTDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/shrunk'
BAGNAME = 'BL100_19102024'


R, BW, SSF, ST, STEP = get_subsampling_step(cutoff = 1, energy_percentage=0.95, plot = True)
print(f"Bandwidth fm: {BW:.4f} Hz")
print(f"Subsampling frequency fs >= 2fm = {2*BW:.4f} Hz")
print(f"Subsampling time 1 sample every each {ST:.4f} s")
print(f"Subsampling step {STEP}")
print("")
       
       
for tod in [TOD.STARTING, TOD.MORNING, TOD.LUNCH, TOD.AFTERNOON, TOD.QUITTING, TOD.OFF]:
    DF = pd.read_csv(os.path.join(INDIR, f"{BAGNAME}", tod.value, f"{BAGNAME}_{tod.value}.csv"), index_col=0)
    DF.reset_index(drop=True, inplace=True)
    DF = DF.loc[R[tod]:, ["R_X", "R_Y", "G_X", "G_Y"]]
    for wp in WP:
        if wp == WP.PARKING or wp == WP.CHARGING_STATION: continue
        WPDF = pd.read_csv(os.path.join(INDIR, f"{BAGNAME}", tod.value, f"{BAGNAME}_{tod.value}_{wp.value}.csv"))
        WPDF.reset_index(drop=True, inplace=True)
        WPDF = WPDF.loc[R[tod]:, ~WPDF.columns.str.contains('^Unnamed')]
        WPDF = WPDF.drop(columns=["R_T"])
        WPDF = pd.concat([DF, WPDF], axis=1)
        print(f"Analysing {BAGNAME}_{tod.value}_{wp.value}.csv ...")
        
        df_original = Data(WPDF, vars = WPDF.columns)
        entropy_original = get_entropy(df_original)
        print(f"Original Entropy: {entropy_original}")
        print("")
        del df_original
        
        df = Data(WPDF, vars = WPDF.columns, subsampling=Static(STEP))
        entropy_static = get_entropy(df)
        print(f"STATIC - Entropy: {entropy_static}")
        print(f"STATIC - Original number of samples: {len(WPDF.values)}")
        print(f"STATIC - Shrunk number of samples: {df.T}")
        print(f"STATIC - Shrunk percentage of samples: {round(df.T/len(WPDF.values)*100, 2)}%")
        
        output_dir = os.path.join(OUTDIR, f"{BAGNAME}", tod.value, "static")
        os.makedirs(output_dir, exist_ok=True)
        df.d.to_csv(os.path.join(output_dir, f"{BAGNAME}_{tod.value}_{wp.value}.csv"), index=False)
        del df
        print("")