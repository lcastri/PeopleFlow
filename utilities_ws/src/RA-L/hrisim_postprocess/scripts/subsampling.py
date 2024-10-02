from fpcmci.preprocessing.data import Data
from fpcmci.preprocessing.subsampling_methods.WSDynamic import WSDynamic
import pandas as pd


D = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/02102024_shrink/starting/2102024_starting_corridor0.csv'
DF = pd.read_csv(D)
for r in range(len(DF)):
    if (DF.iloc[r]["g_x"] != -1000 and DF.iloc[r]["g_y"] != -1000 
        and DF.iloc[r].notnull().all()):
        break
DF = DF.loc[r:, ["time_of_day", "r_v", "r_T", "r_battery", "toilet2_np", "toilet2_pd", "toilet2_bac"]]
min_window_size = len(DF)
ent_thres = 0.1
data = Data(DF, subsampling = WSDynamic(min_window_size, ent_thres))

data.plot_timeseries()