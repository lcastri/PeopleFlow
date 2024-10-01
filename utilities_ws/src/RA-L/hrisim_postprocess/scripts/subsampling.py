from fpcmci.preprocessing.data import Data
from fpcmci.preprocessing.subsampling_methods.WSDynamic import WSDynamic



D = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/28092024/starting/28092024_starting.csv'
min_window_size = 500
ent_thres = 0.1
data = Data(D, subsampling = WSDynamic(min_window_size, ent_thres))

data.plot_timeseries()