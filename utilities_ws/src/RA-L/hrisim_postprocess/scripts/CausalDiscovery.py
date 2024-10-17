# Imports
import os
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd

from tigramite.jpcmciplus import JPCMCIplus
from tigramite.independence_tests.parcorr_mult import ParCorrMult
from tigramite.independence_tests.regressionCI import RegressionCI
from tigramite.independence_tests.gpdc_torch import GPDCtorch
import tigramite.data_processing as pp
import tigramite.plotting as tp
from utils import *


INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv'
BAGNAME = ['BL100_12102024', 'BL50_13102024'] 
# SSMode = ('cutoff_02', 'static')
SSMode = ('noRT', 'static')
# SSMode = ()

node_classification = {
    NODES.TOD.value: "space_context",
    NODES.R_V.value: "system",
    # NODES.R_T.value: "system",
    NODES.R_B.value: "system",
    # NODES.NP.value: "system",
    NODES.PD.value: "system",
    NODES.BAC.value: "system",
    NODES.WP.value: "space_context",
}

var_names = [n.name for n in NODES]
DATA_DICT = {}
DATA_TYPE = {}
for bagname in BAGNAME:
    for tod in TOD:
        for wp in WP:
            if wp == WP.PARKING or wp == WP.CHARGING_STATION: continue
            print(f"Loading : {bagname}-{tod.value}-{wp.value}")
            if len(SSMode) > 0:
                filename = os.path.join(INDIR, "shrunk", f"{bagname}", SSMode[0], tod.value, SSMode[1], f"{bagname}_{tod.value}_{wp.value}.csv")
            else:
                filename = os.path.join(INDIR, "original", f"{bagname}", tod.value, f"{bagname}_{tod.value}_{wp.value}.csv")
            WPDF = pd.read_csv(filename)
            
            # Check for NaN values
            if WPDF.isnull().values.any():
                print(f"Warning: NaN values found in {filename}. Skipping this file.")
            
            idx = len(DATA_DICT)
            DATA_DICT[idx] = WPDF[var_names].values
            DATA_TYPE[idx] = np.zeros(WPDF[var_names].values.shape, dtype='int')
            DATA_TYPE[idx][:, NODES.TOD.value] = 1
            DATA_TYPE[idx][:, NODES.R_V.value] = 0
            # DATA_TYPE[idx][:, NODES.R_T.value] = 1
            DATA_TYPE[idx][:, NODES.R_B.value] = 0
            # DATA_TYPE[idx][:, NODES.NP.value] = 0
            DATA_TYPE[idx][:, NODES.PD.value] = 0
            DATA_TYPE[idx][:, NODES.BAC.value] = 0
            DATA_TYPE[idx][:, NODES.WP.value] = 1

# cond_ind_test=GPDCtorch()
cond_ind_test=RegressionCI()
# cond_ind_test=ParCorrMult(significance='analytic')

dataframe = pp.DataFrame(
    data = DATA_DICT,
    data_type = DATA_TYPE if isinstance(cond_ind_test, RegressionCI) else None,
    analysis_mode = 'multiple',
    var_names = var_names
    )
    
jpcmciplus = JPCMCIplus(dataframe=dataframe,
                        cond_ind_test=cond_ind_test, 
                        node_classification=node_classification,
                        verbosity=2,)

# Define the analysis parameters.
tau_max = 1
pc_alpha = 0.01

# Run J-PCMCI+
results = jpcmciplus.run_jpcmciplus(tau_min = 0, 
                                    tau_max = tau_max, 
                                    pc_alpha = pc_alpha)

tp.plot_graph(results['graph'], val_matrix=results['val_matrix'], var_names=var_names)
tp.plot_time_series_graph(results['graph'], val_matrix = results['val_matrix'], 
              node_classification = node_classification, var_names=var_names)
plt.show()