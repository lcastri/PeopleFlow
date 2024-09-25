# Imports
import numpy as np
from numpy.random import SeedSequence
from matplotlib import pyplot as plt

from tigramite.toymodels import structural_causal_processes as toys
from tigramite.toymodels.context_model import ContextModel
from tigramite.jpcmciplus import JPCMCIplus
from tigramite.independence_tests.parcorr_mult import ParCorrMult
import tigramite.data_processing as pp
import tigramite.plotting as tp


# Set seeds for reproducibility
seed = 12345
ss = SeedSequence(seed)
noise_seed = ss.spawn(1)[0]

random_state = np.random.default_rng(noise_seed)

# Choose the time series length and number of spatial contexts
T = 100
nb_domains = 50

transient_fraction=0.2
tau_max = 2
frac_observed = 0.5

# Specify the model
def lin(x): return x

links = {0: [((0, -1), 0.3, lin), ((3, -1), 0.6, lin), ((4, -1), 0.9, lin)],
         1: [((1, -1), 0.4, lin), ((3, -1), 0.4, lin)],
         2: [((2, -1), 0.3, lin), ((1, -2), -0.5, lin), ((4, -1), 0.5, lin), ((5, 0), 0.6, lin)] ,
         3: [], 
         4: [], 
         5: []
            }

# Specify which node is a context node via node_type (can be "system", "time_context", or "space_context")
node_classification = {
    0: "system",
    1: "system",
    2: "system",
    3: "time_context",
    4: "time_context",
    5: "space_context"
}

# Specify dynamical noise term distributions, here unit variance Gaussians
noises = [random_state.standard_normal for j in range(6)]

contextmodel = ContextModel(links=links, node_classification=node_classification,
                            noises=noises, 
                            seed=seed)

data_ens, nonstationary = contextmodel.generate_data(nb_domains, T)

assert not nonstationary

system_indices = [0,1,2]
observed_indices_time = [3,4]
observed_indices_space = [5]

# all system variables are also observed, thus we get the following observed data
observed_indices = system_indices + observed_indices_time + observed_indices_space
data_dict = {key: data_ens[key][:,observed_indices] for key in data_ens}


# Define vector-valued variables including dummy variables as well as observed (system and context) variables
nb_observed_context_nodes = len(observed_indices_time) + len(observed_indices_space)
N = len(system_indices)
process_vars = system_indices
observed_temporal_context_nodes = list(range(N, N + len(observed_indices_time)))
observed_spatial_context_nodes = list(range(N + len(observed_indices_time), 
                                            N + len(observed_indices_time) + len(observed_indices_space)))

vector_vars = {i: [(i, 0)] for i in process_vars + observed_temporal_context_nodes + observed_spatial_context_nodes}
# Name all the variables and initialize the dataframe object
# Be careful to use analysis_mode = 'multiple'
sys_var_names = ['$X_%s$' % str(i) for i in process_vars]
context_var_names = ['t-$C_%s$' % str(i) for i in observed_indices_time] + ['s-$C_%s$' % str(i) for i in observed_indices_space]
var_names = sys_var_names + context_var_names


dataframe = pp.DataFrame(
    data=data_dict,
    vector_vars = vector_vars,
    analysis_mode = 'multiple',
    var_names = var_names
    )
    
jpcmciplus = JPCMCIplus(dataframe=dataframe,
                          cond_ind_test=ParCorrMult(significance='analytic'), 
                          node_classification=node_classification,
                          verbosity=1,)

# Define the analysis parameters.
tau_max = 2
pc_alpha = 0.01

# Run J-PCMCI+
results = jpcmciplus.run_jpcmciplus(tau_min=0, 
                              tau_max=tau_max, 
                              pc_alpha=pc_alpha)

tp.plot_graph(results['graph'], val_matrix=results['val_matrix'], var_names=var_names)
tp.plot_time_series_graph(results['graph'], val_matrix = results['val_matrix'], 
              node_classification = node_classification, var_names=var_names)
plt.show()