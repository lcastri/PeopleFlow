import itertools
import pickle
import os
from metrics_utils import *
from utils import *
from tqdm import tqdm
from scipy.stats import chi2_contingency
from scipy.stats import poisson
import statsmodels.api as sm
import statsmodels.formula.api as smf
import pandas as pd
from scipy.stats import fisher_exact

INDIR = '/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/csv/HH/original'
BAGNAMES = ['base', 'causal']
CATEGORIES = {'base': 'Baseline', 'causal': 'Causal'}
OUTDIR = os.path.join('/home/lcastri/git/PeopleFlow/utilities_ws/src/RA-L/hrisim_postprocess/results', 'comparison', '__'.join(BAGNAMES), 'overall')
os.makedirs(OUTDIR, exist_ok=True)

pval_success_failure = []
pval_success_failure2 = []
pval_dangerous_interaction = []
pval_dangerous_interaction2 = []
pval_working_time = {bagname: {"Active Time": [], "Stalled Time": [], "Wasted Time": []} for bagname in BAGNAMES}
pval_travelled_distance = {bagname: {"Planned Travelled Distance": [], "Extra Travelled Distance": [], "Wasted Travelled Distance": []} for bagname in BAGNAMES}
pval_battery = {bagname: {"Effective Battery Usage": [], "Wasted Battery Usage": []} for bagname in BAGNAMES}
pval_proxemics = {bagname: {"Distances": []} for bagname in BAGNAMES}

# Load metrics for each bag
for bagname in BAGNAMES:
    metrics_path = os.path.join(INDIR, bagname, "metrics.pkl")
    with open(metrics_path, 'rb') as pkl_file:
        METRICS = pickle.load(pkl_file)
    pval_success_failure.append([METRICS['overall_success'], METRICS['overall_failure_people'], METRICS['overall_failure_critical_battery']])
    pval_success_failure2.append([METRICS[tod.value][i]['result'] for tod in TOD for i in range(200) if tod == TOD.OFF and i < 90 or tod != TOD.OFF])
    pval_dangerous_interaction.append([METRICS['overall_human_collision']])
    pval_dangerous_interaction2.append([METRICS[tod.value][i]['human_collision'] for tod in TOD for i in range(200) if tod == TOD.OFF and i < 90 or tod != TOD.OFF])
    for tod in TOD:
        metrics_tod = METRICS[tod.value]
        NTASK = 200 if tod != TOD.OFF else 90
        for task in tqdm(list(np.arange(NTASK)), desc=f"{bagname}-{tod.value}"):
            pval_travelled_distance[bagname]["Planned Travelled Distance"].append(metrics_tod[task]['travelled_distance_planned'])
            if metrics_tod[task]['result'] == 1:
                pval_working_time[bagname]["Active Time"].append(metrics_tod[task]['time_to_reach_goal_actual'])
                pval_working_time[bagname]["Stalled Time"].append(metrics_tod[task]['time_to_reach_goal_stalled'])
                pval_travelled_distance[bagname]["Extra Travelled Distance"].append(metrics_tod[task]['travelled_distance_actual'] - metrics_tod[task]['travelled_distance_planned'])
                pval_battery[bagname]["Effective Battery Usage"].append(metrics_tod[task]['battery_consumption_actual'])
            else:
                pval_working_time[bagname]["Wasted Time"].append(metrics_tod[task]['time_to_reach_goal_wasted'])
                pval_travelled_distance[bagname]["Wasted Travelled Distance"].append(metrics_tod[task]['travelled_distance_wasted'])
                pval_battery[bagname]["Wasted Battery Usage"].append(metrics_tod[task]['battery_consumption_wasted'])
            pval_proxemics[bagname]["Distances"].extend(
                value for value in itertools.chain(*metrics_tod[task]['agent_distances'].values()) if value < 7.6
            )

metrics = {
    # "pval_success_failure": ["N. Success", "N. Failures (People)", "N. Failures (Critical Battery)"],
    "pval_success_failure2": ["N. Success", "N. Failures (People)", "N. Failures (Critical Battery)"],
    # "pval_dangerous_interaction": ["N. Dangerous Interactions"],
    "pval_dangerous_interaction2": ["N. Dangerous Interactions"],
    "pval_working_time": ["Active Time", "Stalled Time", "Wasted Time"],
    "pval_travelled_distance": ["Planned Travelled Distance", "Extra Travelled Distance", "Wasted Travelled Distance"],
    "pval_battery": ["Effective Battery Usage", "Wasted Battery Usage"],
    # "pval_proxemics": ["Distances"],
}
pvalues = {}

# Compute p-values for each metric
for metric_type, metric_list in metrics.items():
    print()
    print(metric_type)
    
    if metric_type == "pval_success_failure":
        # Use Chi-Square test for categorical data
        # for i, category in enumerate(metrics[metric_type]):
        #     data = eval(metric_type)  # Get the data dictionary dynamically
            
        #     # Store p-values
        #     ppvalues = {}

        #     # Create a 2x2 contingency table for this specific category
        #     contingency_table = [
        #         [data[0][i], data[1][i]],  # Baseline vs Causal for this category
        #         [sum(data[0]) - data[0][i], sum(data[1]) - data[1][i]]  # Complement category
        #     ]
                
        #     # Run Chi-Square test for this category
        #     if i == 2:
        #         test_used = "Fisher’s Exact"
        #         p_value = fisher_exact(contingency_table)[1]
        #     else:
        #         test_used = "Chi-Square"
        #         _, p_value, _, _ = chi2_contingency(contingency_table)
                
        #     # Store and print the p-value
        #     print(f"{test_used} Test for {category}: p-value = {p_value:.2e}")
        #     pvalues[category] = {"p_value": p_value, "test_used": test_used}
        # Compute Overall Chi-Square Test
        overall_contingency_table = [
            pval_success_failure[0],  # Baseline row (Success, Failure-People, Failure-Battery)
            pval_success_failure[1]   # Causal row
        ]
        test_used = "Chi-Square"
        chi2_stat, overall_p_value, _, _ = chi2_contingency(overall_contingency_table)

        # Print only the overall result
        print(f"Overall Chi-Square Test: p-value = {overall_p_value:.2e}")
        pvalues[metric_type] = {"p_value": overall_p_value, "test_used": test_used}

        del pval_success_failure
    # elif metric_type == "pval_dangerous_interaction":
    #     metric_name = "N. Dangerous Interactions"
    #     # Example values (replace X and Y with actual counts)
    #     dangerous_base = pval_dangerous_interaction[0][0]
    #     dangerous_causal = pval_dangerous_interaction[1][0]

    #     # Perform Poisson test (approximate method for comparing two Poisson rates)
    #     rate_ratio = dangerous_causal / dangerous_base
    #     p_value = poisson.cdf(dangerous_causal, dangerous_base) + (1 - poisson.cdf(dangerous_base, dangerous_causal))
    #     test_used = "Poisson"
    #     print(f"{metric_name}: {test_used}, p-value = {p_value:.2e}")
    #     pvalues[metric_name] = {"p_value": p_value, "test_used": test_used}
    #     del pval_dangerous_interaction
    elif metric_type == "pval_dangerous_interaction2":
        metric_name = "N. Dangerous Interactions"
        dangerous_base = pval_dangerous_interaction2[0]  # List of collisions per task (Baseline)
        dangerous_causal = pval_dangerous_interaction2[1]  # List of collisions per task (Causal)


        # Convert per-task collision counts into a single list
        collisions = pval_dangerous_interaction2[0] + pval_dangerous_interaction2[1]

        # Create a corresponding condition label for each task
        conditions = ["Baseline"] * len(pval_dangerous_interaction2[0]) + ["Causal"] * len(pval_dangerous_interaction2[1])

        # Create DataFrame
        df = pd.DataFrame({
            "Collisions": collisions,  # Number of collisions per task
            "Condition": conditions    # "Baseline" or "Causal"
        })

        # Fit Negative Binomial regression model
        model = smf.glm("Collisions ~ Condition", data=df, family=sm.families.NegativeBinomial()).fit()
        p_value = model.pvalues["Condition[T.Causal]"]
        test_used = "Negative Binomial Regression"

        print(f"Negative Binomial Test: p-value = {p_value:.2e}")

        pvalues[metric_name] = {"p_value": p_value, "test_used": test_used}
        del pval_dangerous_interaction2

    else:
        # Use normality + t-test / Mann-Whitney U for continuous data
        for metric_name in metric_list:
            data = eval(metric_type)  # Get the data dictionary dynamically
            p_value, test_used = compute_p_values(data, metric_name)
            print(f"{metric_name}: {test_used}, p-value = {p_value:.2e}")
            pvalues[metric_name] = {"p_value": p_value, "test_used": test_used}
        del data 
        del globals()[metric_type]

# Save the p-values dictionary to a pickle file
pvalues_path = os.path.join(OUTDIR, "pvalues.pkl")
with open(pvalues_path, "wb") as pkl_file:
    pickle.dump(pvalues, pkl_file)

print(f"P-values saved to: {pvalues_path}")