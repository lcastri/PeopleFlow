#!/usr/bin/env python

import math
import pickle
import numpy as np
import networkx as nx
import pyAgrum
import pyAgrum.causal as pyc
from enum import Enum
import time
import xml.etree.ElementTree as ET
from enum import Enum
import random
import statistics
import matplotlib.pyplot as plt 
import numpy as np              
from tqdm import tqdm
import json


class TaskResult(Enum):
    SUCCESS = 1
    FAILURE = -1
    CRITICAL_BATTERY = -2

class Task(Enum):
    DELIVERY = "delivery"
    CLEANING = "cleaning"
    CHARGING = "charging"

class TOD(Enum):
    H1 = "H1"
    H2 = "H2"
    H3 = "H3"
    H4 = "H4"
    H5 = "H5"
    H6 = "H6"
    H7 = "H7"
    H8 = "H8"
    H9 = "H9"
    H10 = "H10"
    OFF = "off"

TODS = {t.value: i for i, t in enumerate(TOD)}

class WP(Enum):
    PARKING = "parking"            
    DOOR_ENTRANCE = "door-entrance"
    ENTRANCE = "entrance"
    DOOR_CANTEEN = "door-canteen"
    CORRIDOR_0 = "corridor-0" 
    CORRIDOR_1= "corridor-1"       
    CORRIDOR_2_R= "corridor-2-r"
    CORRIDOR_3_R= "corridor-3-r"
    CORRIDOR_4_R= "corridor-4-r"
    CORRIDOR_5_R= "corridor-5-r"   
    CORRIDOR_6_R= "corridor-6-r"   
    CORRIDOR_7_R= "corridor-7-r"   
    CORRIDOR_2_C= "corridor-2-c"
    CORRIDOR_3_C= "corridor-3-c"
    CORRIDOR_4_C= "corridor-4-c"
    CORRIDOR_5_C= "corridor-5-c"   
    CORRIDOR_6_C= "corridor-6-c"
    CORRIDOR_7_C= "corridor-7-c"
    CORRIDOR_3_L= "corridor-3-l"
    CORRIDOR_4_L= "corridor-4-l"
    CORRIDOR_5_L= "corridor-5-l"   
    SUPPORT_1= "support-1"   
    SUPPORT_2= "support-2"   
    SUPPORT_3= "support-3"   
    SUPPORT_4= "support-4"   
    DOOR_OFFICE_1= "door-office-1" 
    DOOR_OFFICE_2= "door-office-2" 
    DOOR_TOILET_1= "door-toilet-1" 
    DOOR_TOILET_2= "door-toilet-2"
    OFFICE_1= "office-1"
    OFFICE_2= "office-2"
    TOILET_1= "toilet-1"
    SUPPORT_5= "support-5"   
    SUPPORT_6= "support-6"   
    SUPPORT_7= "support-7"   
    SUPPORT_8= "support-8"   
    TOILET_2= "toilet-2"
    DOOR_CORRIDOR_1= "door-corridor-1"
    DOOR_CORRIDOR_2= "door-corridor-2"
    DOOR_CORRIDOR_3= "door-corridor-3"
    TABLE_12= "table-12"           
    TABLE_23= "table-23"           
    SUPPORT_0= "support-0"           
    SUPPORT_9= "support-9"           
    CORRIDOR_CANTEEN_1= "corridor-canteen-1"
    CORRIDOR_CANTEEN_2= "corridor-canteen-2"
    CORRIDOR_CANTEEN_3= "corridor-canteen-3"
    CORRIDOR_CANTEEN_4= "corridor-canteen-4"
    WA_1_R= "wa-1-r"
    WA_2_R= "wa-2-r"
    WA_3_R= "wa-3-r"
    WA_3_CR= "wa-3-cr"
    WA_4_R= "wa-4-r"
    WA_5_R= "wa-5-r"
    WA_1_C= "wa-1-c"
    WA_2_C= "wa-2-c"
    WA_3_C= "wa-3-c"
    WA_4_C= "wa-4-c"
    WA_5_C= "wa-5-c"
    WA_1_L= "wa-1-l"
    WA_2_L= "wa-2-l"
    WA_3_L= "wa-3-l"
    WA_3_CL= "wa-3-cl"
    WA_4_L= "wa-4-l"
    WA_5_L= "wa-5-l"
    TARGET_1= "target-1"
    TARGET_2= "target-2"
    TARGET_3= "target-3"
    TARGET_4= "target-4"
    TARGET_5= "target-5"
    TARGET_6= "target-6"
    TARGET_7= "target-7"
    CHARGING_STATION= "charging-station"

WPS = {wp.value: i for i, wp in enumerate(WP)}


class Robot():
    def __init__(self) -> None:
        self.x = None
        self.y = None
        self.yaw = 0
        self.v = 0
        self.battery_level = 0
        self.is_charging = 0
        self.closest_wp = '' # This is the only robot state we'll need to mock
        self.task_result = 0
        
    
class Time:
    def __init__(self, name, duration) -> None:
        self.name = name
        self.duration = duration
        self.dests = {}
        
        
def heuristic(a, b):
    pos = nx.get_node_attributes(G, 'pos')
    (x1, y1) = pos[a]
    (x2, y2) = pos[b]
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def get_info(D, auditDict, var):
    if auditDict[var]['method'] == 'NoDiscretization':
        edges = auditDict[var]['values']
        midpoints = auditDict[var]['values']
        return 'NoDiscretization', edges, midpoints
    elif 'param' in auditDict[var] and isinstance(auditDict[var]['param'], list):
        edges = auditDict[var]['param']
        midpoints = [(edges[i] + edges[i+1]) / 2.0 for i in range(len(edges)-1)]
        return auditDict[var]['param'], edges, midpoints
    quantiles = np.linspace(0, 100, auditDict[var]['param'] + 1 if 'param' in auditDict[var] else auditDict[var]['nbBins'] + 1)
    edges = np.percentile(D[var].values, quantiles)
    midpoints = [(edges[i] + edges[i+1]) / 2.0 for i in range(len(edges)-1)]
    return quantiles, edges, midpoints


def readScenario(scenario):
    # Load and parse the XML file
    schedule = {}
    tree = ET.parse(scenario)
    root = tree.getroot()
        
    # Parse schedule
    for time in root.find('schedule').findall('time'):
        tmp = Time(time.get('name'), float(time.get('duration')))
        for adddest in time.findall('adddest'):
            dest_name = adddest.get('name')
            tmp.dests[dest_name] = {'mean': float(adddest.get('p')), 'std': float(adddest.get('std'))}
        schedule[tmp.name] = tmp
        
    wps = {}
    for waypoint in root.findall('waypoint'):
        waypoint_id = waypoint.get('id')
        x = float(waypoint.get('x'))
        y = float(waypoint.get('y'))
        r = float(waypoint.get('r'))
        a = math.pi * r**2
        wps[waypoint_id] = {'x': x, 'y': y, 'r': r, 'A': a}

    return schedule, wps


class PredictionManager:
    def __init__(self, G, WPS_COORD, SCHEDULE, CIEDIR, PREDICTION_STEP, ROBOT_MAX_VEL):
        """
        Class constructor. (NON-ROS VERSION)
        """
        print("Initializing Prediction Manager...")
        self.robot = Robot()
        
        # Store parameters passed from main
        self.G = G
        self.WPS_COORD = WPS_COORD # This is the *filtered* dict
        self.SCHEDULE = SCHEDULE
        self.CIEDIR = CIEDIR
        self.PREDICTION_STEP = PREDICTION_STEP
        self.ROBOT_MAX_VEL = ROBOT_MAX_VEL
        
        # We don't have subscribers, so we'll mock these values
        self.WPs = {}
        self.PDs = {}
        self.TOD = ''        
        self.hhmmss = ''
        self.elapsed = 0 # Will be set by the run_prediction method

        # --- Load CIE models ---
        print("Loading CIE models...")
        AUDITs = pickle.load(open(f"{self.CIEDIR}/AUDITs.pkl", "rb"))
        Ds = pickle.load(open(f"{self.CIEDIR}/Ds.pkl", "rb"))
        self.CIE = {}
        for wp in self.WPS_COORD.keys():
            # These WPs are part of the graph but we don't predict for them
            if wp in ['parking', 'charging-station']: continue 
            # print(f"  Loading model for {wp}")
            bn = pyAgrum.loadBN(f"{self.CIEDIR}/CIE_{wp}.bifxml")
            cm = pyc.CausalModel(bn)
            self.CIE[wp] = {'audit': AUDITs[wp], 'd': Ds[wp], 'bn': bn, 'cm': cm}
            
        quantiles_RV, edges_RV, midpoints_RV = get_info(Ds['target-3'], AUDITs['target-3'], "RVt")
        quantiles_EC, edges_EC, midpoints_EC = get_info(Ds['target-3'], AUDITs['target-3'], "ECt")
        self.RV_info = {'quantiles': quantiles_RV, 'edges': edges_RV, 'midpoints': midpoints_RV}
        self.EC_info = {'quantiles': quantiles_EC, 'edges': edges_EC, 'midpoints': midpoints_EC}
               
        # --- Calculate Travel Times ---
        print("Calculating travel times...")
        self.TTWP = self.get_ttwp() # Uses self.G
        self.ARCs = []
        for arc in self.TTWP.keys():
            wp_i, wp_j = arc
            if wp_i in ['parking', 'charging-station']: continue            
            if wp_j in ['parking', 'charging-station']: continue   
            self.ARCs.append('__'.join(arc))
        
        print(f"Prediction Manager ready! ({len(self.WPS_COORD)} waypoints, {len(self.ARCs)} arcs)")     
                                      
    def get_ttwp(self, relative=False):
        """
        Calculate the time to travel between each pair of waypoints in the graph.
        (NON-ROS VERSION: Uses self.G.nodes['pos'] instead of WPS_COORD)
        """
        travelled_distances = {}
        
        # Get node positions directly from the graph object
        try:
            pos = nx.get_node_attributes(self.G, 'pos')
            if not pos:
                raise KeyError
        except KeyError:
            print("ERROR in get_ttwp: Graph nodes are missing 'pos' attribute (x, y coordinates).")
            print("Please ensure your graph.pkl file has node positions stored.")
            print("You can check with: G = pickle.load(...); print(nx.get_node_attributes(G, 'pos'))")
            return {}

        if relative:
            for wp in self.WPS_COORD.keys():
                path = nx.astar_path(self.G, self.robot.closest_wp, wp, heuristic=heuristic, weight='weight')
                travelled_distance = 0
                for wp_idx in range(1, len(path)):
                    wp_current = path[wp_idx-1]
                    wp_next = path[wp_idx]
                    (x_curr, y_curr) = pos[wp_current]
                    (x_next, y_next) = pos[wp_next]
                    travelled_distance += math.sqrt((x_next - x_curr)**2 + (y_next - y_curr)**2)
                travelled_distances[wp] = math.ceil((travelled_distance/self.ROBOT_MAX_VEL)/self.PREDICTION_STEP)
        else:
            for arc in self.G.edges():
                wp_i, wp_j = arc
                (x_i, y_i) = pos[wp_i]
                (x_j, y_j) = pos[wp_j]
                travelled_distance = math.sqrt((x_i - x_j)**2 + (y_i - y_j)**2)
                travelled_distances[arc] = math.ceil((travelled_distance/self.ROBOT_MAX_VEL)/self.PREDICTION_STEP)
        return travelled_distances
                       
            
    def elapsed2TOD(self, t):
        d = 0
        for time in self.SCHEDULE:
            d += self.SCHEDULE[time].duration
            if t > d:
                continue
            else:
                return self.SCHEDULE[time].name

            
    def predict_BC(self, rv, cs):
        def find_bin(value, edges):
            idx = np.digitize(value, edges, right=False) - 1
            return int(max(0, min(idx, len(edges) - 2)))
    
        RV_bin_idx = find_bin(rv, self.RV_info['edges'])
        cm = self.CIE['target-3']['cm']
        evidence = {"RVt": RV_bin_idx, "CSt": cs}
        _, adj, _ = pyc.causalImpact(cm, on="ECt", doing="RVt", knowing={"CSt"}, values=evidence)
        posterior_causal = adj.toarray()
        pred_causal = sum(posterior_causal[j] * self.EC_info['midpoints'][j] for j in range(len(posterior_causal)))
        return pred_causal
    
    
    def predict_PD(self, tod, wp):
        wp_bin = 0

        tod_bin = list(TODS.keys()).index(tod)

        cm = self.CIE[wp]['cm']
        dwp = self.CIE[wp]['d']
        _, _, midpoints_PD = get_info(dwp, self.CIE[wp]['audit'], 'PD0')
      
        evidence = {"TOD0": tod_bin, "WP0": wp_bin}
        _, adj, _ = pyc.causalImpact(cm, on="PD0", doing="TOD0", knowing={"WP0"}, values=evidence)
        posterior_causal = adj.toarray()
        pred_causal = sum(posterior_causal[j] * midpoints_PD[j] for j in range(len(posterior_causal)))
        return pred_causal
    

    def run_prediction(self, mock_closest_wp, mock_elapsed_time):
        """
        This replaces handle_get_risk_map.
        It takes mock state as input and returns a dict of results.
        """
        start_time = time.perf_counter()
        
        # Set the mock state
        self.robot.closest_wp = mock_closest_wp
        self.elapsed = mock_elapsed_time
        
        TTWP_relative = self.get_ttwp(relative=True)
               
        PD_wps = {}
        PD_infs = {}
        PDs = []
        BCs = []
        PD_inf_time = []
        BC_inf_time = []
        
        # 1. Predict PD for all waypoints
        for wp in self.WPS_COORD.keys():
            if wp in ['parking', 'charging-station']: continue
            traversal_step = TTWP_relative[wp]
            tod = self.elapsed2TOD(self.elapsed + traversal_step * self.PREDICTION_STEP)
            
            PD_start_time = time.perf_counter()
            PD_wps[wp] = self.predict_PD(tod, wp)
            PD_end_time = time.perf_counter()
            PD_infs[wp] = PD_end_time - PD_start_time
            
        # 2. Predict BC for all arcs
        for arc in self.ARCs:
            wp_i, wp_j = arc.split('__')
            if wp_i in ['parking', 'charging-station']: continue            
            if wp_j in ['parking', 'charging-station']: continue            
            traversal_step = self.TTWP[(wp_i, wp_j)]
                        
            BC_start_time = time.perf_counter()
            BCs.append(self.predict_BC(rv=self.ROBOT_MAX_VEL, cs=0)*traversal_step)
            BC_end_time = time.perf_counter()
            BC_inf_time.append(BC_end_time - BC_start_time)
            
            # Use the already-computed PDs
            PDs.append((PD_wps[wp_i] + PD_wps[wp_j])/2)
            PD_inf_time.append(PD_infs[wp_i] + PD_infs[wp_j])
            
        end_time = time.perf_counter()
        tot_inf_time = end_time - start_time
                    
        # Return results as a simple dictionary
        return {
            "ARCs": self.ARCs,
            "PDs": PDs,
            "BCs": BCs,
            "tot_inf_time": tot_inf_time,
            "PD_inf_time": PD_inf_time,
            "BC_inf_time": BC_inf_time
        }
        

def plot_results(results_summary):
    """
    Generates a plot of scalability results (Time vs. Waypoints) with error bars,
    connecting the mean values.
    """
    
    # Extract data for plotting
    scenarios = list(results_summary.keys())
    # Sort by key (which is the waypoint count)
    scenarios.sort(key=lambda s: int(s)) 
    
    # --- MODIFIED ---
    x_values = [results_summary[s]['waypoints'] for s in scenarios] # Use waypoints for X
    # --- END MODIFIED ---
    
    y_values = [results_summary[s]['time_mean'] for s in scenarios]
    y_errors = [results_summary[s]['time_stdev'] for s in scenarios]

    print(f"Plotting data:")
    print(f"  Waypoints (X): {x_values}")
    print(f"  Arcs:          {[results_summary[s]['arcs'] for s in scenarios]}") # Print arcs as info
    print(f"  Time (Y):      {[round(y, 4) for y in y_values]}")
    print(f"  StdDev (E):    {[round(e, 4) for e in y_errors]}")

    plt.figure(figsize=(10, 6))
    
    # Plot error bars and markers
    plt.errorbar(x_values, y_values, 
                 yerr=y_errors, 
                 fmt='s',                     # Square markers
                 capsize=5,                   # Error bar caps
                 linestyle='None',            # No line connecting markers
                 color='tab:blue')
    
    # Plot the line connecting the means
    plt.plot(x_values, y_values, 
             linestyle='--',       # Dashed line
             marker=None,          # No markers on this line
             color='tab:blue')

    plt.title('Causal Prediction Scalability', fontsize=16)
    plt.xlabel('Number of Waypoints', fontsize=12)
    plt.ylabel('Mean Inference Time (s)', fontsize=12)
    
    # Set x-axis ticks to show *some* of the data points for clarity
    if len(x_values) > 10:
        step = len(x_values) // 5
        plt.xticks(x_values[::step])
    else:
        plt.xticks(x_values)
    
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.legend()
    plt.tight_layout()

    # --- MODIFIED ---
    plot_filename = "scalability_plot.png" # New filename
    # --- END MODIFIED ---
    
    plt.savefig(plot_filename)
    print(f"Plot saved to {plot_filename}")

    # Optionally, show the plot in a new window
    # plt.show()


# if __name__ == "__main__":
        
 

    # # --- Save Results to JSON ---
    # results_filename = "/home/hrisim/ros_ws/src/HRISim/prediction/hrisim_prediction_manager/scripts/scalability_results_incremental.json"
    # with open(results_filename, 'r') as f:
    #     results_summary = json.load(f)
    # plot_results(results_summary)

    # print("Script finished.")
if __name__ == "__main__":
        
    # --- USER CONFIGURATION ---
    CIEDIR = "/home/hrisim/ros_ws/src/HRISim/prediction/hrisim_prediction_manager/CIEs/CIE3.8"     
    G_PATH = "/home/hrisim/ros_ws/src/HRISim/peopleflow/peopleflow_manager/res/warehouse/graph.pkl"
    SCENARIO_PATH = "/home/hrisim/ros_ws/src/pedsim_ros/pedsim_simulator/scenarios/warehouse.xml"
        
    PREDICTION_STEP = 5
    ROBOT_MAX_VEL = 0.5
    NUM_RUNS = 1000
    
    # --- Incremental Test Configuration ---
    STARTING_WP = 'target-1' # Seed node for the BFS traversal
    STEP_SIZE = 10
    # --- END USER CONFIGURATION ---

    # --- Load Data Files ---
    print("Loading all data files...")
    try:
        with open(G_PATH, 'rb') as f:
            G = pickle.load(f) # Load the full graph
        SCHEDULE, WPS_COORD_FULL = readScenario(SCENARIO_PATH) # Load all WPs
            
    except FileNotFoundError as e:
        print(f"Error loading file: {e}")
        exit(1)
    except Exception as e:
        print(f"An error occurred loading data: {e}")
        exit(1)

    # --- Generate Waypoint List via BFS ---
    print(f"Generating BFS traversal from seed: {STARTING_WP}")
    bfs_tree = nx.bfs_tree(G, source=STARTING_WP)
    all_nodes_in_bfs_order = list(bfs_tree.nodes()) 
       
    all_predictable_wps = list(WPS_COORD_FULL.keys())
    ordered_wps_to_test = [wp for wp in all_nodes_in_bfs_order if wp in all_predictable_wps]
    
    max_wps = len(ordered_wps_to_test)
    print(f"Found {max_wps} total predictable waypoints.")

    # --- Define the Scenario Sizes ---
    scenario_sizes = list(range(STEP_SIZE, max_wps, STEP_SIZE))
    if scenario_sizes[-1] != max_wps:
        scenario_sizes.append(max_wps)

    print(f"Will test scenarios with waypoint counts: {scenario_sizes}")

    results_summary = {}

    # --- Main Test Loop ---
    for k_wps in scenario_sizes:
        print(f"\n--- RUNNING SCENARIO: {k_wps} Waypoints ---")
        
        active_wps_list = ordered_wps_to_test[:k_wps]
        
        graph_nodes = active_wps_list + ['parking', 'charging-station']
        active_G = G.subgraph([wp for wp in graph_nodes if wp in G.nodes()])
        
        active_WPS_COORD = {wp: WPS_COORD_FULL[wp] for wp in active_wps_list if wp in WPS_COORD_FULL}
        
        print("Instantiating Prediction Manager...")
        pm = PredictionManager(
            G=active_G,
            WPS_COORD=active_WPS_COORD,
            SCHEDULE=SCHEDULE,
            CIEDIR=CIEDIR,
            PREDICTION_STEP=PREDICTION_STEP,
            ROBOT_MAX_VEL=ROBOT_MAX_VEL
        )
        
        print(f"Running {NUM_RUNS} repetitions...")
        timings = []
        
        for i in tqdm(range(NUM_RUNS), desc=f"Scenario {k_wps}-WPs"):
            mock_wp = random.choice(active_wps_list)
            mock_time = random.randint(600, 82200)
            
            results = pm.run_prediction(mock_wp, mock_time)
            timings.append(results['tot_inf_time'])

        mean_time = statistics.mean(timings)
        stdev_time = statistics.stdev(timings)
        
        # Use str(k_wps) as the key for JSON compatibility
        results_summary[str(k_wps)] = {
            "waypoints": k_wps,
            "arcs": len(pm.ARCs),
            "time_mean": mean_time,
            "time_stdev": stdev_time,
            "num_runs": NUM_RUNS
        }

    # --- Print Final Summary ---
    print("\n\n--- SCALABILITY TEST SUMMARY (STATISTICAL) ---")
    print("--------------------------------------------------")
    for k in sorted(results_summary.keys(), key=int): # Sort keys as integers
        data = results_summary[k]
        print(f"Scenario: {data['waypoints']} Waypoints")
        print(f"  Arcs:      {data['arcs']}")
        print(f"  Runs:      {data['num_runs']}")
        print(f"  Time Mean (s):  {data['time_mean']:.6f}")
        print(f"  Time StDev (s): {data['time_stdev']:.6f}")
    print("--------------------------------------------------")

    # --- Generate Plot ---
    print("Generating plot...")
    plot_results(results_summary)

    # --- Save Results to JSON ---
    results_filename = "scalability_results_incremental.json"
    print(f"Saving results to {results_filename}...")
    try:
        with open(results_filename, 'w') as f:
            json.dump(results_summary, f, indent=4)
    except Exception as e:
        print(f"Error saving results file: {e}")

    print("Script finished.")