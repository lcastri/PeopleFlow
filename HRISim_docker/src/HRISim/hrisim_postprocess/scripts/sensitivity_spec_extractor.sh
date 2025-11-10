#!/bin/bash

# --- Color Definitions ---
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# --- Parameter Sweep Definitions ---
TIMES_TO_TEST=("H2" "H6")

# --- DEFINE THE TRiPLES YOU WANT TO RUN HERE ---
# Use '01' for 0.1 and '05' for 0.5 to match your names
# Example: Run (Kd=1, Kpd=10, Kbc=5) and (Kd=10, Kpd=100, Kbc=50)
KD_RUNS=(1)
KP_RUNS=(100)
KB_RUNS=(50)
# --- END OF TRIPLES DEFINITION ---

# --- Script Main Logic ---
num_combos=${#KD_RUNS[@]}
total_runs=$((${#TIMES_TO_TEST[@]} * num_combos))

echo -e "${GREEN}===================================================${NC}"
echo -e "${GREEN}Starting rosbag post-processing batch run...${NC}"
echo -e "${GREEN}Total bags to process: $total_runs${NC}"
echo -e "${GREEN}===================================================${NC}"

current_run=1

# Loop over H2 and H6
for t in "${TIMES_TO_TEST[@]}"; do

  # Loop through the specific combinations
  for (( i=0; i<${num_combos}; i++ )); do
    
    # Get the parameters for this specific run
    kd=${KD_RUNS[$i]}
    kp=${KP_RUNS[$i]}
    kb=${KB_RUNS[$i]}
    
    # Construct the bag name based on your new format
    launch_bagname="KD${kd}_KPD${kp}_KBC${kb}-${t}"

    echo ""
    echo -e "${CYAN}===================================================${NC}"
    echo -e "${CYAN}Processing Run $current_run / $total_runs${NC}"
    echo -e "${CYAN}Bag name: $launch_bagname${NC}"
    echo -e "${CYAN}===================================================${NC}"

    # Run the roslaunch command for post-processing
    roslaunch hrisim_postprocess HH_bringup.launch bagname:="$launch_bagname"

    echo ""
    echo -e "${GREEN}===================================================${NC}"
    echo -e "${GREEN}Run $current_run complete${NC}"
    echo -e "${GREEN}===================================================${NC}"
    
    ((current_run++))
    
    # Optional: Add a small delay if needed
    # sleep 1

  done
done

echo ""
echo -e "${GREEN}===================================================${NC}"
echo -e "${GREEN}All selected rosbag processing finished.${NC}"
echo -e "${GREEN}===================================================${NC}"