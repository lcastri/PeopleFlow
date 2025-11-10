#!/bin/bash

# --- Color Definitions ---
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# --- Parameter Sweep Definitions ---
# These MUST match the lists used to generate the rosbags
TIMES_TO_TEST=("H2" "H6")
K_D_LIST=(01 1 10)
K_PD_LIST=(1 10 100)
K_BC_LIST=(05 5 50)

# --- Script Main Logic ---
echo -e "${GREEN}===================================================${NC}"
echo -e "${GREEN}Starting rosbag post-processing batch run...${NC}"
total_runs=$((${#TIMES_TO_TEST[@]} * ${#K_D_LIST[@]} * ${#K_PD_LIST[@]} * ${#K_BC_LIST[@]}))
echo -e "${GREEN}Total bags to process: $total_runs${NC}"
echo -e "${GREEN}===================================================${NC}"

current_run=1

# Nested loops for all parameters
for t in "${TIMES_TO_TEST[@]}"; do
  for kd in "${K_D_LIST[@]}"; do
    for kp in "${K_PD_LIST[@]}"; do
      for kb in "${K_BC_LIST[@]}"; do
        
        # Construct the bag name based on the new format
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
        
      done
    done
  done
done

echo ""
echo -e "${GREEN}===================================================${NC}"
echo -e "${GREEN}All rosbag processing finished.${NC}"
echo -e "${GREEN}===================================================${NC}"