#!/bin/bash

# --- Color Definitions ---
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# --- Configuration ---
TMULE_CONFIG_FILE="/home/hrisim/ros_ws/src/HRISim/hrisim_tmule/tmule/hrisim_bringup.yaml"
SESSION_NAME="HRISim_bringup"

# --- Parameter Sweep Definitions ---
TIMES_TO_TEST=("H2" "H6")

# Define *only* the specific combinations you want to re-run
KD_RUNS=(1 1)
KP_RUNS=(10 100)
KB_RUNS=(5 50)

# --- Script Main Logic ---
num_combos=${#KD_RUNS[@]}
total_runs=$((${#TIMES_TO_TEST[@]} * num_combos))

echo -e "${GREEN}===================================================${NC}"
echo -e "${GREEN}Starting simulation batch re-run...${NC}"
echo -e "${GREEN}Total runs to perform: $total_runs${NC}"
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
    
    echo ""
    echo -e "${CYAN}===================================================${NC}"
    echo -e "${CYAN}Starting Run $current_run / $total_runs: INIT_TIME=$t, K_D=$kd, K_PD=$kp, K_BC=$kb${NC}"
    echo -e "${CYAN}===================================================${NC}"

    # EXPORT the variables so tmule can read them
    export INIT_TIME="$t"
    export K_D="$kd"
    export K_PD="$kp"
    export K_BC="$kb"
    # Export other defaults from your tmule file just in case
    export INIT_BATTERY="100"
    export ABORT_TIME_THRESHOLD="45"

    # Run the tmule launch command and wait for it to finish.
    # This is your blocking command.
    tmule -c "$TMULE_CONFIG_FILE" -W 1 launch

    echo -e "${YELLOW}===================================================${NC}"
    echo -e "${YELLOW}Waiting for session '$SESSION_NAME' to complete...${NC}"
    echo -e "${YELLOW}===================================================${NC}"
    
    # This loop waits for the session to fully terminate
    while tmux has-session -t "$SESSION_NAME" 2>/dev/null; do
      sleep 2
    done

    echo ""
    echo -e "${GREEN}===================================================${NC}"
    echo -e "${GREEN}Run $current_run complete${NC}"
    echo -e "${GREEN}===================================================${NC}"
    
    ((current_run++))
    
    # Add a delay to ensure all ROS nodes have shut down
    echo ""
    echo -e "${YELLOW}===================================================${NC}"
    echo -e "${YELLOW}Waiting 5 seconds before next run...${NC}"
    echo -e "${YELLOW}===================================================${NC}"
    sleep 5

  done
done

echo ""
echo -e "${GREEN}===================================================${NC}"
echo -e "${GREEN}All re-run simulations finished.${NC}"
echo -e "${GREEN}===================================================${NC}"