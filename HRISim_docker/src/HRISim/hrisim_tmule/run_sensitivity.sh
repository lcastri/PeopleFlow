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
K_D_LIST=(0.1 1 10)
K_PD_LIST=(1 10 100)
K_BC_LIST=(0.5 5 50)

# --- Script Main Logic ---
echo -e "${GREEN}===================================================${NC}"
echo -e "${GREEN}Starting simulation batch run...${NC}"
echo -e "${GREEN}Total runs to perform: $((${#TIMES_TO_TEST[@]} * ${#K_D_LIST[@]} * ${#K_PD_LIST[@]} * ${#K_BC_LIST[@]}))${NC}"
echo -e "${GREEN}===================================================${NC}"

current_run=1

# Nested loops for all parameters
for t in "${TIMES_TO_TEST[@]}"; do
  for kd in "${K_D_LIST[@]}"; do
    for kp in "${K_PD_LIST[@]}"; do
      for kb in "${K_BC_LIST[@]}"; do

        # --- SKIP LOGIC ---
        # Check if the current run number should be skipped
        case "$current_run" in (1|2|3|4|6|7)
            echo ""
            echo -e "${YELLOW}===================================================${NC}"
            echo -e "${YELLOW}SKIPPING Run $current_run.${NC}"
            echo -e "${YELLOW}===================================================${NC}"
            ((current_run++)) # Increment the counter
            continue          # Skip to the next 'kb' iteration
            ;;
        esac
        # --- END SKIP LOGIC ---
        
        echo ""
        echo -e "${CYAN}===================================================${NC}"
        echo -e "${CYAN}Starting Run $current_run: INIT_TIME=$t, K_D=$kd, K_PD=$kp, K_BC=$kb${NC}"
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
  done
done

echo ""
echo -e "${GREEN}===================================================${NC}"
echo -e "${GREEN}All simulations finished.${NC}"
echo -e "${GREEN}================================G===================${NC}"