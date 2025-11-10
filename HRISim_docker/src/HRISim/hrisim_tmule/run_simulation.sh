#!/bin/bash

GREEN='\033[0;32m'
YELLOW='\033[0;33m'
NC='\033[0m'

# This script runs the HRISim_bringup session for TS=H1 through H10
echo -e "${GREEN}===================================================${NC}"
echo -e "${GREEN}Starting simulation batch run...${NC}"
echo -e "${GREEN}===================================================${NC}"

# Loop from 1 to 10
for i in {9..9}
do
  export INIT_TIME="H$i"

  echo ""
  echo -e "${GREEN}===================================================${NC}"
  echo -e "${GREEN}Starting simulation for TS = $INIT_TIME${NC}"
  echo -e "${GREEN}===================================================${NC}"

  # Run the tstart command and wait for it to finish.
  tmule -c /home/hrisim/ros_ws/src/HRISim/hrisim_tmule/tmule/hrisim_bringup.yaml -W 1 launch
  echo -e "${YELLOW}===================================================${NC}"
  echo -e "${YELLOW}Waiting for session 'HRISim_bringup' to complete...${NC}"
  echo -e "${YELLOW}===================================================${NC}"
  while tmux has-session -t HRISim_bringup 2>/dev/null; do
    sleep 2
  done

  echo ""
  echo -e "${GREEN}===================================================${NC}"
  echo -e "${GREEN}Simulation for TS = $INIT_TIME complete${NC}"
  echo -e "${GREEN}===================================================${NC}"
  
  # Add a small delay to ensure all ROS nodes have shut down before starting the next one.
  echo ""
  echo -e "${YELLOW}===================================================${NC}"
  echo -e "${YELLOW}Waiting 5 seconds before next run...${NC}"
  echo -e "${YELLOW}===================================================${NC}"
  sleep 5

done

echo ""
echo -e "${GREEN}===================================================${NC}"
echo -e "${GREEN}All simulations finished.${NC}"
echo -e "${GREEN}===================================================${NC}"