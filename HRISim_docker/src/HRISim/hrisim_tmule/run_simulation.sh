#!/bin/bash

# This script runs the HRISim_bringup session for INIT_TIME=H1 through H10

echo "Starting simulation batch run..."
echo "================================="

# Loop from 1 to 10
for i in {1..10}
do
  # Set the environment variable for this run
  # This will override the "H1" default in your tmule file
  export INIT_TIME="H$i"

  echo ""
  echo "--- Launching simulation for INIT_TIME=$INIT_TIME ---"
  
  # Run the tstart command and wait for it to finish.
  tmule -c /home/hrisim/ros_ws/src/HRISim/hrisim_tmule/tmule/hrisim_bringup.yaml -W 1 launch
  echo "Waiting for session 'HRISim_bringup' to complete..."
  while tmux has-session -t HRISim_bringup 2>/dev/null; do
    sleep 2
  done

  echo "--- Simulation for $INIT_TIME complete. ---"
  
  # Add a small delay to ensure all ROS nodes have shut down
  # before starting the next one.
  echo "Waiting 5 seconds before next run..."
  sleep 5

done

echo ""
echo "================================="
echo "All simulations finished."