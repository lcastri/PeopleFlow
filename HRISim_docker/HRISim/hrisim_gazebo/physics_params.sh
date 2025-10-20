#!/bin/bash

# Store the provided update rate in a variable
MAX_STEP_SIZE=$1
REAL_TIME_UPDATE_RATE=$2

# Wait until Gazebo is running
while ! pgrep -x "gzserver" > /dev/null; do
  echo "Waiting for Gazebo to start..."
  sleep 5
done

# Change physics parameters
gz physics -s "$MAX_STEP_SIZE" -u "$REAL_TIME_UPDATE_RATE"