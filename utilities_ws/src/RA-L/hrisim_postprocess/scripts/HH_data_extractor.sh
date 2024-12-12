#!/bin/bash

# Set the base parameters
BAGNAME="BL25_28102024"
INTERVAL=3600  # Duration of each segment in seconds
END_TIME=42619  # Total time in seconds (adjust this to your actual end time)

# Initialize start time
START_TIME=0

# Loop through time intervals
while [ $START_TIME -lt $END_TIME ]; do
    # Calculate the time of day label
    HOUR=$((START_TIME / 3600))
    TIME_OF_DAY="hour_$HOUR"
    NEXT_SAVE_TIME=$((START_TIME + INTERVAL))
    
    echo "Launching ROS with time_of_the_day=$TIME_OF_DAY and start_time=$START_TIME"
    
    # Run the ROS launch command
    roslaunch hrisim_postprocess bringup.launch bagname:=$BAGNAME next_save_time:=$NEXT_SAVE_TIME start_time:=$((START_TIME-20)) load_goal:=true

    # Move to the next interval
    START_TIME=$NEXT_SAVE_TIME

    # Optional: Wait a bit before starting the next launch
    sleep 5
done
