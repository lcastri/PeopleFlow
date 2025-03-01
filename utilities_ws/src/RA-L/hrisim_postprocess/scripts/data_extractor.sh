#!/bin/bash

bagname="noncausal-28022025"

for i in {1..11}
do
    if [ $i -eq 1 ]; then
        load_goal=false
    else
        load_goal=true
    fi

    if [ $i -eq 11 ]; then
        time_of_the_day="off"
    else
        time_of_the_day="H$i"
    fi
    if [ $i -eq 1 ]; then
        start_time=0
    else
        # start_time=$((300 * (i-1) - 20))
        start_time=$((3600 * (i-1) - 20))
    fi

    roslaunch hrisim_postprocess HH_bringup.launch bagname:=$bagname time_of_the_day:=$time_of_the_day start_time:=$start_time load_goal:=$load_goal
done