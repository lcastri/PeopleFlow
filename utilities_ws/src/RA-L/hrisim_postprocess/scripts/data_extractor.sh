#!/bin/bash

bagname="noncausal-28022025"

for i in {1..11}
do
    if [ $i -eq 11 ]; then
        time_of_the_day="off"
    else
        time_of_the_day="H$i"
    fi
    launch_bagname="$bagname-$time_of_the_day"
    roslaunch hrisim_postprocess HH_bringup.launch bagname:=$launch_bagname
done