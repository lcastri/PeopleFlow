#!/bin/bash

bagname="causal"

for i in {11..11}
do
    if [ $i -eq 11 ]; then
        time_of_the_day="off"
    else
        time_of_the_day="H$i"
    fi
    launch_bagname="$bagname-$time_of_the_day"
    roslaunch hrisim_postprocess data_extractor_bringup.launch bagname:=$launch_bagname
done
