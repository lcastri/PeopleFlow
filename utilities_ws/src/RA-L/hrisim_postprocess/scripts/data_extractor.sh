#!/bin/bash

bagname="base"

for i in {5..5}
do
    if [ $i -eq 11 ]; then
        time_of_the_day="off"
    else
        time_of_the_day="H$i"
    fi
    launch_bagname="$bagname-$time_of_the_day"
    roslaunch hrisim_postprocess HH_bringup.launch bagname:=$launch_bagname
done