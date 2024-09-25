#!/bin/bash

# roslaunch hrisim_postprocess bringup.launch bagname:=24092024 time_of_the_day:=starting start_time:=0
# roslaunch hrisim_postprocess bringup.launch bagname:=24092024 time_of_the_day:=morning start_time:=4915.965
roslaunch hrisim_postprocess bringup.launch bagname:=24092024 time_of_the_day:=lunch start_time:=17980
roslaunch hrisim_postprocess bringup.launch bagname:=24092024 time_of_the_day:=afternoon start_time:=21580
roslaunch hrisim_postprocess bringup.launch bagname:=24092024 time_of_the_day:=quitting start_time:=32380
roslaunch hrisim_postprocess bringup.launch bagname:=24092024 time_of_the_day:=off start_time:=35980