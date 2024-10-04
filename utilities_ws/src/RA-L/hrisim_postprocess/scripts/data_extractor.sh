#!/bin/bash

# roslaunch hrisim_postprocess bringup.launch bagname:=BL100 time_of_the_day:=starting start_time:=0 load_goal:=false
# roslaunch hrisim_postprocess bringup.launch bagname:=BL100 time_of_the_day:=morning start_time:=3580 load_goal:=true
# roslaunch hrisim_postprocess bringup.launch bagname:=BL100 time_of_the_day:=lunch start_time:=17980 load_goal:=true
# roslaunch hrisim_postprocess bringup.launch bagname:=BL100 time_of_the_day:=afternoon start_time:=21580 load_goal:=true
# roslaunch hrisim_postprocess bringup.launch bagname:=BL100 time_of_the_day:=quitting start_time:=32380 load_goal:=true
roslaunch hrisim_postprocess bringup.launch bagname:=BL100 time_of_the_day:=off start_time:=35980 load_goal:=true