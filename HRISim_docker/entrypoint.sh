#!/bin/bash

set -e

# setup environment
source "$HOME/.bashrc"

echo " "
echo "###"
echo "### This is the HRISim container!"
echo "###"
echo " "

{

  echo "Container is now running."
  echo " "

  echo "function tstart(){  tmule -c ~/ros_ws/src/HRISim/hrisim_tmule/tmule/hrisim_bringup.yaml -W 3 launch ; }" >> ~/.bashrc
  echo "function tstop(){  tmule -c ~/ros_ws/src/HRISim/hrisim_tmule/tmule/hrisim_bringup.yaml terminate ; }" >> ~/.bashrc
  echo "function trestart(){  tmule -c ~/ros_ws/src/HRISim/hrisim_tmule/tmule/hrisim_bringup.yaml -W 3 relaunch ; }" >> ~/.bashrc
  echo "function tshow(){  tmux a -t HRISim_bringup ; }" >> ~/.bashrc

  source /opt/ros/noetic/setup.bash
  source ~/tiago_ws/devel/setup.bash
  cd ~/ros_ws
  catkin build
  source ~/ros_ws/devel/setup.bash

  echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
  
  echo "export ROS_MASTER_URI=http://127.0.0.1:11311" >> ~/.bashrc
  echo "export ROS_HOSTNAME=127.0.0.1" >> ~/.bashrc
  echo "export ROS_IP=127.0.0.1" >> ~/.bashrc 

  source ~/.bashrc
  exec "/bin/bash"

} || {

  echo "Container failed."
  exec "$@"

}
