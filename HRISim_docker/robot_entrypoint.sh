#!/bin/bash

ROBOT_NUM=$1
ETH=$2

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

# === Check arguments early ===
if [ -z "$ROBOT_NUM" ]; then
  echo -e "${RED}ERROR: No argument supplied!${NC}"
  echo
  echo -e "${RED}Usage: ./HRISim_run.sh ROBOT_NUM ETH${NC}"
  echo -e "${RED}Example: ./HRISim_run.sh 125 true${NC}"
  exit 1
fi

echo
echo "###"
echo "### This is the HRISim container!"
echo "### Running TIAGo ${ROBOT_NUM}"
echo "###"
echo

# Add robot host (needs sudo)
if ! echo -e "10.68.0.1\ttiago-${ROBOT_NUM}c" | sudo tee -a /etc/hosts > /dev/null; then
  echo -e "${RED}[ERROR] Failed to add to /etc/hosts.${NC}"
  exit 1
fi

# Setup env
source "$HOME/.bashrc"
source /opt/ros/noetic/setup.bash
source ~/tiago_ws/devel/setup.bash
cd ~/ros_ws
catkin build
source ~/ros_ws/devel/setup.bash
echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc

# Add convenience functions (once only)
grep -q 'tmule.*tiago.yaml' ~/.bashrc || {
  echo "function tstart(){  tmule -c ~/ros_ws/src/HRISim/hrisim_tmule/tmule/tiago_bringup.yaml -W 3 launch ; }" >> ~/.bashrc
  echo "function tstop(){  tmule -c ~/ros_ws/src/HRISim/hrisim_tmule/tmule/tiago_bringup.yaml terminate ; }" >> ~/.bashrc
  echo "function trestart(){  tmule -c ~/ros_ws/src/HRISim/hrisim_tmule/tmule/tiago_bringup.yaml -W 3 relaunch ; }" >> ~/.bashrc
  echo "function tshow(){  tmux a -t TIAGo ; }" >> ~/.bashrc
  echo "function connect(){  source ~/ros_ws/src/HRISim/scripts/connect_tiago.sh ${ROBOT_NUM} ${ETH} ; }" >> ~/.bashrc
  echo "source ~/ros_ws/src/HRISim/scripts/connect_tiago.sh ${ROBOT_NUM} ${ETH}" >> ~/.bashrc
}

echo
echo -e "${GREEN}Setup complete. Ready to run.${NC}"
exec "/bin/bash"
