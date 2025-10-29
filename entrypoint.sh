#!/bin/bash

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

set -e

echo " "
echo -e "${GREEN}###${NC}"
echo -e "${GREEN}### Welcome to the HRISim dev container!${NC}"
echo -e "${GREEN}###${NC}"
echo " "

if ! grep -q "### Sourced by HRISim Entrypoint ###" "$HOME/.bashrc"; then
  echo "" >> ~/.bashrc
  echo "### Sourced by HRISim Entrypoint ###" >> ~/.bashrc
  echo "# Define colors for shell messages" >> ~/.bashrc
  echo "GREEN='\\033[0;32m'" >> ~/.bashrc
  echo "NC='\\033[0m'" >> ~/.bashrc
  echo "" >> ~/.bashrc

  echo "export PNP_HOME=/home/hrisim/ros_ws/src/pnp_ros/" >> ~/.bashrc
  echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
  echo "export GAZEBO_MODEL_PATH=/home/hrisim/.gazebo/models:$GAZEBO_MODEL_PATH" >> ~/.bashrc
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

  echo "source /home/hrisim/tiago_ws/devel/setup.bash" >> ~/.bashrc
  echo "echo -e \"\${GREEN}Sourced TIAGo ws.\${NC}\"" >> ~/.bashrc
  echo "" >> ~/.bashrc

  echo "# Source user workspace only if it has been built" >> ~/.bashrc
  echo "if [ -f \"/home/hrisim/ros_ws/devel/setup.bash\" ]; then" >> ~/.bashrc
  echo "  source \"/home/hrisim/ros_ws/devel/setup.bash\"" >> ~/.bashrc
  echo "  echo -e \"\${GREEN}Sourced ros_ws ws.\${NC}\"" >> ~/.bashrc
  echo "fi" >> ~/.bashrc
  echo "" >> ~/.bashrc
  
  echo "### Custom Aliases Added by Entrypoint ###" >> ~/.bashrc
  echo "function tstart(){  tmule -c /home/hrisim/ros_ws/src/HRISim/hrisim_tmule/tmule/hrisim_bringup.yaml -W 1 launch ; }" >> ~/.bashrc
  echo "function tstop(){  tmule -c /home/hrisim/ros_ws/src/HRISim/hrisim_tmule/tmule/hrisim_bringup.yaml terminate ; }" >> ~/.bashrc
  echo "function trestart(){  tmule -c /home/hrisim/ros_ws/src/HRISim/hrisim_tmule/tmule/hrisim_bringup.yaml -W 1 relaunch ; }" >> ~/.bashrc
  echo "function tshow(){  tmux a -t HRISim_bringup ; }" >> ~/.bashrc
  echo "" >> ~/.bashrc
  
  echo "export ROS_MASTER_URI=http://127.0.0.1:11311" >> ~/.bashrc
  echo "export ROS_HOSTNAME=127.0.0.1" >> ~/.bashrc
  echo "export ROS_IP=127.0.0.1" >> ~/.bashrc
  echo "" >> ~/.bashrc
  echo "Added custom environment setup to .bashrc"
fi

echo " "
echo -e "${GREEN}Container is ready. Your workspaces are sourced.${NC}"
echo -e "Remember to run 'catkin build' in /home/hrisim/ros_ws after making code changes."
echo " "

exec "$@"

