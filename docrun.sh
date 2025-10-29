#!/bin/bash
# This script starts the container in detached (background) mode.

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m'

echo " "
echo "Checking host graphics configuration..."

# Get the renderer string just one time
renderer_string=$(glxinfo -B | grep "OpenGL renderer string")

# Check for known-bad renderers (CPU software or integrated Intel)
if echo "$renderer_string" | grep -q -E "llvmpipe|Mesa Intel"; then
    echo -e "${RED}ERROR: Incompatible renderer detected!${NC}"
    echo -e "${YELLOW}Your host is using software rendering or integrated Intel graphics:${NC}"
    echo -e "${YELLOW}${renderer_string}${NC}"
    echo -e "${YELLOW}Gazebo's 'gpu_ray' sensor will fail. Please switch to your GPU.${NC}"   
    exit 1
fi
echo -e "${GREEN}GPU renderer detected. Proceeding to launch Docker...${NC}"

echo " "
echo "Starting the container in the background..."
docker compose up -d

echo " "
echo -e "${GREEN}Container is running.${NC}"
echo -e "${YELLOW}You can now run './docshell.sh' to open a shell inside the container.${NC}"
echo -e "${YELLOW}You can now run './docstop.sh' to stop the container.${NC}"