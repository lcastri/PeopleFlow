#!/bin/bash
# This script builds the Docker image using the docker-compose configuration.

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m'

echo " "
echo "Building the Docker environment using docker compose..."
echo "This will use 'Dockerfile' and 'docker-compose.yml' in the current directory."
docker compose build

echo " "
echo "Removing old images..."
docker image prune -f

echo " "
echo -e "${GREEN}Docker environment built successfully!${NC}"
echo -e "${YELLOW}You can now run './docrun.sh' to start the container.${NC}"