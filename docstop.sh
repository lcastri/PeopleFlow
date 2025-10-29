#!/bin/bash
# This script stops and removes the container defined in docker-compose.yml.

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m'

echo " "
echo "Stopping and removing the container..."
docker compose down

echo " "
echo -e "${GREEN}Environment has been stopped.${NC}"
