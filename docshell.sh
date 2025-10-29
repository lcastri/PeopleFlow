#!/bin/bash
# A simple script to get a bash shell inside the running container.

echo "Opening a bash shell inside the HRISim container"
docker exec -it HRISim bash
