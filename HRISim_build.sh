#!/bin/bash

container_name=HRISim
image_name=hrisim-docker
host_folder=$(pwd)/shared
container_folder=/root/shared

echo " "
echo "Starting docker engine..."
source ./start_docker.sh

echo " "
echo "Building ${container_name} docker..."
docker build -f HRISim_docker/Dockerfile -t ${image_name} .

echo " "
echo "Removing old images..."
dangling_images=$(docker images -qa -f 'dangling=true')
if [ -n "$dangling_images" ]; then
    docker rmi $dangling_images
fi

# echo " "
# echo "Pruning builder cache..."
# docker builder prune -a -f

echo " "
echo "${container_name} docker container built!"