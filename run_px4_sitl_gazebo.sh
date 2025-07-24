#!/bin/bash
export PX4_SITL_DOCKER_NAME=px4_sitl
export PX4_SITL_DOCKER_VER=$PX4_SITL_DOCKER_NAME:v0.1
# kill all containers
docker kill $PX4_SITL_DOCKER_NAME

./run_px4_sitl_docker.sh 'make px4_sitl gazebo-classic'