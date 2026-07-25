#!/bin/bash
export PX4_SITL_DOCKER_NAME=px4-noble-sim-ros
export PX4_SITL_DOCKER_VER=$PX4_SITL_DOCKER_NAME:latest
# kill all containers
docker kill $PX4_SITL_DOCKER_NAME 2>/dev/null || true

./run_px4_sitl_docker.sh 'make px4_sitl gz_x500'
