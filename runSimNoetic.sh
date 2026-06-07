#!/bin/bash
export PX4_SITL_DOCKER_NAME=px4-noetic-sim-ros
export PX4_SITL_DOCKER_VER=$PX4_SITL_DOCKER_NAME:latest
# kill all containers
# docker kill $PX4_SITL_DOCKER_NAME

./run_px4_sitl_docker.sh 'rm -f /home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf && git config --global --add safe.directory /home/valentin/PX4-Autopilot && git config --global --add safe.directory /home/valentin/PX4-Autopilot/src/modules/mavlink/mavlink && make px4_sitl gazebo-classic'
