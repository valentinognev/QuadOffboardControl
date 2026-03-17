#!/bin/bash

# Set default values if not already set
export PX4_SITL_DOCKER_NAME=${PX4_SITL_DOCKER_NAME:-px4_sitl}
export PX4_SITL_DOCKER_VER=${PX4_SITL_DOCKER_VER:-${PX4_SITL_DOCKER_NAME}:v0.1}

# enable access to xhost from the container
sudo xhost +

# Set up environment variables for Gazebo Sim
# XDG_RUNTIME_DIR is needed for Qt/GUI applications
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/tmp/runtime-root}
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# GZ_SIM_RESOURCE_PATH is needed for Gazebo Sim to find resources
# This should point to the Gazebo Sim resource directories
export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:-/usr/share/gz}

docker run -it --rm \
	-e DISPLAY=$DISPLAY \
	-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
	-e GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH \
	-e QT_X11_NO_MITSHM=1 \
	--network host \
	--privileged \
	--name ${PX4_SITL_DOCKER_NAME} \
	-v $XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR \
	-v $(pwd)/irisModel/iris.sdf.jinja:/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja:rw \
    -v $(pwd)/irisModel/10015_gazebo-classic_iris:/home/valentin/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10015_gazebo-classic_iris \
	-v $(pwd)/irisModel/rcS:/home/valentin/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS \
	${PX4_SITL_DOCKER_VER} /bin/bash -c "$1 $2 $3"
	# -v $(pwd)/10015_gazebo-classic_iris:/src/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10015_gazebo-classic_iris \
	# -v $(pwd)/iris.sdf.jinja:/src/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja \


# PX4_SITL_DOCKER_VER="px4_sitl:v0.1"
# # PX4_SITL_DOCKER_VER="px4-noetic-sim-ros:latest"

# docker run -it --rm \
# 	-e DISPLAY=$DISPLAY \
# 	--network host \
# 	--privileged \
# 	-v /home/$USER/Projects/GambitonBiut/docker_sim/PX4-Autopilot/:/home/$USER/PX4-Autopilot \
# 	-w /home/$USER/PX4-Autopilot \
# 	${PX4_SITL_DOCKER_VER} /bin/bash -c "$1 $2 $3"	
#make px4_sitl none_iris
