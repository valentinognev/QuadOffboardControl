CONTAINER_NAME="px4-noetic-sim-ros"

# Get script directory to make paths invariant to project location
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

xhost +
xhost +local:docker
# Remove existing container if it exists
docker rm -f ${CONTAINER_NAME} 2>/dev/null || true
docker run -it --net=host \
               --cap-drop=all \
               --privileged \
               --env="DISPLAY=$DISPLAY" \
               --env="QT_X11_NO_MITSHM=1" \
               --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
               --volume="${SCRIPT_DIR}/positions.txt:/home/valentin/PX4-Autopilot/Tools/simulation/positions.txt:rw" \
               --volume="${SCRIPT_DIR}/sitl_multiple_run.sh:/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run2.sh:rw" \
               --volume="${SCRIPT_DIR}/irisModel/iris.sdf.jinja:/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja:rw" \
               --name=${CONTAINER_NAME} \
               ${CONTAINER_NAME}
            #    --env="XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" \
            #    --env="GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH" \
xhost -local:docker
xhost -


# docker run -it --rm \
# 	-e DISPLAY=$DISPLAY \
# 	-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
# 	-e GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH \
# 	-e QT_X11_NO_MITSHM=1 \
# 	--network host \
# 	--privileged \
# 	--name ${PX4_SITL_DOCKER_NAME} \
# 	-v $XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR \
# 	${PX4_SITL_DOCKER_VER} /bin/bash -c "$1 $2 $3"
