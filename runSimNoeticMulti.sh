#!/bin/bash
# Script to start PX4 multi-drone simulation in Docker with a single command
# This script starts the Docker container and automatically runs the simulation
# based on positions defined in positions.txt

# Get script directory to make paths invariant to project location
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CONTAINER_NAME="px4-noetic-sim-ros"

# Cleanup function to be called on script exit or --kill
cleanup_on_exit() {
    echo ""
    echo "Cleaning up..."
    
    # Remove container
    echo "Removing container..."
    docker rm -f ${CONTAINER_NAME} 2>/dev/null || true
    
    # Clean up X11
    echo "Cleaning up X11 resources..."
    xhost -local:docker 2>/dev/null || true
    xhost - 2>/dev/null || true
    rm -f /tmp/.X*-lock 2>/dev/null || true
    
    # Clean up processes
    echo "Killing related processes..."
    pkill -x px4 2>/dev/null || true
    pkill gzclient 2>/dev/null || true
    pkill gzserver 2>/dev/null || true
    pkill -f "gz master" 2>/dev/null || true
    pkill -f "gazebo.*master" 2>/dev/null || true
    
    # Kill process on port 11345 if any
    if lsof -ti:11345 >/dev/null 2>&1; then
        echo "Killing process using Gazebo master port 11345..."
        lsof -ti:11345 | xargs kill -9 2>/dev/null || true
    fi
}

# Default values
NUM_DRONES=1
POSITIONS_FILE="${SCRIPT_DIR}/multidrone/positions.txt"

# Argument parsing
while [[ $# -gt 0 ]]; do
    case "$1" in
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Script to start PX4 multi-drone simulation in Docker."
            echo ""
            echo "Options:"
            echo "  --help, -h               Show this help message and exit"
            echo "  --kill                   Clean up/kill existing simulation containers and processes"
            echo "  --num=N, --num N         Number of drones to spawn (default: 1)"
            echo "  --file=PATH, --file PATH Path to positions file (default: multidrone/positions.txt)"
            echo ""
            exit 0
            ;;
        --kill)
            echo "Kill command received."
            cleanup_on_exit
            exit 0
            ;;
        --num=*)
            NUM_DRONES="${1#*=}"
            shift
            ;;
        --num)
            NUM_DRONES="$2"
            shift 2
            ;;
        --file=*)
            POSITIONS_FILE="${1#*=}"
            shift
            ;;
        --file)
            POSITIONS_FILE="$2"
            shift 2
            ;;
        [0-9]*)
            NUM_DRONES="$1"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information."
            exit 1
            ;;
    esac
done

# Set trap to cleanup on script exit
trap cleanup_on_exit EXIT INT TERM

# Initial cleanup to ensure clean state
cleanup_on_exit

# Setup X11 forwarding
xhost + 2>/dev/null || true
xhost +local:docker 2>/dev/null || true

# Verify display is accessible
if [ -z "$DISPLAY" ]; then
    echo "Warning: DISPLAY environment variable is not set"
    export DISPLAY=:0
fi

# Set XAUTHORITY if not set
if [ -z "$XAUTHORITY" ] && [ -f "$HOME/.Xauthority" ]; then
    export XAUTHORITY="$HOME/.Xauthority"
fi

# Build docker run command with conditional XAUTHORITY volume
XAUTH_FILE="${XAUTHORITY:-$HOME/.Xauthority}"

# Determine positions file mount
DEFAULT_POSITIONS="${SCRIPT_DIR}/multidrone/positions.txt"
if [ "$POSITIONS_FILE" = "$DEFAULT_POSITIONS" ]; then
    # Use default positions file location
    CONTAINER_POSITIONS_PATH="/home/valentin/PX4-Autopilot/Tools/simulation/positions.txt"
    DOCKER_VOLUMES=(
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
        --volume="${POSITIONS_FILE}:${CONTAINER_POSITIONS_PATH}:rw"
        --volume="${SCRIPT_DIR}/multidrone/sitl_multiple_run.sh:/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run2.sh:rw"
        --volume="${SCRIPT_DIR}/multidrone/inject_iris_sensors.py:/home/valentin/PX4-Autopilot/Tools/simulation/inject_iris_sensors.py:ro"
        --volume="${SCRIPT_DIR}/multidrone/inject_iris_colors.py:/home/valentin/PX4-Autopilot/Tools/simulation/inject_iris_colors.py:ro"
        --volume="${SCRIPT_DIR}/multidrone/airframes/10015_gazebo-classic_iris.post:/home/valentin/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/10015_gazebo-classic_iris.post:ro"
    )
else
    # Use custom positions file
    CONTAINER_POSITIONS_PATH="/tmp/custom_positions.txt"
    DOCKER_VOLUMES=(
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
        --volume="${SCRIPT_DIR}/multidrone/positions.txt:/home/valentin/PX4-Autopilot/Tools/simulation/positions.txt:rw"
        --volume="${POSITIONS_FILE}:${CONTAINER_POSITIONS_PATH}:ro"
        --volume="${SCRIPT_DIR}/multidrone/sitl_multiple_run.sh:/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run2.sh:rw"
        --volume="${SCRIPT_DIR}/multidrone/inject_iris_sensors.py:/home/valentin/PX4-Autopilot/Tools/simulation/inject_iris_sensors.py:ro"
        --volume="${SCRIPT_DIR}/multidrone/inject_iris_colors.py:/home/valentin/PX4-Autopilot/Tools/simulation/inject_iris_colors.py:ro"
        --volume="${SCRIPT_DIR}/multidrone/airframes/10015_gazebo-classic_iris.post:/home/valentin/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/10015_gazebo-classic_iris.post:ro"
    )
fi

# Add XAUTHORITY volume only if file exists
if [ -f "$XAUTH_FILE" ]; then
    DOCKER_VOLUMES+=(--volume="${XAUTH_FILE}:${XAUTH_FILE}:ro")
fi

# Run docker container with the simulation command
# Pin Gazebo transport to loopback. Without this, gzserver floods
# "Exception sending a multicast message: Network is unreachable" on hosts
# with no working multicast route; after a while simulator_mavlink poll
# timeouts freeze HIL (DISTANCE_SENSOR / OF / IMU stop → FLIG bottom_clearance=-1).
docker run -it --net=host \
           --cap-drop=all \
           --privileged \
           --env="DISPLAY=$DISPLAY" \
           --env="QT_X11_NO_MITSHM=1" \
           --env="CATSWARM_OF_MODE=${CATSWARM_OF_MODE:-mockup}" \
           --env="GAZEBO_IP=127.0.0.1" \
           --env="GAZEBO_MASTER_URI=http://127.0.0.1:11345" \
           --env="XAUTHORITY=${XAUTH_FILE}" \
           "${DOCKER_VOLUMES[@]}" \
           --name=${CONTAINER_NAME} \
           ${CONTAINER_NAME} \
           /bin/bash -c "./Tools/simulation/gazebo-classic/sitl_multiple_run2.sh -p ${CONTAINER_POSITIONS_PATH} -n ${NUM_DRONES}"

# Note: Cleanup is handled by the trap function on exit
