#!/bin/bash
# Script to start PX4 multi-drone simulation in Docker with a single command
# This script starts the Docker container and automatically runs the simulation
# based on positions defined in positions.txt

# Get script directory to make paths invariant to project location
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export PX4_SITL_DOCKER_NAME=px4-noetic-sim-ros
export PX4_SITL_DOCKER_VER=$PX4_SITL_DOCKER_NAME:latest

# Same defaults as runSimNoetic.sh (only applied if the file exists).
DEFAULT_RCS_FILE="$SCRIPT_DIR/irisModel/rcS.cat2"
DEFAULT_JINJA_FILE="$SCRIPT_DIR/irisModel/iris.sdf.jinja.cat2"
PX4_IN_CONTAINER="${PX4_IN_CONTAINER:-/home/valentin/PX4-Autopilot}"

# Cleanup function to be called on script exit or --kill
cleanup_on_exit() {
    echo ""
    echo "Cleaning up..."
    
    # Remove container
    echo "Removing container..."
    docker rm -f ${PX4_SITL_DOCKER_NAME} 2>/dev/null || true
    
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
            echo "  --rcS=PATH               Host path to rcS; bind-mounted into the PX4 container when set."
            echo "  --jinja=PATH             Host path to iris.sdf.jinja; bind-mounted when set."
            echo "                           If either is omitted and the corresponding default file exists under"
            echo "                           this repo (irisModel/rcS.cat2, irisModel/iris.sdf.jinja.cat2), that"
            echo "                           file is used; otherwise the image default is used for that role."
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
        --rcS=*)
            export PX4_SITL_RCS_FILE="${1#*=}"
            shift
            ;;
        --jinja=*)
            export PX4_SITL_JINJA_FILE="${1#*=}"
            shift
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

# Apply default rcS / jinja only when not specified and the default file is present.
[[ -z "${PX4_SITL_RCS_FILE:-}" ]] && [[ -f "$DEFAULT_RCS_FILE" ]] && export PX4_SITL_RCS_FILE="$DEFAULT_RCS_FILE"
[[ -z "${PX4_SITL_JINJA_FILE:-}" ]] && [[ -f "$DEFAULT_JINJA_FILE" ]] && export PX4_SITL_JINJA_FILE="$DEFAULT_JINJA_FILE"

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
    )
else
    # Use custom positions file
    CONTAINER_POSITIONS_PATH="/tmp/custom_positions.txt"
    DOCKER_VOLUMES=(
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
        --volume="${SCRIPT_DIR}/multidrone/positions.txt:/home/valentin/PX4-Autopilot/Tools/simulation/positions.txt:rw"
        --volume="${POSITIONS_FILE}:${CONTAINER_POSITIONS_PATH}:ro"
        --volume="${SCRIPT_DIR}/multidrone/sitl_multiple_run.sh:/home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run2.sh:rw"
    )
fi

if [[ -n "${PX4_SITL_RCS_FILE:-}" ]]; then
    DOCKER_VOLUMES+=(--volume="${PX4_SITL_RCS_FILE}:${PX4_IN_CONTAINER}/ROMFS/px4fmu_common/init.d-posix/rcS:rw")
fi
if [[ -n "${PX4_SITL_JINJA_FILE:-}" ]]; then
    DOCKER_VOLUMES+=(--volume="${PX4_SITL_JINJA_FILE}:${PX4_IN_CONTAINER}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf.jinja:rw")
fi

# Add XAUTHORITY volume only if file exists
if [ -f "$XAUTH_FILE" ]; then
    DOCKER_VOLUMES+=(--volume="${XAUTH_FILE}:${XAUTH_FILE}:ro")
fi

# Run docker container with the simulation command
docker run -it --net=host \
           --cap-drop=all \
           --privileged \
           --env="DISPLAY=$DISPLAY" \
           --env="QT_X11_NO_MITSHM=1" \
           --env="XAUTHORITY=${XAUTH_FILE}" \
           "${DOCKER_VOLUMES[@]}" \
           --name=${PX4_SITL_DOCKER_NAME} \
           ${PX4_SITL_DOCKER_VER} \
           /bin/bash -c "./Tools/simulation/gazebo-classic/sitl_multiple_run2.sh -p ${CONTAINER_POSITIONS_PATH} -n ${NUM_DRONES}"

# Note: Cleanup is handled by the trap function on exit
