#!/bin/bash
export PX4_SITL_DOCKER_NAME=px4-noetic-sim-ros
export PX4_SITL_DOCKER_VER=$PX4_SITL_DOCKER_NAME:latest

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Defaults when not overridden on the command line; only applied if the file exists.
DEFAULT_RCS_FILE="$SCRIPT_DIR/irisModel/rcS.cat2"
DEFAULT_JINJA_FILE="$SCRIPT_DIR/irisModel/iris.sdf.jinja.cat2"

usage() {
	cat <<EOF
Usage: $0 [--rcSfile=PATH] [--jinja=PATH]

  --rcSfile=PATH   Host path to rcS; bind-mounted into the PX4 container when set.
  --jinja=PATH     Host path to iris.sdf.jinja; bind-mounted when set.
                   If either is omitted and the corresponding default file exists under
                   this repo (irisModel/rcS.cat2, irisModel/iris.sdf.jinja.cat2), that
                   file is used; otherwise the image default is used for that role.

Optional PX4 SITL is started via run_px4_sitl_docker.sh (make px4_sitl gazebo-classic).
EOF
}

while [[ $# -gt 0 ]]; do
	case $1 in
		-h|--help)
			usage
			exit 0
			;;
		--rcSfile=*)
			export PX4_SITL_RCS_FILE="${1#*=}"
			shift
			;;
		--jinja=*)
			export PX4_SITL_JINJA_FILE="${1#*=}"
			shift
			;;
		*)
			echo "Unknown option: $1" >&2
			echo "Try '$0 --help' for usage." >&2
			exit 1
			;;
	esac
done

# Apply default rcS / jinja only when not specified and the default file is present.
[[ -z "${PX4_SITL_RCS_FILE:-}" ]] && [[ -f "$DEFAULT_RCS_FILE" ]] && export PX4_SITL_RCS_FILE="$DEFAULT_RCS_FILE"
[[ -z "${PX4_SITL_JINJA_FILE:-}" ]] && [[ -f "$DEFAULT_JINJA_FILE" ]] && export PX4_SITL_JINJA_FILE="$DEFAULT_JINJA_FILE"

# Remove existing container if it exists (whether running or stopped)
docker rm -f $PX4_SITL_DOCKER_NAME 2>/dev/null || true

./run_px4_sitl_docker.sh 'make px4_sitl gazebo-classic'