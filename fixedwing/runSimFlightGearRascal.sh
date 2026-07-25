#!/bin/bash
# Start PX4 SITL + FlightGear Rascal in the Noble sim container.
# Mounts fixedwing/fg_spawn.env so spawn altitude/speed can change without rebuilding the image.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CONTAINER_NAME="${PX4_SITL_DOCKER_NAME:-px4-noble-sim-ros}"
IMAGE_TAG="${PX4_SITL_DOCKER_VER:-${CONTAINER_NAME}:latest}"
SPAWN_ENV="${SCRIPT_DIR}/fg_spawn.env"
CONTAINER_SPAWN_ENV="/tmp/fg_spawn.env"

cleanup_on_exit() {
	echo ""
	echo "Cleaning up..."
	docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true
	xhost -local:docker 2>/dev/null || true
}

while [[ $# -gt 0 ]]; do
	case "$1" in
		--help|-h)
			echo "Usage: $0 [--kill]"
			echo "  Starts FlightGear Rascal SITL with spawn args from ${SPAWN_ENV}"
			echo "  --kill  Remove container and exit"
			exit 0
			;;
		--kill)
			cleanup_on_exit
			exit 0
			;;
		*)
			echo "Unknown option: $1 (use --help)"
			exit 1
			;;
	esac
done

if [[ ! -f "${SPAWN_ENV}" ]]; then
	echo "Missing spawn config: ${SPAWN_ENV}" >&2
	exit 1
fi

trap cleanup_on_exit EXIT INT TERM

docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true

xhost + 2>/dev/null || true
xhost +local:docker 2>/dev/null || true

if [[ -z "${DISPLAY:-}" ]]; then
	echo "Warning: DISPLAY is not set; defaulting to :0"
	export DISPLAY=:0
fi

if [[ -z "${XAUTHORITY:-}" && -f "${HOME}/.Xauthority" ]]; then
	export XAUTHORITY="${HOME}/.Xauthority"
fi

XAUTH_FILE="${XAUTHORITY:-$HOME/.Xauthority}"
XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"
mkdir -p "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}"

DOCKER_VOLUMES=(
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
	--volume="${XDG_RUNTIME_DIR}:${XDG_RUNTIME_DIR}"
	--volume="${SPAWN_ENV}:${CONTAINER_SPAWN_ENV}:ro"
)

if [[ -f "${XAUTH_FILE}" ]]; then
	DOCKER_VOLUMES+=(--volume="${XAUTH_FILE}:${XAUTH_FILE}:ro")
fi

# -t only when stdin is a TTY (so Python can start this script in the background).
DOCKER_IT=(-i)
if [[ -t 0 ]]; then
	DOCKER_IT=(-it)
fi

echo "Starting ${IMAGE_TAG} with spawn config ${SPAWN_ENV}"
docker run "${DOCKER_IT[@]}" --rm \
	--net=host \
	--privileged \
	--name "${CONTAINER_NAME}" \
	--env="DISPLAY=${DISPLAY}" \
	--env="QT_X11_NO_MITSHM=1" \
	--env="XAUTHORITY=${XAUTH_FILE}" \
	--env="XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR}" \
	"${DOCKER_VOLUMES[@]}" \
	"${IMAGE_TAG}" \
	/bin/bash -lc "set -euo pipefail; set -a; source '${CONTAINER_SPAWN_ENV}'; set +a; cd /home/valentin/PX4-Autopilot; make px4_sitl_nolockstep flightgear_rascal"
