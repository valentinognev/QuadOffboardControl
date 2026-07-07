#!/usr/bin/env bash
# Boot wrapper: start CatSwarm companion stack in tmux (hardware adapter + system manager + RTK).
set -euo pipefail

usage() {
    cat <<EOF
Usage: $0
       $0 -h|--help

Start CatSwarm companion stack at boot (called by companion-drone.service).

Reads configuration from:
  /etc/default/companion-drone
  ~/.config/companion-drone

Requires COMPANION_DRONE_ID in config. Installs via:
  sudo ~/deploy_pi5/install-companion-boot.sh <drone_id>

Logs: journalctl -u companion-drone -f
EOF
}

case "${1:-}" in
    -h|--help|help) usage; exit 0 ;;
    "")
        ;;
    *)
        echo "Error: unknown argument: $1" >&2
        echo "Run: $0 --help" >&2
        exit 1
        ;;
esac

if [ -f /etc/default/companion-drone ]; then
    # shellcheck disable=SC1091
    . /etc/default/companion-drone
elif [ -f "${HOME}/.config/companion-drone" ]; then
    # shellcheck disable=SC1091
    . "${HOME}/.config/companion-drone"
fi

DRONE_ID="${COMPANION_DRONE_ID:-}"
RL_ROOT="${RL_ROOT:-/home/pi/RL}"
LAUNCHER="${RL_ROOT}/startup_scripts/start_companion_drone_tmux.sh"
TMUX_SESSION="${CATSWARM_TMUX_SESSION:-catswarm_sim}"
UART_WAIT_S="${COMPANION_UART_WAIT_S:-30}"
USB_WAIT_S="${COMPANION_USB_WAIT_S:-${UART_WAIT_S}}"
GPS_STATE_FILE="${COMPANION_GPS_STATE_FILE:-${HOME}/.config/companion-gps}"
export COMPANION_BASE_HOST="${COMPANION_BASE_HOST:-192.168.0.43}"
export COMPANION_BASE_PORT_NUM="${COMPANION_BASE_PORT_NUM:-5560}"
export COMPANION_RTK_ZMQ_BIND="${COMPANION_RTK_ZMQ_BIND:-tcp://127.0.0.1:5562}"

if [ -z "${DRONE_ID}" ]; then
    echo "companion-drone: COMPANION_DRONE_ID is not set (install with: sudo install-companion-boot.sh <drone_id>)" >&2
    exit 1
fi

if ! [[ "${DRONE_ID}" =~ ^[0-9]+$ ]] || [ "${DRONE_ID}" -lt 1 ]; then
    echo "companion-drone: invalid COMPANION_DRONE_ID=${DRONE_ID}" >&2
    exit 1
fi

if [ ! -x "${LAUNCHER}" ]; then
    echo "companion-drone: launcher not found or not executable: ${LAUNCHER}" >&2
    exit 1
fi

_activate_companion_conda() {
    local conda_lib="/usr/local/lib/companion-conda-env.sh"
    if [ -f "${conda_lib}" ]; then
        # shellcheck disable=SC1091
        . "${conda_lib}"
        return 0
    fi
    local deploy_conda="${HOME}/deploy_pi5/files/companion-conda-env.sh"
    if [ -f "${deploy_conda}" ]; then
        # shellcheck disable=SC1091
        . "${deploy_conda}"
        return 0
    fi
    echo "companion-drone: conda env helper not found (install companion-conda-env.sh)" >&2
    exit 1
}

_activate_companion_conda
export RL_ROOT

if tmux has-session -t "${TMUX_SESSION}" 2>/dev/null; then
    echo "companion-drone: tmux session ${TMUX_SESSION} already running, skipping"
    exit 0
fi

_boot_gps_module() {
    local module="${COMPANION_GPS_MODULE:-}"
    if [[ -f "${GPS_STATE_FILE}" ]]; then
        # shellcheck disable=SC1090
        source "${GPS_STATE_FILE}"
    fi
    module="$(printf '%s' "${module:-ea}" | tr '[:upper:]' '[:lower:]')"
    case "${module}" in
        da|uart|serial|d) printf 'da\n' ;;
        *) printf 'ea\n' ;;
    esac
}

wait_for_device() {
    local dev="$1"
    local wait_s="$2"
    local elapsed=0
    while [ ! -e "${dev}" ] && [ "${elapsed}" -lt "${wait_s}" ]; do
        sleep 1
        elapsed=$((elapsed + 1))
    done
    if [ ! -e "${dev}" ]; then
        echo "companion-drone: warning: ${dev} not present after ${wait_s}s" >&2
        return 1
    fi
    return 0
}

wait_for_uarts() {
    local dev
    for dev in /dev/ttyAMA0 /dev/ttyAMA2 /dev/ttyAMA3 /dev/ttyAMA4; do
        wait_for_device "${dev}" "${UART_WAIT_S}" || true
    done
}

BOOT_GPS_MODULE="$(_boot_gps_module)"
BOOT_ROVER_PORT="${ROVER_PORT:-/dev/ttyUSB0}"
echo "companion-drone: boot GPS module=${BOOT_GPS_MODULE} (from ${GPS_STATE_FILE})" >&2
if [[ "${BOOT_GPS_MODULE}" == "ea" ]]; then
    wait_for_device "${BOOT_ROVER_PORT}" "${USB_WAIT_S}" || true
fi

wait_for_uarts
exec "${LAUNCHER}" "${DRONE_ID}"
