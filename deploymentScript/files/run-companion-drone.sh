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
LAUNCHER="${RL_ROOT}/deployscripts/start_companion_drone_tmux.sh"
TMUX_SESSION="${CATSWARM_TMUX_SESSION:-catswarm_sim}"
UART_WAIT_S="${COMPANION_UART_WAIT_S:-30}"

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

wait_for_uarts() {
    local dev elapsed=0
    for dev in /dev/ttyAMA0 /dev/ttyAMA2 /dev/ttyAMA3 /dev/ttyAMA4; do
        elapsed=0
        while [ ! -e "${dev}" ] && [ "${elapsed}" -lt "${UART_WAIT_S}" ]; do
            sleep 1
            elapsed=$((elapsed + 1))
        done
        if [ ! -e "${dev}" ]; then
            echo "companion-drone: warning: ${dev} not present after ${UART_WAIT_S}s" >&2
        fi
    done
}

wait_for_uarts
exec "${LAUNCHER}" "${DRONE_ID}"
