#!/usr/bin/env bash
#
# Raspberry Pi 5 companion computer: start Hardware Adapter + System Manager in tmux
# with the same pane layout as the CatSwarm GUI (hardware_adapter_multi.sh).
#
# ── Fleet hardware (Pi 5, /boot/firmware/config.txt overlays) ─────────────────
#
#   Fleet UART   Role                         Typical /dev      Started by
#   ──────────   ────                         ─────────────     ────────────
#   UART1        LC29H DA — GPS/RTK rover in   /dev/ttyAMA0      startRtkCommPI.sh (rover_zmq)
#   UART2        Ground-station radio link    /dev/ttyAMA2      ZMQ_to_comm PY (--serialcomm)
#   UART3        PX4 MAVLink telemetry        /dev/ttyAMA3      mavlink-server (separate service)
#   UART4        NMEA / GPS out to PX4         /dev/ttyAMA4      startRtkCommPI.sh (emulate_gps_to_px4)
#
#   RTK corrections: GS 107-byte frames on UART2 → ZMQ_to_comm reassembly → local ZMQ
#   tcp://127.0.0.1:5562 → rover_zmq → DA UART1.  GS PC runs startRtkCommGS.sh.
#
# Usage:
#   ./start_companion_drone_tmux.sh <drone_id> [CPP|PY|python]
#   ./start_companion_drone_tmux.sh --drone-id=N [--version=PY] [--serial=DEVICE] [--session=NAME]
#   ./start_companion_drone_tmux.sh --kill [--session=NAME]
#
# Pi 5 example (GS radio on UART2, drone 1):
#   ./start_companion_drone_tmux.sh 1 --serial=/dev/ttyAMA2

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CATSWARM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
HW_SCRIPT="${CATSWARM_ROOT}/hardware_adapter/hardware_adapter_multi.sh"
MULTI_SETUP_LIST="${CATSWARM_ROOT}/system_manager/MultiInput/multiSetup.list"
CPP_SYS_MANAGER="${CATSWARM_ROOT}/system_manager/SystemManagerMain"
PY_SYS_MANAGER_DIR="${CATSWARM_ROOT}/system_manager/system_managerPY"
PY_SYS_MANAGER="${PY_SYS_MANAGER_DIR}/system_manager.py"
VENV_PYTHON="${CATSWARM_ROOT}/venv/bin/python"
if [ ! -x "${VENV_PYTHON}" ] && [ -x "/home/pi/miniconda/envs/RL/bin/python" ]; then
    VENV_PYTHON="/home/pi/miniconda/envs/RL/bin/python"
fi
COMPANION_PYTHON="${COMPANION_PYTHON:-${VENV_PYTHON}}"
if [ -x "${COMPANION_PYTHON}" ]; then
    export PYTHON="${COMPANION_PYTHON}"
fi
GPS_PI_LAUNCHER="${CATSWARM_ROOT}/GPS_RTK/startRtkCommPI.sh"
UART_DEPLOY_DOC="${SCRIPT_DIR}/RPi_second_UART_GPIO4_GPIO5_deployment.md"
GPS_GUIDE="${CATSWARM_ROOT}/GPS_RTK/docs/guide-raspberry-pi-rover-px4.md"

TMUX_SESSION="${CATSWARM_TMUX_SESSION:-catswarm_sim}"
GPS_WINDOW="${COMPANION_GPS_WINDOW:-gps_rtk}"
# Local ZMQ bridge: ZMQ_to_comm PUSH → rover_zmq PULL (same process owns UART2).
COMPANION_RTK_ZMQ_BIND="${COMPANION_RTK_ZMQ_BIND:-tcp://127.0.0.1:5562}"

UART1_GPS_DA="${COMPANION_UART1_GPS_DA:-/dev/ttyAMA0}"
UART2_GS_RADIO="${COMPANION_UART2_GS_RADIO:-/dev/ttyAMA2}"
UART3_PX4_MAVLINK="${COMPANION_UART3_PX4_MAVLINK:-/dev/ttyAMA3}"
UART4_PX4_GPS_NMEA="${COMPANION_UART4_PX4_GPS_NMEA:-/dev/ttyAMA4}"

DRONE_ID=""
VERSION="PY"
SERIAL_DEVICE=""

usage() {
    cat <<EOF
Usage:
  $(basename "$0") <drone_id> [CPP|PY|python]
  $(basename "$0") --drone-id=N [--version=PY] [--serial=DEVICE] [--session=SESSION]
  $(basename "$0") --kill [--session=SESSION]

Companion: tmux ${TMUX_SESSION} with hardware_adapter_<id> + ${GPS_WINDOW} (RF RTK).

  --kill                 Stop companion tmux session and comm/GPS processes

  UART2 ${UART2_GS_RADIO} — ZMQ_to_comm (PY) reassembles GS RTK → ${COMPANION_RTK_ZMQ_BIND}
  UART1 ${UART1_GPS_DA} / UART4 ${UART4_PX4_GPS_NMEA} — startRtkCommPI.sh

GS PC: ./startRtkCommGS.sh  (comm default /dev/ttyUSB0, BS on other tty)

Paths: ${CATSWARM_ROOT}
EOF
}

print_topology() {
    cat <<EOF
Companion topology (Raspberry Pi 5):
  UART1 ${UART1_GPS_DA}  — LC29H DA (${GPS_WINDOW})
  UART2 ${UART2_GS_RADIO}  — GS radio → ZMQ_to_comm PY${SERIAL_DEVICE:+ (${SERIAL_DEVICE})}
  UART3 ${UART3_PX4_MAVLINK}  — PX4 MAVLink
  UART4 ${UART4_PX4_GPS_NMEA}  — NMEA to PX4 (${GPS_WINDOW})
  RTK path — UART2 GS frames → ${COMPANION_RTK_ZMQ_BIND} → rover_zmq → DA
EOF
}

start_gps_combo() {
    if [ ! -f "${GPS_PI_LAUNCHER}" ]; then
        echo "Error: GPS launcher not found: ${GPS_PI_LAUNCHER}" >&2
        return 1
    fi
    echo "Starting GPS/RTK stack in ${TMUX_SESSION}:${GPS_WINDOW}…"
    GPS_ARGS=(
        --rtk-zmq-url="${COMPANION_RTK_ZMQ_BIND}"
        --da-port="${UART1_GPS_DA}"
        --px4-port="${UART4_PX4_GPS_NMEA}"
        --join-session
        --no-attach
        --session="${TMUX_SESSION}"
        --window="${GPS_WINDOW}"
    )
    if [ -x "${COMPANION_PYTHON:-}" ]; then
        GPS_ARGS+=( --python="${COMPANION_PYTHON}" )
    fi
    "${GPS_PI_LAUNCHER}" "${GPS_ARGS[@]}"
    sleep 1
    if ! tmux list-windows -t "${TMUX_SESSION}" -F "#{window_name}" 2>/dev/null | grep -qx "${GPS_WINDOW}"; then
        echo "Error: tmux window ${TMUX_SESSION}:${GPS_WINDOW} not found after GPS start." >&2
        return 1
    fi
}

is_raspberry_pi_5() {
    local model=""
    if [ -r /proc/device-tree/model ]; then
        model="$(tr -d '\0' </proc/device-tree/model)"
    fi
    [[ "${model}" == *"Raspberry Pi 5"* ]]
}

kill_companion() {
    echo "Stopping companion stack (session ${TMUX_SESSION})…" >&2
    pkill -TERM -f "hardware_adapter/python/ZMQ_to_comm.py" 2>/dev/null || true
    pkill -TERM -f "bin/ZMQ_to_comm_c" 2>/dev/null || true
    pkill -TERM -f "bin/comm_to_ZMQ_c" 2>/dev/null || true
    pkill -TERM -f "hardware_adapter/python/mavlink_to_ZMQ.py" 2>/dev/null || true
    pkill -TERM -f "hardware_adapter/python/comm_to_ZMQ.py" 2>/dev/null || true
    pkill -TERM -f "hardware_adapter/python/zmq_commands_mavlink.py" 2>/dev/null || true
    pkill -TERM -f "GPS_RTK/combination/rover_zmq.py" 2>/dev/null || true
    pkill -TERM -f "GPS_RTK/PX4Integration/emulate_gps_to_px4.py" 2>/dev/null || true
    sleep 0.4
    if tmux has-session -t "${TMUX_SESSION}" 2>/dev/null; then
        tmux kill-session -t "${TMUX_SESSION}"
        echo "start_companion_drone_tmux.sh: killed tmux session ${TMUX_SESSION}" >&2
    else
        echo "start_companion_drone_tmux.sh: tmux session ${TMUX_SESSION} not running" >&2
    fi
}

_KILL=0
while [ $# -gt 0 ]; do
    case "$1" in
        -h|--help|help) usage; exit 0 ;;
        --kill) _KILL=1; shift ;;
        --drone-id=*) DRONE_ID="${1#--drone-id=}"; shift ;;
        --version=*) VERSION="${1#--version=}"; shift ;;
        --serial=*) SERIAL_DEVICE="${1#--serial=}"; shift ;;
        --session=*) TMUX_SESSION="${1#--session=}"; shift ;;
        -*)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 1
            ;;
        *)
            if [ -z "${DRONE_ID}" ]; then
                DRONE_ID="$1"
            elif echo "$1" | grep -qiE '^(CPP|PY|python)$'; then
                VERSION="$1"
            else
                echo "Unexpected argument: $1" >&2
                exit 1
            fi
            shift
            ;;
    esac
done

if [[ $_KILL -eq 1 ]]; then
    kill_companion
    exit 0
fi

if [ -z "${DRONE_ID}" ]; then
    echo "Error: drone id is required." >&2
    usage >&2
    exit 1
fi

if ! [[ "${DRONE_ID}" =~ ^[0-9]+$ ]] || [ "${DRONE_ID}" -lt 1 ]; then
    echo "Error: drone id must be a positive integer (got: ${DRONE_ID})" >&2
    exit 1
fi

VERSION_UPPER="$(printf '%s' "${VERSION}" | tr '[:lower:]' '[:upper:]')"
if [ "${VERSION_UPPER}" = "PYTHON" ]; then
    VERSION_UPPER="PY"
fi

# RF RTK reassembly in GS 107-byte frames is implemented in Python ZMQ_to_comm only.
if [ -n "${SERIAL_DEVICE}" ] && [ "${VERSION_UPPER}" = "CPP" ]; then
    echo "Note: RF RTK requires PY ZMQ_to_comm; switching --version from CPP to PY." >&2
    VERSION_UPPER="PY"
fi

if [ ! -f "${HW_SCRIPT}" ]; then
    echo "Error: hardware adapter script not found: ${HW_SCRIPT}" >&2
    exit 1
fi

if ! command -v tmux >/dev/null 2>&1; then
    echo "Error: tmux is not installed." >&2
    exit 1
fi

if [ -z "${SERIAL_DEVICE}" ] && is_raspberry_pi_5; then
    SERIAL_DEVICE="${UART2_GS_RADIO}"
fi

if [ -z "${SERIAL_DEVICE}" ]; then
    echo "Warning: --serial not set; ZMQ_to_comm will run without GS radio." >&2
elif [ ! -e "${SERIAL_DEVICE}" ]; then
    echo "Warning: serial device not present: ${SERIAL_DEVICE}" >&2
fi

if [ ! -f "${GPS_PI_LAUNCHER}" ]; then
    echo "Error: GPS launcher not found: ${GPS_PI_LAUNCHER}" >&2
    exit 1
fi

echo ""
print_topology
echo ""

HW_CMD=( "${HW_SCRIPT}"
    "--index=${DRONE_ID}"
    "--session=${TMUX_SESSION}"
    "--drone-id=${DRONE_ID}"
    "--version=${VERSION_UPPER}"
)
if [ -n "${SERIAL_DEVICE}" ]; then
    HW_CMD+=( "--serialcomm=${SERIAL_DEVICE}" )
    HW_CMD+=( "--rtk-zmq-bind=${COMPANION_RTK_ZMQ_BIND}" )
fi

echo "Starting hardware adapter (tmux layout)…"
echo "  Session:  ${TMUX_SESSION}"
echo "  Window:   hardware_adapter_${DRONE_ID}"
echo "  Version:  ${VERSION_UPPER}"
echo "  GS UART2: ${SERIAL_DEVICE:-<none>}"
echo "  RTK ZMQ:  ${COMPANION_RTK_ZMQ_BIND}"
"${HW_CMD[@]}"

HW_WINDOW="hardware_adapter_${DRONE_ID}"
sleep 2

if ! tmux list-windows -t "${TMUX_SESSION}" -F "#{window_name}" 2>/dev/null | grep -qx "${HW_WINDOW}"; then
    echo "Error: tmux window ${TMUX_SESSION}:${HW_WINDOW} not found after start." >&2
    exit 1
fi

if [ ! -f "${MULTI_SETUP_LIST}" ]; then
    echo "Error: multiSetup.list not found: ${MULTI_SETUP_LIST}" >&2
    exit 1
fi

mapfile -t CONFIG_LINES < <(grep -v '^[[:space:]]*$\|^[[:space:]]*#' "${MULTI_SETUP_LIST}" | sed 's/^[[:space:]]*//;s/[[:space:]]*$//')
LINE_INDEX=$(( DRONE_ID - 1 ))
if [ "${LINE_INDEX}" -lt 0 ] || [ "${LINE_INDEX}" -ge "${#CONFIG_LINES[@]}" ]; then
    echo "Error: no system manager config for drone id ${DRONE_ID}." >&2
    exit 1
fi

CONFIG_ENTRY="${CONFIG_LINES[${LINE_INDEX}]}"
if [[ "${CONFIG_ENTRY}" = /* ]]; then
    CONFIG_PATH="${CONFIG_ENTRY}"
else
    CONFIG_PATH="${CATSWARM_ROOT}/system_manager/MultiInput/${CONFIG_ENTRY}"
fi

if [ ! -f "${CONFIG_PATH}" ]; then
    echo "Error: system manager config not found: ${CONFIG_PATH}" >&2
    exit 1
fi

TARGET="${TMUX_SESSION}:${HW_WINDOW}.{bottom}"

start_cpp_sysmgr() {
    if [ ! -x "${CPP_SYS_MANAGER}" ]; then
        echo "Error: C++ system manager not found: ${CPP_SYS_MANAGER}" >&2
        exit 1
    fi
    tmux send-keys -t "${TARGET}" C-c
    sleep 0.3
    tmux send-keys -t "${TARGET}" "${CPP_SYS_MANAGER} ${CONFIG_PATH}" ENTER
}

start_py_sysmgr() {
    if [ ! -f "${PY_SYS_MANAGER}" ]; then
        echo "Error: Python system manager not found: ${PY_SYS_MANAGER}" >&2
        exit 1
    fi
    local pybin="python3"
    if [ -x "${VENV_PYTHON}" ]; then
        pybin="${VENV_PYTHON}"
    fi
    tmux send-keys -t "${TARGET}" C-c
    sleep 0.3
    tmux send-keys -t "${TARGET}" "cd ${PY_SYS_MANAGER_DIR} && ${pybin} system_manager.py --config=${CONFIG_PATH}" ENTER
}

echo "Starting system manager in bottom pane…"
echo "  Config: ${CONFIG_PATH}"

if [ "${VERSION_UPPER}" = "PY" ]; then
    start_py_sysmgr
elif [ -x "${CPP_SYS_MANAGER}" ]; then
    start_cpp_sysmgr
else
    start_py_sysmgr
fi

start_gps_combo

echo ""
echo "Done."
echo "  Attach: tmux attach -t ${TMUX_SESSION}"
echo "  Windows: ${HW_WINDOW}, ${GPS_WINDOW} (RF RTK via ${COMPANION_RTK_ZMQ_BIND})"
