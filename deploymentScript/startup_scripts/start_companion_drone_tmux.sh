#!/usr/bin/env bash
#
# Raspberry Pi 5 companion computer: start Hardware Adapter + System Manager in tmux
# with the same pane layout as the CatSwarm GUI (hardware_adapter_multi.sh).
#
# ── Fleet hardware (Pi 5, /boot/firmware/config.txt overlays) ─────────────────
#
#   Fleet UART   Role                         Typical /dev      Started by
#   ──────────   ────                         ─────────────     ────────────
#   UART1        LC29H DA — GPS/RTK rover (optional) /dev/ttyAMA0  startRtkCommPI.sh
#   USB          LC29H EA — GPS/RTK rover (default) /dev/ttyUSB0 startRtkCommPI.sh
#   UART2        Ground-station radio link    /dev/ttyAMA2      ZMQ_to_comm PY (--serialcomm)
#   UART3        PX4 MAVLink telemetry        /dev/ttyAMA3      mavlink-server (separate service)
#   UART4        NMEA / GPS out to PX4         /dev/ttyAMA4      startRtkCommPI.sh (emulate_gps_to_px4)
#
#   RTK corrections (saved in ~/.config/companion-rtk; switch with switch_rtk_WIFI_RF.sh):
#     WiFi  — rover_zmq ZMQ PULL tcp://<GS_IP>:5560  (GS: startRtkWiFiGS.sh)
#     Serial — UART2 GS frames → ZMQ_to_comm → tcp://127.0.0.1:5562 → rover_zmq
#
# Usage:
#   ./start_companion_drone_tmux.sh <drone_id> [CPP|PY|python]
#   ./start_companion_drone_tmux.sh --drone-id=N [--version=CPP|PY] [--serial=DEVICE] [--session=NAME]
#   ./start_companion_drone_tmux.sh --kill [--session=NAME]
#
# Pi 5 example (GS radio on UART2, drone 1):
#   ./start_companion_drone_tmux.sh 1 --serial=/dev/ttyAMA2

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CATSWARM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/util/companion_rtk_connection.sh"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/util/companion_gps_module.sh"

_activate_companion_conda() {
    if [ -n "${COMPANION_RUN_IN_RL:-}" ] && [ -x "${COMPANION_RUN_IN_RL}" ] && [ -x "${PYTHON:-}" ]; then
        return 0
    fi
    local conda_lib="/usr/local/lib/companion-conda-env.sh"
    local deploy_conda="${HOME}/deploy_pi5/files/companion-conda-env.sh"
    if [ -f "${conda_lib}" ]; then
        # shellcheck disable=SC1091
        . "${conda_lib}"
    elif [ -f "${deploy_conda}" ]; then
        # shellcheck disable=SC1091
        . "${deploy_conda}"
    elif [ -f "/home/pi/miniconda/etc/profile.d/conda.sh" ]; then
        COMPANION_CONDA_ROOT="/home/pi/miniconda"
        COMPANION_CONDA_ENV="RL"
        COMPANION_CONDA_SH="${COMPANION_CONDA_ROOT}/etc/profile.d/conda.sh"
        # shellcheck disable=SC1090
        source "${COMPANION_CONDA_SH}"
        conda activate "${COMPANION_CONDA_ENV}"
        export PATH
        export PYTHON="${COMPANION_CONDA_ROOT}/envs/${COMPANION_CONDA_ENV}/bin/python"
        export COMPANION_PYTHON="${PYTHON}"
        for candidate in \
            /usr/local/bin/companion-run-in-rl.sh \
            "${HOME}/deploy_pi5/files/companion-run-in-rl.sh"; do
            if [ -x "${candidate}" ]; then
                export COMPANION_RUN_IN_RL="${candidate}"
                break
            fi
        done
        if [ -z "${COMPANION_RUN_IN_RL:-}" ]; then
            echo "Error: companion-run-in-rl.sh not found" >&2
            exit 1
        fi
    else
        echo "Error: conda RL env not found (install companion boot or miniconda)" >&2
        exit 1
    fi
}

_sync_tmux_conda_env() {
    local session="$1"
    if ! tmux has-session -t "${session}" 2>/dev/null; then
        return 0
    fi
    tmux set-option -t "${session}" default-shell /bin/bash
    tmux set-environment -t "${session}" PATH "${PATH}"
    tmux set-environment -t "${session}" PYTHON "${PYTHON}"
    tmux set-environment -t "${session}" CONDA_DEFAULT_ENV "${CONDA_DEFAULT_ENV:-RL}"
    if [ -n "${COMPANION_RUN_IN_RL:-}" ]; then
        tmux set-environment -t "${session}" COMPANION_RUN_IN_RL "${COMPANION_RUN_IN_RL}"
    fi
}

_activate_companion_conda

HW_SCRIPT="${CATSWARM_ROOT}/hardware_adapter/hardware_adapter_multi.sh"
MULTI_SETUP_LIST="${CATSWARM_ROOT}/system_manager/MultiInput/multiSetup.list"
CPP_SYS_MANAGER="${CATSWARM_ROOT}/system_manager/SystemManagerMain"
PY_SYS_MANAGER_DIR="${CATSWARM_ROOT}/system_manager/system_managerPY"
PY_SYS_MANAGER="${PY_SYS_MANAGER_DIR}/system_manager.py"
VENV_PYTHON="${COMPANION_PYTHON:-${PYTHON:-}}"
if [ -z "${VENV_PYTHON}" ] || [ ! -x "${VENV_PYTHON}" ]; then
    VENV_PYTHON="${CATSWARM_ROOT}/venv/bin/python"
    if [ ! -x "${VENV_PYTHON}" ] && [ -x "/home/pi/miniconda/envs/RL/bin/python" ]; then
        VENV_PYTHON="/home/pi/miniconda/envs/RL/bin/python"
    fi
fi
COMPANION_PYTHON="${COMPANION_PYTHON:-${VENV_PYTHON}}"
if [ -x "${COMPANION_PYTHON}" ]; then
    export PYTHON="${COMPANION_PYTHON}"
fi
# EA + DA both use startRtkCommPI.sh (port/baud choose the rover). Legacy startRtkEaUSB.sh is gone.
GPS_LAUNCHER="${CATSWARM_ROOT}/GPS_RTK/startRtkCommPI.sh"
GPS_EA_LAUNCHER="${GPS_LAUNCHER}"
GPS_DA_LAUNCHER="${GPS_LAUNCHER}"
UART_DEPLOY_DOC="${SCRIPT_DIR}/RPi_second_UART_GPIO4_GPIO5_deployment.md"
GPS_GUIDE="${CATSWARM_ROOT}/GPS_RTK/docs/guide-raspberry-pi-rover-px4.md"

TMUX_SESSION="${CATSWARM_TMUX_SESSION:-catswarm_sim}"
COMPANION_RTK_ZMQ_BIND="${COMPANION_RTK_ZMQ_BIND:-tcp://127.0.0.1:5562}"
COMPANION_BASE_HOST="${COMPANION_BASE_HOST:-192.168.0.43}"
COMPANION_BASE_PORT_NUM="${COMPANION_BASE_PORT_NUM:-5560}"
_RTK_MODE_EXPLICIT=0
_GPS_MODULE_EXPLICIT=0

UART1_GPS_DA="${COMPANION_UART1_GPS_DA:-/dev/ttyAMA0}"
UART2_GS_RADIO="${COMPANION_UART2_GS_RADIO:-/dev/ttyAMA2}"
UART3_PX4_MAVLINK="${COMPANION_UART3_PX4_MAVLINK:-/dev/ttyAMA3}"
UART4_PX4_GPS_NMEA="${COMPANION_UART4_PX4_GPS_NMEA:-/dev/ttyAMA4}"

DRONE_ID=""
VERSION="CPP"
SERIAL_DEVICE=""

usage() {
    cat <<EOF
Usage:
  $(basename "$0") <drone_id> [CPP|PY|python]   (default: CPP)
  $(basename "$0") --drone-id=N [--version=CPP] [--serial=DEVICE] [--session=SESSION]
  $(basename "$0") --kill [--session=SESSION]

Companion: tmux ${TMUX_SESSION} with hardware_adapter_<id> + GPS window (EA or DA).

  GPS module uses last saved mode (~/.config/companion-gps). Override once:
    --ea | --da | --gps-module=ea|da

  RTK uses last saved mode (~/.config/companion-rtk). Override once:
    --wifi | --serial | --rtk-mode=wifi|serial   --base-host=<GS IP>

  --kill                 Stop companion tmux session and comm/GPS processes

  WiFi:  rover_zmq → tcp://<GS>:5560  (GS: startRtkWiFiGS.sh)
  Serial: UART2 → ZMQ_to_comm → ${COMPANION_RTK_ZMQ_BIND}  (GS: startRtkCommGS.sh)

Paths: ${CATSWARM_ROOT}
EOF
}

print_topology() {
    local rtk_line gps_line
    if [[ "${COMPANION_USE_RF_RTK_BRIDGE:-0}" -eq 1 ]]; then
        rtk_line="Serial RF — UART2 → ${COMPANION_RTK_ZMQ_BIND} → rover_zmq"
    else
        rtk_line="WiFi/LAN — rover_zmq → ${COMPANION_RTK_ZMQ_URL:-tcp://${COMPANION_BASE_HOST}:${COMPANION_BASE_PORT_NUM}}"
    fi
    if [[ "${COMPANION_GPS_MODULE:-ea}" == "da" ]]; then
        gps_line="DA UART1 ${ROVER_PORT_UART} (${COMPANION_GPS_WINDOW:-gps_rtk}) + NMEA → ${COMPANION_PX4_GPS_PORT}"
    else
        gps_line="EA USB ${ROVER_PORT} (${COMPANION_GPS_WINDOW:-gps_ea}) + NMEA → ${COMPANION_PX4_GPS_PORT}"
    fi
    cat <<EOF
Companion topology (Raspberry Pi 5):
  GPS rover — ${gps_line}
  UART2 ${UART2_GS_RADIO}  — GS radio (ZMQ_to_comm PY)${SERIAL_DEVICE:+ (${SERIAL_DEVICE})}
  UART3 ${UART3_PX4_MAVLINK}  — PX4 MAVLink
  RTK path — ${rtk_line} → $(companion_gps_module_label | cut -d' ' -f1)
EOF
}

start_gps_combo() {
    companion_gps_start_in_tmux "${TMUX_SESSION}" "${COMPANION_RTK_ZMQ_URL}" "${COMPANION_PYTHON:-${PYTHON}}" "${CATSWARM_ROOT}"
    sleep 1
    if ! tmux list-windows -t "${TMUX_SESSION}" -F "#{window_name}" 2>/dev/null | grep -qx "${COMPANION_GPS_WINDOW}"; then
        echo "Error: tmux window ${TMUX_SESSION}:${COMPANION_GPS_WINDOW} not found after GPS start." >&2
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
        -h|--help) usage; exit 0 ;;
        --kill) _KILL=1; shift ;;
        --drone-id=*) DRONE_ID="${1#--drone-id=}"; shift ;;
        --version=*) VERSION="${1#--version=}"; shift ;;
        --serial=*) SERIAL_DEVICE="${1#--serial=}"; shift ;;
        --session=*) TMUX_SESSION="${1#--session=}"; shift ;;
        --rtk-mode=*) COMPANION_RTK_MODE="${1#--rtk-mode=}"; _RTK_MODE_EXPLICIT=1; shift ;;
        --base-host=*) COMPANION_BASE_HOST="${1#*=}"; _RTK_MODE_EXPLICIT=1; export COMPANION_RTK_HOST_EXPLICIT=1; shift ;;
        --wifi) COMPANION_RTK_MODE=wifi; _RTK_MODE_EXPLICIT=1; shift ;;
        --serial|--rf) COMPANION_RTK_MODE=serial; _RTK_MODE_EXPLICIT=1; shift ;;
        --ea|--gps-ea) COMPANION_GPS_MODULE=ea; _GPS_MODULE_EXPLICIT=1; shift ;;
        --da|--gps-da) COMPANION_GPS_MODULE=da; _GPS_MODULE_EXPLICIT=1; shift ;;
        --gps-module=*) COMPANION_GPS_MODULE="${1#--gps-module=}"; _GPS_MODULE_EXPLICIT=1; shift ;;
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

if [ ! -f "${GPS_LAUNCHER}" ]; then
    echo "Error: GPS launcher not found: ${GPS_LAUNCHER}" >&2
    exit 1
fi

# Resolve this drone's system_manager mission config (MultiInput/multiSetup.list, 1-indexed
# by DRONE_ID) *before* starting the hardware adapter, so its FLIGHT_DATA_FOR_COMM /
# COMM_NEIGHBOUR_DATA ports (zmqSysManagerOutPort / zmqSysManagerInPort) can be passed
# through instead of silently falling back to base+i (which will not match system_manager).
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

_read_mission_json_port() {
    # Mission JSON allows // comments; strip them the same way system_manager.py /
    # ObservationBoard app.py do before parsing.
    "${PYTHON}" -c "
import json, re, sys
with open(sys.argv[1]) as f:
    content = f.read()
content = re.sub(r'//.*', '', content)
try:
    data = json.loads(content)
except Exception:
    sys.exit(0)
val = data.get(sys.argv[2])
if val is not None:
    print(val)
" "${CONFIG_PATH}" "$1" 2>/dev/null
}

MISSION_SYS_MANAGER_OUT="$(_read_mission_json_port zmqSysManagerOutPort)"
MISSION_SYS_MANAGER_IN="$(_read_mission_json_port zmqSysManagerInPort)"

if [ -z "${MISSION_SYS_MANAGER_OUT}" ] || [ -z "${MISSION_SYS_MANAGER_IN}" ]; then
    echo "Warning: could not read zmqSysManagerOutPort/zmqSysManagerInPort from ${CONFIG_PATH};" >&2
    echo "         ZMQ_to_comm will fall back to base+i ports and will NOT match system_manager." >&2
fi

if [[ "${_GPS_MODULE_EXPLICIT}" -eq 1 ]]; then
    companion_gps_resolve_module save
else
    companion_gps_resolve_module
fi
companion_gps_apply_module
companion_gps_show_current_choice "${COMPANION_GPS_SOURCE:-}"

if [[ "${_RTK_MODE_EXPLICIT}" -eq 1 ]]; then
    companion_rtk_resolve_mode save
else
    companion_rtk_resolve_mode
fi
companion_rtk_apply_mode
companion_rtk_show_current_choice "${COMPANION_RTK_SOURCE:-}"

echo ""
print_topology
echo ""

HW_CMD=( "${HW_SCRIPT}"
    "--index=${DRONE_ID}"
    "--session=${TMUX_SESSION}"
    "--drone-id=${DRONE_ID}"
    "--version=${VERSION_UPPER}"
    # Real hardware: mavlink-server always bridges PX4 (UART3) to a single fixed
    # 127.0.0.1:14540 (see /etc/mavlink-server/mavlink-server.conf), regardless of
    # drone ID. hardware_adapter_multi.sh's default (14540+index) is only correct
    # for the desktop multi-drone simulator where each simulated PX4 instance binds
    # its own offset port. Without this override, mavlink_to_ZMQ listens on the
    # wrong port and never receives MAVLink data (GPS/flight data silently stay at 0).
    "--mavlink-udp=${COMPANION_MAVLINK_UDP_PORT:-14540}"
)
if [ -n "${SERIAL_DEVICE}" ]; then
    HW_CMD+=( "--serialcomm=${SERIAL_DEVICE}" )
    if [[ "${COMPANION_USE_RF_RTK_BRIDGE}" -eq 1 ]]; then
        HW_CMD+=( "--rtk-zmq-bind=${COMPANION_RTK_ZMQ_BIND}" )
        # ZMQ_to_comm defaults telemetry TX to off when --rtk-zmq-bind is set (half-duplex
        # UART: avoids TX colliding with RTCM RX). This topology needs both RTK downlink
        # AND drone telemetry uplink on the same radio, so force TX back on explicitly.
        HW_CMD+=( "--serial-comm-tx" )
    fi
fi
if [ -n "${MISSION_SYS_MANAGER_OUT}" ]; then
    HW_CMD+=( "--zmqSysManagerOutPort=${MISSION_SYS_MANAGER_OUT}" )
fi
if [ -n "${MISSION_SYS_MANAGER_IN}" ]; then
    HW_CMD+=( "--zmqSysManagerInPort=${MISSION_SYS_MANAGER_IN}" )
fi

echo "Starting hardware adapter (tmux layout)…"
echo "  Session:  ${TMUX_SESSION}"
echo "  Window:   hardware_adapter_${DRONE_ID}"
echo "  Version:  ${VERSION_UPPER}"
echo "  GS UART2: ${SERIAL_DEVICE:-<none>}"
echo "  RTK mode: ${COMPANION_RTK_MODE} → ${COMPANION_RTK_ZMQ_URL}"
echo "  GPS module: ${COMPANION_GPS_MODULE} → ${COMPANION_GPS_WINDOW}"
"${HW_CMD[@]}"

_sync_tmux_conda_env "${TMUX_SESSION}"

HW_WINDOW="hardware_adapter_${DRONE_ID}"
sleep 2

if ! tmux list-windows -t "${TMUX_SESSION}" -F "#{window_name}" 2>/dev/null | grep -qx "${HW_WINDOW}"; then
    echo "Error: tmux window ${TMUX_SESSION}:${HW_WINDOW} not found after start." >&2
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
    local pybin="${PYTHON}"
    local inner="cd ${PY_SYS_MANAGER_DIR} && ${pybin} system_manager.py --config=${CONFIG_PATH}"
    local cmd="${inner}"
    if [ -n "${COMPANION_RUN_IN_RL:-}" ] && [ -x "${COMPANION_RUN_IN_RL}" ]; then
        cmd="$(printf '%s %q' "${COMPANION_RUN_IN_RL}" "${inner}")"
    fi
    tmux send-keys -t "${TARGET}" C-c
    sleep 0.3
    tmux send-keys -t "${TARGET}" "${cmd}" ENTER
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
echo "  Windows: ${HW_WINDOW}, ${COMPANION_GPS_WINDOW} (GPS: $(companion_gps_module_label); RTK: $(companion_rtk_mode_label))"
