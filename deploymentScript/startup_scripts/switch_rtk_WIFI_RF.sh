#!/usr/bin/env bash
# Switch companion RTK path (WiFi/LAN or serial RF) and/or RTCM sink; restarts ZMQ_to_comm and GPS/RTCM window.
#
# Usage:
#   ~/RL/startup_scripts/switch_rtk_WIFI_RF.sh [drone_id]
#   COMPANION_RTK_MODE=wifi BASE_HOST=192.168.0.43 ~/RL/startup_scripts/switch_rtk_WIFI_RF.sh 3
#   ~/RL/startup_scripts/switch_rtk_WIFI_RF.sh 3 --serial
#   ~/RL/startup_scripts/switch_rtk_WIFI_RF.sh 3 --wifi --base-host=192.168.0.43
#   ~/RL/startup_scripts/switch_rtk_WIFI_RF.sh 3 --sink=mavlink_rtcm
#   ~/RL/startup_scripts/switch_rtk_WIFI_RF.sh 3 --sink=rover_uart --wifi

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CATSWARM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/util/companion_rtk_connection.sh"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/util/companion_gps_module.sh"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/util/ensure_mavlink_rtcm_udp.sh"

SESSION="${CATSWARM_TMUX_SESSION:-catswarm_sim}"
DRONE_ID=3
HW_WIN="hardware_adapter_${DRONE_ID}"
UART2="${COMPANION_UART2_GS_RADIO:-/dev/ttyAMA2}"
PYTHON="${PYTHON:-/home/pi/miniconda/envs/RL/bin/python}"
export RTK_ZMQ_SNDHWM="${RTK_ZMQ_SNDHWM:-4096}"
export RTK_ZMQ_COALESCE_MS="${RTK_ZMQ_COALESCE_MS:-120}"
_RTK_MODE_EXPLICIT=0
_RTK_SINK_EXPLICIT=0

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [drone_id] [options]
  $(basename "$0") -h|--help

Restart ZMQ_to_comm RTK bridge and the GPS/RTCM tmux window.

Options:
  --wifi                  RTK via WiFi/LAN to GS PC and save
  --serial, --rf          RTK via UART2 RF bridge and save
  --base-host=IP          GS IP for WiFi RTK
  --sink=mavlink_rtcm|rover_uart
                          Correction sink (default rover_uart). mavlink_rtcm starts
                          rtcm_to_mavlink (no rover_zmq). RF Apply (--serial) → rf-only
                          bridge; WiFi Apply (--wifi) → wifi-only bridge.

Saved GPS preference: $(companion_gps_state_file)
Saved RTK preference:  $(companion_rtk_state_file)

Examples:
  $(basename "$0") 3 --wifi --base-host=192.168.0.43
  $(basename "$0") 3 --serial
  $(basename "$0") 3 --sink=mavlink_rtcm
  $(basename "$0") 3 --sink=rover_uart --wifi
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --wifi) COMPANION_RTK_MODE=wifi; _RTK_MODE_EXPLICIT=1; shift ;;
    --serial|--rf) COMPANION_RTK_MODE=serial; _RTK_MODE_EXPLICIT=1; shift ;;
    --sink=*) COMPANION_RTK_SINK="${1#--sink=}"; _RTK_SINK_EXPLICIT=1; shift ;;
    --sink)
      [ $# -lt 2 ] && { echo "switch_rtk_WIFI_RF.sh: --sink requires mavlink_rtcm|rover_uart" >&2; exit 1; }
      COMPANION_RTK_SINK="$2"; _RTK_SINK_EXPLICIT=1; shift 2 ;;
    --base-host=*) COMPANION_BASE_HOST="${1#*=}"; _RTK_MODE_EXPLICIT=1; export COMPANION_RTK_HOST_EXPLICIT=1; shift ;;
    --base-host)
      [ $# -lt 2 ] && { echo "switch_rtk_WIFI_RF.sh: --base-host requires IP" >&2; exit 1; }
      COMPANION_BASE_HOST="$2"; _RTK_MODE_EXPLICIT=1; export COMPANION_RTK_HOST_EXPLICIT=1; shift 2 ;;
    [0-9]*) DRONE_ID="$1"; HW_WIN="hardware_adapter_${DRONE_ID}"; shift ;;
    -*)
      echo "$(basename "$0"): unknown option: $1" >&2
      echo "Run: $(basename "$0") --help" >&2
      exit 1
      ;;
    *)
      echo "$(basename "$0"): unexpected argument: $1" >&2
      echo "Run: $(basename "$0") --help" >&2
      exit 1
      ;;
  esac
done

companion_gps_resolve_module
companion_gps_apply_module

if [[ "${_RTK_MODE_EXPLICIT}" -eq 1 || "${_RTK_SINK_EXPLICIT}" -eq 1 ]]; then
  companion_rtk_resolve_mode save
else
  companion_rtk_resolve_mode
fi
companion_rtk_apply_mode
companion_rtk_show_current_choice "${COMPANION_RTK_SOURCE:-}"

if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "switch_rtk_WIFI_RF.sh: tmux session ${SESSION} not found" >&2
  exit 1
fi

Z2C_BIN="${CATSWARM_ROOT}/hardware_adapter/bin/ZMQ_to_comm_c"
Z2C_PY="${CATSWARM_ROOT}/hardware_adapter/python/ZMQ_to_comm.py"
# Fleet C binary uses MultiInput-style ports (base + drone_id); Python path below is legacy.
FD_PORT=$((21700 + DRONE_ID))
COMM_PUB_PORT=$((7800 + DRONE_ID))
NEIGH_SUB_PORT=$((22700 + DRONE_ID))
MAV_FB_PORT=$((9900 + DRONE_ID))
if [[ -x "${Z2C_BIN}" ]]; then
  Z2C_CMD="${Z2C_BIN}"
  Z2C_CMD+=" --zmq-flight-data-port=${FD_PORT} --zmq-comm-pub-port=${COMM_PUB_PORT}"
  Z2C_CMD+=" --drone-id=${DRONE_ID} --zmq-mavlink-fallback-port=${MAV_FB_PORT}"
  Z2C_CMD+=" --zmq-comm-neighbour-sub-port=${NEIGH_SUB_PORT}"
  Z2C_CMD+=" --zmq-sys-manager-out-port=${FD_PORT}"
  Z2C_CMD+=" --serialcomm=${UART2} --zmq-gs-forward-port=7799"
  if [[ "${COMPANION_USE_RF_RTK_BRIDGE}" -eq 1 ]]; then
    Z2C_CMD+=" --rtk-zmq-bind=${COMPANION_RTK_ZMQ_BIND} --serial-comm-tx"
  else
    Z2C_CMD+=" --serial-comm-tx"
  fi
  _Z2C_LAUNCH_CD="${CATSWARM_ROOT}/hardware_adapter"
else
  Z2C_CMD="${PYTHON} ${Z2C_PY}"
  Z2C_CMD+=" --zmq-flight-data-port=${FD_PORT} --zmq-comm-pub-port=${COMM_PUB_PORT}"
  Z2C_CMD+=" --drone-id=${DRONE_ID} --zmq-mavlink-fallback-port=${MAV_FB_PORT}"
  Z2C_CMD+=" --zmq-comm-neighbour-sub-port=${NEIGH_SUB_PORT}"
  Z2C_CMD+=" --serialcomm=${UART2} --serial-comm-tx"
  if [[ "${COMPANION_USE_RF_RTK_BRIDGE}" -eq 1 ]]; then
    Z2C_CMD+=" --rtk-zmq-bind=${COMPANION_RTK_ZMQ_BIND}"
  fi
  _Z2C_LAUNCH_CD="${CATSWARM_ROOT}/hardware_adapter/python"
fi

_send() {
  local target="$1"
  local inner_cmd="$2"
  tmux send-keys -t "${target}" C-c
  sleep 1.5
  local launch="cd ${_Z2C_LAUNCH_CD} && export PYTHONPATH=${CATSWARM_ROOT}/system_manager/system_managerPY:\$PYTHONPATH && ${inner_cmd}"
  if [[ -n "${COMPANION_RUN_IN_RL:-}" ]] && [[ -x "${COMPANION_RUN_IN_RL}" ]]; then
    launch="${COMPANION_RUN_IN_RL} $(printf '%q' "${launch}")"
  fi
  tmux send-keys -t "${target}" "${launch}" C-m
}

echo "switch_rtk_WIFI_RF.sh: RTK=$(companion_rtk_mode_label) sink=${COMPANION_RTK_SINK} GPS=$(companion_gps_module_label)" >&2
echo "switch_rtk_WIFI_RF.sh: restarting ${SESSION}:${HW_WIN}.3 (ZMQ_to_comm)…" >&2
_send "${SESSION}:${HW_WIN}.3" "${Z2C_CMD}"

RTCM_WIN="${COMPANION_RTCM_MAV_WINDOW:-rtcm_mav}"

if [[ "${COMPANION_RTK_SINK}" == "mavlink_rtcm" ]]; then
  # Stop rover_uart stack; start rtcm_to_mavlink.
  # RTCM inject needs mavlink-server udpserver :14580 (often missing on older fleet conf).
  companion_ensure_mavlink_rtcm_udp || echo "switch_rtk_WIFI_RF.sh: WARN ensure 14580 failed — RTCM may not reach FC" >&2
  pkill -TERM -f "GPS_RTK/combination/rover_zmq.py" 2>/dev/null || true
  pkill -TERM -f "emulate_gps_to_px4" 2>/dev/null || true
  for old_win in gps_ea gps_rtk gps_f9p; do
    tmux kill-window -t "${SESSION}:${old_win}" 2>/dev/null || true
  done
  echo "switch_rtk_WIFI_RF.sh: starting ${SESSION}:${RTCM_WIN} (rtcm_to_mavlink)…" >&2
  companion_rtcm_mavlink_start_in_tmux "${SESSION}" "${PYTHON}" "${CATSWARM_ROOT}"
else
  # Stop mavlink RTCM bridge; restart GPS rover window.
  pkill -TERM -f "GPS_RTK/combination/rtcm_to_mavlink.py" 2>/dev/null || true
  tmux kill-window -t "${SESSION}:${RTCM_WIN}" 2>/dev/null || true
  for old_win in gps_ea gps_rtk gps_f9p; do
    if [[ "${old_win}" != "${COMPANION_GPS_WINDOW}" ]] \
        && tmux list-windows -t "${SESSION}" -F "#{window_name}" 2>/dev/null | grep -qx "${old_win}"; then
      tmux kill-window -t "${SESSION}:${old_win}" 2>/dev/null || true
    fi
  done
  echo "switch_rtk_WIFI_RF.sh: restarting ${SESSION}:${COMPANION_GPS_WINDOW} → ${COMPANION_RTK_ZMQ_URL}…" >&2
  companion_gps_start_in_tmux "${SESSION}" "${COMPANION_RTK_ZMQ_URL}" "${PYTHON}" "${CATSWARM_ROOT}"
fi

sleep 2
echo "switch_rtk_WIFI_RF.sh: done. Attach: tmux attach -t ${SESSION}" >&2
