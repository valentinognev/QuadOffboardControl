#!/usr/bin/env bash
# Switch companion GPS rover module (EA USB or DA UART) and restart the GPS tmux window.
#
# Usage:
#   ~/RL/startup_scripts/switch_EAUSB_DAUART.sh [drone_id]
#   ~/RL/startup_scripts/switch_EAUSB_DAUART.sh 3 --ea
#   ~/RL/startup_scripts/switch_EAUSB_DAUART.sh 3 --da

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CATSWARM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/util/companion_rtk_connection.sh"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/util/companion_gps_module.sh"

SESSION="${CATSWARM_TMUX_SESSION:-catswarm_sim}"
DRONE_ID=3
PYTHON="${PYTHON:-/home/pi/miniconda/envs/RL/bin/python}"
_GPS_MODULE_EXPLICIT=0
_RTK_MODE_EXPLICIT=0

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [drone_id] [options]
  $(basename "$0") -h|--help

Restart the companion GPS tmux window using the saved module (EA or DA),
or switch module and save the new preference.

Options:
  --ea, --gps-ea          Use EA on USB (/dev/ttyUSB0) and save
  --da, --gps-da          Use DA on UART1 (/dev/ttyAMA0) + PX4 NMEA and save
  --gps-module=ea|da      Same as --ea / --da
  --wifi                  RTK via WiFi/LAN (also restarts with saved RTK host)
  --serial, --rf          RTK via serial RF bridge
  --base-host=IP          GS IP for WiFi RTK

Saved GPS preference: $(companion_gps_state_file)
Saved RTK preference:  $(companion_rtk_state_file)

Examples:
  $(basename "$0")              # restart GPS window (saved module + RTK)
  $(basename "$0") 3 --da       # switch to DA UART and restart
  $(basename "$0") --ea --wifi --base-host=192.168.0.43
EOF
}

show_status() {
  companion_gps_resolve_module
  companion_gps_apply_module
  companion_gps_show_current_choice "${COMPANION_GPS_SOURCE:-}"
  companion_rtk_resolve_mode
  companion_rtk_apply_mode
  companion_rtk_show_current_choice "${COMPANION_RTK_SOURCE:-}"
}

while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --status) show_status; exit 0 ;;
    --ea|--gps-ea) COMPANION_GPS_MODULE=ea; _GPS_MODULE_EXPLICIT=1; shift ;;
    --da|--gps-da) COMPANION_GPS_MODULE=da; _GPS_MODULE_EXPLICIT=1; shift ;;
    --gps-module=*) COMPANION_GPS_MODULE="${1#*=}"; _GPS_MODULE_EXPLICIT=1; shift ;;
    --wifi) COMPANION_RTK_MODE=wifi; _RTK_MODE_EXPLICIT=1; shift ;;
    --serial|--rf) COMPANION_RTK_MODE=serial; _RTK_MODE_EXPLICIT=1; shift ;;
    --base-host=*) COMPANION_BASE_HOST="${1#*=}"; _RTK_MODE_EXPLICIT=1; export COMPANION_RTK_HOST_EXPLICIT=1; shift ;;
    [0-9]*) DRONE_ID="$1"; shift ;;
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

if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
  echo "switch_EAUSB_DAUART.sh: tmux session ${SESSION} not found" >&2
  exit 1
fi

# Remove stale GPS window from the other module (ea ↔ da use different names).
for old_win in gps_ea gps_rtk; do
  if [[ "${old_win}" != "${COMPANION_GPS_WINDOW}" ]] \
      && tmux list-windows -t "${SESSION}" -F "#{window_name}" 2>/dev/null | grep -qx "${old_win}"; then
    tmux kill-window -t "${SESSION}:${old_win}" 2>/dev/null || true
  fi
done

companion_gps_start_in_tmux "${SESSION}" "${COMPANION_RTK_ZMQ_URL}" "${PYTHON}" "${CATSWARM_ROOT}"
sleep 2
echo "switch_EAUSB_DAUART.sh: done. Window ${SESSION}:${COMPANION_GPS_WINDOW} ($(companion_gps_module_label))" >&2
