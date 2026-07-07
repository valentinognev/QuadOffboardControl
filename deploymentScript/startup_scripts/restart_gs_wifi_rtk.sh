#!/usr/bin/env bash
# Run on the GROUND STATION PC (192.168.0.43) to restart WiFi RTK with steady-pipe defaults.
#
# Example:
#   BASE_PORT=/dev/ttyUSB2 BASE_BAUD=auto ~/RL/startup_scripts/restart_gs_wifi_rtk.sh
#
# Copies are in ~/RL on the GS PC — sync from Pi first if needed:
#   rsync -avz pi@192.168.0.32:~/RL/GPS_RTK/startRtkWiFiGS.sh ~/RL/GPS_RTK/
#   rsync -avz pi@192.168.0.32:~/RL/GPS_RTK/combination/base_zmq.py ~/RL/GPS_RTK/combination/

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RL_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
GS_LAUNCHER="${RL_ROOT}/GPS_RTK/startRtkWiFiGS.sh"

if [[ ! -x "${GS_LAUNCHER}" ]]; then
  echo "restart_gs_wifi_rtk.sh: missing ${GS_LAUNCHER}" >&2
  exit 1
fi

export BASE_PORT="${BASE_PORT:-/dev/ttyUSB2}"
export BASE_BAUD="${BASE_BAUD:-auto}"
export GS_ZMQ_SNDHWM="${GS_ZMQ_SNDHWM:-4096}"

echo "restart_gs_wifi_rtk.sh: stopping old WiFi GS session…" >&2
"${GS_LAUNCHER}" --kill 2>/dev/null || true
sleep 1

echo "restart_gs_wifi_rtk.sh: starting BASE_PORT=${BASE_PORT} BASE_BAUD=${BASE_BAUD}" >&2
exec env BASE_PORT="${BASE_PORT}" BASE_BAUD="${BASE_BAUD}" GS_ZMQ_SNDHWM="${GS_ZMQ_SNDHWM}" \
  "${GS_LAUNCHER}" 0.0.0.0
