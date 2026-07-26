#!/usr/bin/env bash
# Sniff LC29H on the Pi and flush ~/.config/companion-gps to matching fleet defaults.
# Sniff fail → warn + EA USB defaults (460800 / 10 Hz).
#
# Usage:
#   flush_companion_gps_from_hw.sh [--no-restart]
#
# Exit 0 always after writing a profile (including EA fallback). Exit 1 only on
# hard script errors (missing helpers).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STARTUP_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck source=companion_gps_module.sh
source "${SCRIPT_DIR}/companion_gps_module.sh"

SESSION="${CATSWARM_TMUX_SESSION:-catswarm_sim}"
PYTHON="${PYTHON:-/home/pi/miniconda/envs/RL/bin/python}"
NO_RESTART=0
USB_PORT="${ROVER_PORT:-/dev/ttyUSB0}"
UART_PORT="${ROVER_PORT_UART:-/dev/ttyAMA4}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-restart) NO_RESTART=1; shift ;;
    -h|--help)
      echo "Usage: $(basename "$0") [--no-restart]"
      exit 0
      ;;
    *)
      echo "flush_companion_gps_from_hw: unknown option: $1" >&2
      exit 1
      ;;
  esac
done

if [[ ! -x "${PYTHON}" ]]; then
  PYTHON="$(command -v python3 || true)"
fi
if [[ -z "${PYTHON}" ]]; then
  echo "flush_companion_gps_from_hw: no python interpreter" >&2
  exit 1
fi

SNIFF_PY="${SCRIPT_DIR}/sniff_lc29h_profile.py"
if [[ ! -f "${SNIFF_PY}" ]]; then
  echo "flush_companion_gps_from_hw: missing ${SNIFF_PY}" >&2
  exit 1
fi

echo "flush_companion_gps_from_hw: stopping GPS panes (best-effort)…" >&2
# Free the rover tty for VERNO sniff.
pkill -f 'rover_zmq.py' 2>/dev/null || true
pkill -f 'emulate_gps_to_px4.py' 2>/dev/null || true
if tmux has-session -t "${SESSION}" 2>/dev/null; then
  for win in gps_ea gps_rtk; do
    if tmux list-windows -t "${SESSION}" -F "#{window_name}" 2>/dev/null | grep -qx "${win}"; then
      tmux kill-window -t "${SESSION}:${win}" 2>/dev/null || true
    fi
  done
fi
sleep 0.5

echo "flush_companion_gps_from_hw: sniffing ${USB_PORT} then ${UART_PORT}…" >&2
set +e
TOKEN="$("${PYTHON}" "${SNIFF_PY}" --usb-port "${USB_PORT}" --uart-port "${UART_PORT}")"
sniff_rc=$?
set -e
TOKEN="$(printf '%s' "${TOKEN}" | tr -d '\r' | tail -n1)"
if [[ "${sniff_rc}" -ne 0 ]]; then
  echo "flush_companion_gps_from_hw: sniff exited ${sniff_rc}" >&2
  TOKEN="none"
fi

PROFILE=""
WARN=0
case "${TOKEN}" in
  usb_ea) PROFILE=ea ;;
  usb_da) PROFILE=da-usb ;;
  uart_da) PROFILE=da-uart ;;
  *)
    WARN=1
    PROFILE=ea
    echo "flush_companion_gps_from_hw: WARNING: sniff failed (token=${TOKEN:-empty}) — falling back to EA USB defaults" >&2
    ;;
esac

companion_gps_write_fleet_profile "${PROFILE}"

if [[ "${NO_RESTART}" -eq 0 ]]; then
  SWITCH="${STARTUP_ROOT}/switch_EAUSB_DAUART.sh"
  if [[ -x "${SWITCH}" ]] || [[ -f "${SWITCH}" ]]; then
    if tmux has-session -t "${SESSION}" 2>/dev/null; then
      echo "flush_companion_gps_from_hw: restarting GPS via switch_EAUSB_DAUART.sh…" >&2
      bash "${SWITCH}" || echo "flush_companion_gps_from_hw: WARNING: GPS restart returned $?" >&2
    else
      echo "flush_companion_gps_from_hw: no tmux session ${SESSION} — config flushed; GPS will start on next companion boot" >&2
    fi
  else
    echo "flush_companion_gps_from_hw: WARNING: missing ${SWITCH}" >&2
  fi
fi

if [[ "${WARN}" -eq 1 ]]; then
  echo "flush_companion_gps_from_hw: done with warning (EA fallback)" >&2
else
  echo "flush_companion_gps_from_hw: done (profile=${PROFILE})" >&2
fi
exit 0
