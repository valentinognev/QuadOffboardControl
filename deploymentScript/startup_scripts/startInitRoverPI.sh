#!/usr/bin/env bash
# Raspberry Pi 5 — configure LC29H DA rover on UART1 (/dev/ttyAMA0).
#
# Run from ~/RL/startup_scripts on the Pi (same layout as start_companion_drone_tmux.sh).
#
# PHASE 1 — set RTK rover mode (run once, then power-cycle):
#   Sends $PQTMCFGRCVRMODE,W,1 (rover mode) + $PQTMSAVEPAR.
#
# PHASE 2 — configure NMEA output (run once after power-cycle from phase 1):
#   Enables GGA, 5 Hz rate, disables noisy sentences, saves to flash.
#
# Optional --verify-rtk injects corrections from the ground station (RF comm or
# companion ZMQ bridge) and prints fix / RTCM status for a short window.
#
# Usage:
#   ~/RL/startup_scripts/startInitRoverPI.sh [options] [--phase1 | --phase2 | --verify-rtk]
#
# After init, start normal operation:
#   ~/RL/startup_scripts/start_companion_drone_tmux.sh 1
#   ~/RL/GPS_RTK/startRtkCommPI.sh --rtk-zmq-url=tcp://127.0.0.1:5562

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CATSWARM_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
GPS_ROOT="${CATSWARM_ROOT}/GPS_RTK"
# shellcheck source=util/gnss_serial_args.sh
source "${SCRIPT_DIR}/util/gnss_serial_args.sh"

PYTHON="${PYTHON:-python3}"
_COMM_PYTHON_SET=0
ROVER_PORT="${ROVER_PORT:-/dev/ttyAMA0}"
ROVER_BAUD="${ROVER_BAUD:-115200}"
RTK_ZMQ_URL="${RTK_ZMQ_URL:-}"
COMM_SERIAL="${COMM_SERIAL:-/dev/ttyAMA2}"
COMM_BAUD="${COMM_BAUD:-115200}"
VERIFY_SECS="${VERIFY_SECS:-90}"

ROVER_PY="${GPS_ROOT}/combination/rover_zmq.py"
_LAUNCHER="${SCRIPT_DIR}/$(basename "$0")"

_pick_python() {
  if [[ "$_COMM_PYTHON_SET" -eq 0 ]] \
      && [[ -z "${PYTHON##python*}" ]] \
      && [[ -x "/home/pi/miniconda/envs/RL/bin/python" ]]; then
    PYTHON="/home/pi/miniconda/envs/RL/bin/python"
  fi
}

_help() {
  cat <<EOF
Usage: ${_LAUNCHER} [options] [--phase1 | --phase2 | --verify-rtk]

Configure LC29H rover on Pi UART1 (default ${ROVER_PORT} @ ${ROVER_BAUD}).

  Rover   : ${ROVER_PORT} @ ${ROVER_BAUD}
  RTK     : ${RTK_ZMQ_URL:-<comm ${COMM_SERIAL} @ ${COMM_BAUD}>} (for --verify-rtk)

Options:
$(gnss_serial_help_options)
  --rtk-zmq-url=URL       ZMQ PULL for reassembled RTCM (companion: tcp://127.0.0.1:5562)
  --connect=URL           Alias for --rtk-zmq-url
  --comm-serial=DEVICE    Comm radio for RF RTK verify (default: /dev/ttyAMA2)
  --comm=DEVICE           Alias for --comm-serial
  --comm-baud=RATE        Comm radio baud (default: 115200)
  --verify-secs=SEC       Duration for --verify-rtk (default: 90)
  --python=PATH           Python interpreter
  --phase1                Phase 1 only (RTK rover mode + save)
  --phase2                Phase 2 only (GGA + 5 Hz — after power cycle)
  --verify-rtk            Inject station RTCM and print fix status (needs RTK source)
  -h, --help              Show this message

$(gnss_serial_help_env)
  RTK_ZMQ_URL, COMM_SERIAL, COMM_BAUD, VERIFY_SECS, PYTHON

Examples:
  ${_LAUNCHER} --phase1
  ${_LAUNCHER} --phase2
  ${_LAUNCHER} --verify-rtk --rtk-zmq-url=tcp://127.0.0.1:5562
  ${_LAUNCHER} --verify-rtk --comm-serial=/dev/ttyAMA2

Companion RF verify: start GS + start_companion_drone_tmux.sh (or ZMQ_to_comm), then --verify-rtk.
EOF
}

MODE="both"
while [ $# -gt 0 ]; do
  if gnss_serial_parse_arg "$1" "${2:-}"; then
    shift "${GNSS_SERIAL_SHIFT}"
    continue
  fi
  case "$1" in
    -h|--help) _help; exit 0 ;;
    --phase1) MODE="phase1"; shift ;;
    --phase2) MODE="phase2"; shift ;;
    --verify-rtk) MODE="verify"; shift ;;
    --rtk-zmq-url=*|--connect=*) RTK_ZMQ_URL="${1#*=}"; shift ;;
    --comm-serial=*|--comm=*) COMM_SERIAL="${1#*=}"; shift ;;
    --comm-baud=*) COMM_BAUD="${1#--comm-baud=}"; shift ;;
    --verify-secs=*) VERIFY_SECS="${1#*=}"; shift ;;
    --python=*) PYTHON="${1#--python=}"; _COMM_PYTHON_SET=1; shift ;;
    --rtk-zmq-url|--connect)
      [ $# -lt 2 ] && { echo "startInitRoverPI.sh: $1 requires a value" >&2; exit 1; }
      RTK_ZMQ_URL="$2"; shift 2 ;;
    --comm-serial|--comm)
      [ $# -lt 2 ] && { echo "startInitRoverPI.sh: $1 requires a value" >&2; exit 1; }
      COMM_SERIAL="$2"; shift 2 ;;
    --comm-baud)
      [ $# -lt 2 ] && { echo "startInitRoverPI.sh: --comm-baud requires a value" >&2; exit 1; }
      COMM_BAUD="$2"; shift 2 ;;
    --verify-secs)
      [ $# -lt 2 ] && { echo "startInitRoverPI.sh: --verify-secs requires a value" >&2; exit 1; }
      VERIFY_SECS="$2"; shift 2 ;;
    --python)
      [ $# -lt 2 ] && { echo "startInitRoverPI.sh: --python requires a value" >&2; exit 1; }
      PYTHON="$2"; _COMM_PYTHON_SET=1; shift 2 ;;
    -*)
      echo "startInitRoverPI.sh: unknown option: $1" >&2
      _help >&2
      exit 1
      ;;
    *)
      echo "startInitRoverPI.sh: unexpected argument: $1" >&2
      _help >&2
      exit 1
      ;;
  esac
done

gnss_serial_migrate_legacy_env
_pick_python

[[ -f "$ROVER_PY" ]] || { echo "missing $ROVER_PY (expected under ${GPS_ROOT})" >&2; exit 1; }
[[ -c "$ROVER_PORT" ]] || { echo "rover port $ROVER_PORT not found" >&2; exit 1; }

_rtk_source_desc() {
  if [[ -n "$RTK_ZMQ_URL" ]]; then
    echo "ZMQ ${RTK_ZMQ_URL}"
  else
    echo "serial ${COMM_SERIAL} @ ${COMM_BAUD}"
  fi
}

_rtk_verify_args() {
  if [[ -n "$RTK_ZMQ_URL" ]]; then
    printf '%s\n' "--connect" "$RTK_ZMQ_URL"
  else
    printf '%s\n' "--comm-serial" "$COMM_SERIAL" "--comm-baud" "$COMM_BAUD"
  fi
}

run_phase1() {
  echo "startInitRoverPI.sh: PHASE 1 — RTK rover mode on ${ROVER_PORT} @ ${ROVER_BAUD}" >&2
  "$PYTHON" "$ROVER_PY" --port "${ROVER_PORT}" --baud "${ROVER_BAUD}" \
      --init-rover --no-forward --no-csv --no-position
}

run_phase2() {
  echo "startInitRoverPI.sh: PHASE 2 — NMEA config on ${ROVER_PORT} @ ${ROVER_BAUD}" >&2
  "$PYTHON" "$ROVER_PY" --port "${ROVER_PORT}" --baud "${ROVER_BAUD}" \
      --config-rover-nmea --no-forward --no-csv --no-position
}

run_verify_rtk() {
  local rtk_src
  rtk_src="$(_rtk_source_desc)"
  echo "startInitRoverPI.sh: VERIFY RTK — ${rtk_src} → ${ROVER_PORT} for ${VERIFY_SECS}s (Ctrl+C to stop)" >&2
  if [[ -n "$RTK_ZMQ_URL" ]]; then
    echo "startInitRoverPI.sh: companion path — ensure ZMQ_to_comm is running with --rtk-zmq-bind ${RTK_ZMQ_URL}" >&2
  else
    echo "startInitRoverPI.sh: RF path — ensure GS is transmitting on comm radio (${COMM_SERIAL})" >&2
  fi
  mapfile -t _RTK_ARGS < <(_rtk_verify_args)
  if command -v timeout >/dev/null 2>&1; then
    timeout --foreground "${VERIFY_SECS}" \
      "$PYTHON" "$ROVER_PY" --port "${ROVER_PORT}" --baud "${ROVER_BAUD}" \
      "${_RTK_ARGS[@]}" --no-csv || true
  else
    "$PYTHON" "$ROVER_PY" --port "${ROVER_PORT}" --baud "${ROVER_BAUD}" \
      "${_RTK_ARGS[@]}" --no-csv &
    _pid=$!
    sleep "${VERIFY_SECS}"
    kill "$_pid" 2>/dev/null || true
    wait "$_pid" 2>/dev/null || true
  fi
}

_post_phase1() {
  echo "" >&2
  echo "startInitRoverPI.sh: Phase 1 done." >&2
  echo "  *** POWER CYCLE the rover module NOW. ***" >&2
  echo "  Then run:  ${_LAUNCHER} --phase2" >&2
}

_post_phase2() {
  echo "" >&2
  echo "startInitRoverPI.sh: Phase 2 done." >&2
  echo "  *** POWER CYCLE the rover module NOW. ***" >&2
  echo "  Then verify RTK from the station:" >&2
  echo "    ${_LAUNCHER} --verify-rtk --rtk-zmq-url=tcp://127.0.0.1:5562" >&2
  echo "  Or start the full stack:" >&2
  echo "    ${SCRIPT_DIR}/start_companion_drone_tmux.sh 1" >&2
  echo "    ${GPS_ROOT}/startRtkCommPI.sh --rtk-zmq-url=tcp://127.0.0.1:5562" >&2
}

case "$MODE" in
  phase1)
    run_phase1
    _post_phase1
    ;;
  phase2)
    run_phase2
    _post_phase2
    ;;
  verify)
    run_verify_rtk
    ;;
  both)
    run_phase1
    _post_phase1
    echo "  Press Enter after the power cycle to continue to phase 2." >&2
    read -r _dummy
    run_phase2
    _post_phase2
    ;;
  *)
    echo "usage: $0 [options] [--phase1 | --phase2 | --verify-rtk]" >&2
    exit 2
    ;;
esac
