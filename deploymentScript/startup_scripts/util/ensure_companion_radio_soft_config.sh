#!/usr/bin/env bash
# Validate CommModules air radio soft-config vs rover timing; apply + NVS if needed.
# Invoked by ObservationBoard deploy New/Update after UART/GPS helpers.
#
# Usage:
#   ensure_companion_radio_soft_config.sh [drone_id] [--dry-run] [--no-restart]
#   COMPANION_DRONE_ID=2 ensure_companion_radio_soft_config.sh
#
# Soft-fail: missing radio / pyserial / no reply → warn and exit 0 (deploy continues).
# Exit 1 only on hard script errors (missing helper / invalid id).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STARTUP_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SESSION="${CATSWARM_TMUX_SESSION:-catswarm_sim}"
PYTHON="${PYTHON:-/home/pi/miniconda/envs/RL/bin/python}"
PORT="${COMPANION_UART2_GS_RADIO:-/dev/ttyAMA2}"
DRY_RUN=0
NO_RESTART=0
DRONE_ID="${COMPANION_DRONE_ID:-}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run|--no-apply) DRY_RUN=1; shift ;;
    --no-restart) NO_RESTART=1; shift ;;
    --port=*) PORT="${1#*=}"; shift ;;
    --port)
      [[ $# -lt 2 ]] && { echo "ensure_companion_radio_soft_config: --port needs path" >&2; exit 1; }
      PORT="$2"; shift 2
      ;;
    -h|--help)
      echo "Usage: $(basename "$0") [drone_id] [--dry-run] [--no-restart] [--port=/dev/ttyAMA2]"
      exit 0
      ;;
    [0-9]*)
      DRONE_ID="$1"; shift
      ;;
    *)
      echo "ensure_companion_radio_soft_config: unknown option: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${DRONE_ID}" ]]; then
  if [[ -f /etc/default/companion-drone ]]; then
    # shellcheck disable=SC1091
    # Parse without sourcing (avoid executing unknown defaults).
    DRONE_ID="$(
      awk -F= '/^[[:space:]]*COMPANION_DRONE_ID[[:space:]]*=/ {
        gsub(/[[:space:]"]/, "", $2); print $2; exit
      }' /etc/default/companion-drone 2>/dev/null || true
    )"
  fi
fi

if [[ -z "${DRONE_ID}" ]] || ! [[ "${DRONE_ID}" =~ ^[0-9]+$ ]] || [[ "${DRONE_ID}" -lt 1 ]]; then
  echo "ensure_companion_radio_soft_config: WARNING: no valid COMPANION_DRONE_ID — skip" >&2
  exit 0
fi

if [[ ! -x "${PYTHON}" ]]; then
  PYTHON="$(command -v python3 || true)"
fi
if [[ -z "${PYTHON}" ]]; then
  echo "ensure_companion_radio_soft_config: WARNING: no python — skip" >&2
  exit 0
fi

PY_HELPER="${SCRIPT_DIR}/ensure_companion_radio_soft_config.py"
if [[ ! -f "${PY_HELPER}" ]]; then
  echo "ensure_companion_radio_soft_config: ERROR: missing ${PY_HELPER}" >&2
  exit 1
fi

echo "ensure_companion_radio_soft_config: unit_id=${DRONE_ID} port=${PORT}" >&2

# Free UART2 so soft-config READ/WRITE is not fighting ZMQ_to_comm.
WAS_RUNNING=0
if pgrep -f 'ZMQ_to_comm(_c)?(\s|$)' >/dev/null 2>&1 \
  || pgrep -f 'hardware_adapter/python/ZMQ_to_comm.py' >/dev/null 2>&1 \
  || pgrep -f 'bin/ZMQ_to_comm_c' >/dev/null 2>&1; then
  WAS_RUNNING=1
fi

echo "ensure_companion_radio_soft_config: freeing ${PORT} (best-effort)…" >&2
pkill -TERM -f 'hardware_adapter/python/ZMQ_to_comm.py' 2>/dev/null || true
pkill -TERM -f 'bin/ZMQ_to_comm_c' 2>/dev/null || true
pkill -TERM -f 'ZMQ_to_comm_c' 2>/dev/null || true
# Also stop the tmux pane if present (avoids respawn races).
if tmux has-session -t "${SESSION}" 2>/dev/null; then
  HW_WIN="hardware_adapter_${DRONE_ID}"
  if tmux list-windows -t "${SESSION}" -F "#{window_name}" 2>/dev/null | grep -qx "${HW_WIN}"; then
    tmux send-keys -t "${SESSION}:${HW_WIN}.3" C-c 2>/dev/null || true
  fi
fi
sleep 0.6
fuser -k "${PORT}" 2>/dev/null || true
sleep 0.3

ARGS=(--port "${PORT}" --unit-id "${DRONE_ID}")
if [[ "${DRY_RUN}" -eq 1 ]]; then
  ARGS+=(--dry-run)
fi

set +e
"${PYTHON}" "${PY_HELPER}" "${ARGS[@]}"
rc=$?
set -e

if [[ "${rc}" -eq 2 ]]; then
  echo "ensure_companion_radio_soft_config: soft-fail (rc=2) — deploy continues" >&2
  rc=0
elif [[ "${rc}" -ne 0 ]]; then
  echo "ensure_companion_radio_soft_config: helper exited ${rc}" >&2
fi

# Restart ZMQ_to_comm when we stopped a live bridge and companion tmux is up.
if [[ "${NO_RESTART}" -eq 0 ]] && [[ "${WAS_RUNNING}" -eq 1 ]] && [[ "${DRY_RUN}" -eq 0 ]]; then
  SWITCH="${STARTUP_ROOT}/switch_rtk_WIFI_RF.sh"
  if tmux has-session -t "${SESSION}" 2>/dev/null; then
    if [[ -x "${SWITCH}" ]] || [[ -f "${SWITCH}" ]]; then
      echo "ensure_companion_radio_soft_config: restarting ZMQ_to_comm via switch_rtk_WIFI_RF.sh…" >&2
      bash "${SWITCH}" "${DRONE_ID}" || \
        echo "ensure_companion_radio_soft_config: WARNING: switch_rtk_WIFI_RF returned $?" >&2
    else
      echo "ensure_companion_radio_soft_config: WARNING: missing ${SWITCH}" >&2
    fi
  else
    echo "ensure_companion_radio_soft_config: no tmux ${SESSION} — radio NVS updated; bridge starts on next companion boot" >&2
  fi
fi

exit "${rc}"
