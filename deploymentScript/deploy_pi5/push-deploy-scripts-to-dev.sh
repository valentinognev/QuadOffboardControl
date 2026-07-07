#!/usr/bin/env bash
# Push ~/deploy_pi5 and ~/RL/startup_scripts from the Pi to a dev PC.
#
# Usage (on Pi):
#   ./push-deploy-scripts-to-dev.sh valentin@192.168.0.39
#   ./push-deploy-scripts-to-dev.sh valentin@192.168.0.39 --remote-path=/home/valentin/RL/CatSwarm/general_infrastructure/deploymentScript
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="${DEPLOY_SOURCE_DIR:-${SCRIPT_DIR}}"
RL_ROOT="${RL_ROOT:-/home/pi/RL}"
STARTUP_SCRIPTS_SRC="${RL_ROOT}/startup_scripts"

DEV_SPEC=""
DEV_USER="${DEV_PC_USER:-}"
DEV_HOST=""
DEV_PASSWORD="${DEV_PC_PASSWORD:-}"
REMOTE_PATH="${DEPLOY_REMOTE_PATH:-/home/valentin/RL/CatSwarm/general_infrastructure/deploymentScript}"
REMOTE_DEPLOY_PI5="${DEPLOY_REMOTE_DEPLOY_PI5:-}"
REMOTE_STARTUP_SCRIPTS="${DEPLOY_REMOTE_STARTUP_SCRIPTS:-}"

SSH_CTRL_DIR=""
SSH_CONTROL_PATH=""
SSH_BASE_OPTS=()

RSYNC_EXCLUDES=(
  --exclude='offline-wheels/'
  --exclude='.git/'
  --exclude='__pycache__/'
)

usage() {
  cat <<EOF
Usage:
  $0 <user@host|host> [--remote-path=PATH] [--remote-startup-scripts=PATH]
  $0 --help

Push deploy_pi5 tooling and startup_scripts from this Pi to a dev PC
(opposite of --from-dev rsync in deploy_pi5_companion.sh).

On the dev PC both trees live under deploymentScript/:
  ${REMOTE_PATH}/deploy_pi5/       deploy tooling (this script's directory)
  ${REMOTE_PATH}/startup_scripts/  companion launchers

Examples:
  $0 valentin@192.168.0.39
  $0 192.168.0.39 --remote-path=/home/valentin/RL/CatSwarm/general_infrastructure/deploymentScript

Sources (Pi):
  deploy_pi5:      ${SOURCE_DIR}/
  startup_scripts: ${STARTUP_SCRIPTS_SRC}/

Environment:
  DEV_PC_PASSWORD              SSH password (uses sshpass if set)
  DEPLOY_SOURCE_DIR            Override deploy_pi5 source directory
  DEPLOY_REMOTE_PATH             Remote deploymentScript directory
  DEPLOY_REMOTE_DEPLOY_PI5       Override remote deploy_pi5 path (default: <remote-path>/deploy_pi5)
  DEPLOY_REMOTE_STARTUP_SCRIPTS  Override remote startup_scripts path
  RL_ROOT                      Local RL tree (default: /home/pi/RL)
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

ensure_sshpass() {
  command -v sshpass >/dev/null 2>&1 || die "sshpass not installed (sudo apt install sshpass) or use SSH keys"
}

parse_dev_spec() {
  local spec="$1"
  if [[ "${spec}" == *@* ]]; then
    DEV_USER="${spec%%@*}"
    DEV_HOST="${spec#*@}"
  else
    DEV_HOST="${spec}"
    [ -n "${DEV_USER}" ] || die "use user@host or set DEV_PC_USER"
  fi
  [ -n "${DEV_HOST}" ] || die "remote host required"
  DEV_SPEC="${DEV_USER}@${DEV_HOST}"
}

resolve_remote_paths() {
  if [ -z "${REMOTE_DEPLOY_PI5}" ]; then
    REMOTE_DEPLOY_PI5="${REMOTE_PATH%/}/deploy_pi5"
  fi
  if [ -z "${REMOTE_STARTUP_SCRIPTS}" ]; then
    REMOTE_STARTUP_SCRIPTS="${REMOTE_PATH%/}/startup_scripts"
  fi
}

cleanup_ssh_session() {
  if [ -n "${SSH_CONTROL_PATH}" ] && [ -S "${SSH_CONTROL_PATH}" ]; then
    ssh "${SSH_BASE_OPTS[@]}" -O exit "${DEV_SPEC}" 2>/dev/null || true
  fi
  [ -n "${SSH_CTRL_DIR}" ] && [ -d "${SSH_CTRL_DIR}" ] && rm -rf "${SSH_CTRL_DIR}"
}

setup_ssh_session() {
  SSH_CTRL_DIR="$(mktemp -d)"
  chmod 700 "${SSH_CTRL_DIR}"
  SSH_CONTROL_PATH="${SSH_CTRL_DIR}/sock"
  SSH_BASE_OPTS=(
    -o StrictHostKeyChecking=accept-new
    -o "ControlPath=${SSH_CONTROL_PATH}"
  )
  trap cleanup_ssh_session EXIT

  local master_opts=(
    "${SSH_BASE_OPTS[@]}"
    -o ControlMaster=yes
    -o ControlPersist=120
    -o ConnectTimeout=20
  )

  if ssh -o BatchMode=yes -o ConnectTimeout=5 "${DEV_SPEC}" 'true' 2>/dev/null; then
    echo "SSH to ${DEV_SPEC} (key-based auth)…"
    ssh "${master_opts[@]}" "${DEV_SPEC}" 'echo dev-pc-ok' >/dev/null
    return 0
  fi

  if [ -n "${DEV_PASSWORD}" ]; then
    ensure_sshpass
    echo "SSH to ${DEV_SPEC} (password via sshpass)…"
    SSHPASS="${DEV_PASSWORD}" sshpass -e ssh "${master_opts[@]}" "${DEV_SPEC}" 'echo dev-pc-ok' >/dev/null \
      || die "SSH to ${DEV_SPEC} failed"
    return 0
  fi

  if command -v sshpass >/dev/null 2>&1; then
    local _pw=""
    read -rsp "SSH password for ${DEV_SPEC} (used once): " _pw
    echo ""
    DEV_PASSWORD="${_pw}"
    SSHPASS="${DEV_PASSWORD}" sshpass -e ssh "${master_opts[@]}" "${DEV_SPEC}" 'echo dev-pc-ok' >/dev/null \
      || die "SSH to ${DEV_SPEC} failed"
    return 0
  fi

  echo "Connecting to ${DEV_SPEC} — enter SSH password once if prompted…"
  ssh "${master_opts[@]}" "${DEV_SPEC}" 'echo dev-pc-ok' >/dev/null \
    || die "SSH to ${DEV_SPEC} failed"
}

_rsync_to() {
  local src="$1"
  local dest="$2"
  if [ -n "${DEV_PASSWORD}" ]; then
    SSHPASS="${DEV_PASSWORD}" rsync -avz "${RSYNC_EXCLUDES[@]}" \
      -e "sshpass -e ssh ${SSH_BASE_OPTS[*]}" \
      "${src}" "${dest}"
  else
    rsync -avz "${RSYNC_EXCLUDES[@]}" \
      -e "ssh ${SSH_BASE_OPTS[*]}" \
      "${src}" "${dest}"
  fi
}

case "${1:-}" in
  -h|--help|help|"") usage; exit 0 ;;
esac

DEV_SPEC="$1"
shift

while [ $# -gt 0 ]; do
  case "$1" in
    --remote-path=*) REMOTE_PATH="${1#--remote-path=}"; shift ;;
    --remote-startup-scripts=*) REMOTE_STARTUP_SCRIPTS="${1#--remote-startup-scripts=}"; shift ;;
    -*) die "unknown option: $1 (try --help)" ;;
    *) die "unexpected argument: $1" ;;
  esac
done

parse_dev_spec "${DEV_SPEC}"
resolve_remote_paths
setup_ssh_session

[ -d "${STARTUP_SCRIPTS_SRC}" ] || die "missing ${STARTUP_SCRIPTS_SRC}"

echo "Creating remote directories…"
if [ -n "${DEV_PASSWORD}" ]; then
  SSHPASS="${DEV_PASSWORD}" sshpass -e ssh "${SSH_BASE_OPTS[@]}" "${DEV_SPEC}" \
    "mkdir -p '${REMOTE_DEPLOY_PI5}' '${REMOTE_STARTUP_SCRIPTS}'"
else
  ssh "${SSH_BASE_OPTS[@]}" "${DEV_SPEC}" \
    "mkdir -p '${REMOTE_DEPLOY_PI5}' '${REMOTE_STARTUP_SCRIPTS}'"
fi

echo "Pushing deploy_pi5: ${SOURCE_DIR}/ → ${DEV_SPEC}:${REMOTE_DEPLOY_PI5}/"
_rsync_to "${SOURCE_DIR}/" "${DEV_SPEC}:${REMOTE_DEPLOY_PI5}/"

echo "Pushing startup_scripts: ${STARTUP_SCRIPTS_SRC}/ → ${DEV_SPEC}:${REMOTE_STARTUP_SCRIPTS}/"
_rsync_to "${STARTUP_SCRIPTS_SRC}/" "${DEV_SPEC}:${REMOTE_STARTUP_SCRIPTS}/"

cleanup_ssh_session
trap - EXIT
echo "Done."
