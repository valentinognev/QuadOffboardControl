#!/usr/bin/env bash
# Push ~/deploy_pi5 from the Pi to a dev PC (e.g. general_infrastructure/deploymentScript).
#
# Usage (on Pi):
#   ./push-deploy-scripts-to-dev.sh valentin@192.168.0.39
#   ./push-deploy-scripts-to-dev.sh valentin@192.168.0.39 --remote-path=/home/valentin/RL/CatSwarm/general_infrastructure/deploymentScript
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="${DEPLOY_SOURCE_DIR:-${SCRIPT_DIR}}"

DEV_SPEC=""
DEV_USER="${DEV_PC_USER:-}"
DEV_HOST=""
DEV_PASSWORD="${DEV_PC_PASSWORD:-}"
REMOTE_PATH="${DEPLOY_REMOTE_PATH:-/home/valentin/RL/CatSwarm/general_infrastructure/deploymentScript}"

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
  $0 <user@host|host> [--remote-path=PATH]
  $0 --help

Push deploy scripts from this Pi to a dev PC (opposite of --from-dev rsync).

Examples:
  $0 valentin@192.168.0.39
  $0 192.168.0.39 --remote-path=/home/valentin/RL/CatSwarm/general_infrastructure/deploymentScript

Source:  ${SOURCE_DIR}/
Default remote: ${REMOTE_PATH}/

Environment:
  DEV_PC_PASSWORD   SSH password (uses sshpass if set)
  DEPLOY_SOURCE_DIR Override source directory
  DEPLOY_REMOTE_PATH Override remote destination directory
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

_rsync() {
  if [ -n "${DEV_PASSWORD}" ]; then
    SSHPASS="${DEV_PASSWORD}" rsync -avz "${RSYNC_EXCLUDES[@]}" \
      -e "sshpass -e ssh ${SSH_BASE_OPTS[*]}" \
      "${SOURCE_DIR}/" "${DEV_SPEC}:${REMOTE_PATH}/"
  else
    rsync -avz "${RSYNC_EXCLUDES[@]}" \
      -e "ssh ${SSH_BASE_OPTS[*]}" \
      "${SOURCE_DIR}/" "${DEV_SPEC}:${REMOTE_PATH}/"
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
    -*) die "unknown option: $1 (try --help)" ;;
    *) die "unexpected argument: $1" ;;
  esac
done

parse_dev_spec "${DEV_SPEC}"
setup_ssh_session

echo "Creating remote directory ${REMOTE_PATH}…"
if [ -n "${DEV_PASSWORD}" ]; then
  SSHPASS="${DEV_PASSWORD}" sshpass -e ssh "${SSH_BASE_OPTS[@]}" "${DEV_SPEC}" "mkdir -p '${REMOTE_PATH}'"
else
  ssh "${SSH_BASE_OPTS[@]}" "${DEV_SPEC}" "mkdir -p '${REMOTE_PATH}'"
fi

echo "Pushing ${SOURCE_DIR}/ → ${DEV_SPEC}:${REMOTE_PATH}/"
_rsync

cleanup_ssh_session
trap - EXIT
echo "Done."
