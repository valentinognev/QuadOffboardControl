#!/usr/bin/env bash
# On the Pi (no internet): fetch wheels via SSH from a dev PC that has internet.
#
# Usage (on rlcat3 / drone Pi):
#   ./pull-offline-packages.sh --from=user@192.168.0.10 matplotlib
#   ./pull-offline-packages.sh --from=192.168.0.10 matplotlib --install
#
# The dev PC must have: SSH server, python3 + pip, outbound internet.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=offline-wheels-common.sh
source "${SCRIPT_DIR}/offline-wheels-common.sh"

WHEELS_DIR="${OFFLINE_WHEELS_DIR:-${SCRIPT_DIR}/offline-wheels}"
DEV_SPEC=""
DEV_USER="${DEV_PC_USER:-}"
DEV_HOST=""
PACKAGES=()
DO_INSTALL=0
DEV_PC_PASSWORD="${DEV_PC_PASSWORD:-}"

# Staging directory on the dev PC (created automatically).
REMOTE_STAGING="${DEV_OFFLINE_STAGING:-deploy_pi5-offline-staging}"

SSH_CTRL_DIR=""
SSH_CONTROL_PATH=""
SSH_BASE_OPTS=()

usage() {
  cat <<EOF
Usage:
  $0 --from=<user@host|host> <package> [package ...] [--install]
  $0 --help

Run on the drone Pi (no internet). Downloads Python wheels on a dev PC that
has internet, copies them over SSH, and optionally installs into conda env RL.

Examples:
  $0 --from=valentin@192.168.0.10 matplotlib --install
  $0 --from=192.168.0.10 matplotlib plotly

SSH authentication (pick one):
  - SSH key (recommended):  ssh-copy-id user@dev-pc
  - Password once: script reuses one SSH session (you should only type it once)
  - Or set DEV_PC_PASSWORD (uses sshpass if installed)

Requirements on dev PC:
  - SSH server (port 22)
  - python3 and pip with internet access

Wheels on this Pi: ${WHEELS_DIR}

Alternative (run on dev PC instead):
  ~/deploy_pi5/push-offline-packages.sh pi@rlcat3 matplotlib --install

Or full deploy on Pi (repos + pip via dev PC):
  sudo ~/deploy_pi5/deploy_pi5_companion.sh --from-dev=user@192.168.0.39
  sudo ~/deploy_pi5/deploy_pi5_companion.sh --phase=python --pip-via-dev=user@192.168.0.39

Environment:
  DEV_PC_USER              Default SSH user if --from=IP only
  DEV_PC_PASSWORD          SSH password (optional; uses sshpass)
  DEV_OFFLINE_STAGING      Remote staging dir under ~ (default: ${REMOTE_STAGING})
  OFFLINE_WHEELS_DIR       Local wheels dir on this Pi
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
    if [ -z "${DEV_USER}" ]; then
      die "use --from=user@host or set DEV_PC_USER"
    fi
  fi
  [ -n "${DEV_HOST}" ] || die "dev PC host required"
  DEV_SPEC="${DEV_USER}@${DEV_HOST}"
}

cleanup_ssh_session() {
  if [ -n "${SSH_CONTROL_PATH}" ] && [ -S "${SSH_CONTROL_PATH}" ]; then
    ssh "${SSH_BASE_OPTS[@]}" -O exit "${DEV_SPEC}" 2>/dev/null || true
  fi
  if [ -n "${SSH_CTRL_DIR}" ] && [ -d "${SSH_CTRL_DIR}" ]; then
    rm -rf "${SSH_CTRL_DIR}"
  fi
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
    -o ControlPersist=300
    -o ConnectTimeout=20
  )

  if ssh -o BatchMode=yes -o ConnectTimeout=5 "${DEV_SPEC}" 'true' 2>/dev/null; then
    echo "SSH to ${DEV_SPEC} (key-based auth)…"
    ssh "${master_opts[@]}" "${DEV_SPEC}" 'echo dev-pc-ok' >/dev/null
    return 0
  fi

  if [ -n "${DEV_PC_PASSWORD}" ]; then
    ensure_sshpass
    echo "SSH to ${DEV_SPEC} (password via sshpass)…"
    SSHPASS="${DEV_PC_PASSWORD}" sshpass -e ssh "${master_opts[@]}" "${DEV_SPEC}" 'echo dev-pc-ok' >/dev/null \
      || die "SSH to ${DEV_SPEC} failed (check password)"
    return 0
  fi

  if command -v sshpass >/dev/null 2>&1; then
    local _pw=""
    read -rsp "SSH password for ${DEV_SPEC} (used once for all steps): " _pw
    echo ""
    DEV_PC_PASSWORD="${_pw}"
    SSHPASS="${DEV_PC_PASSWORD}" sshpass -e ssh "${master_opts[@]}" "${DEV_SPEC}" 'echo dev-pc-ok' >/dev/null \
      || die "SSH to ${DEV_SPEC} failed (check password)"
    return 0
  fi

  echo "Connecting to ${DEV_SPEC} — enter SSH password once if prompted…"
  ssh "${master_opts[@]}" "${DEV_SPEC}" 'echo dev-pc-ok' >/dev/null \
    || die "SSH to ${DEV_SPEC} failed"
}

_ssh() {
  if [ -n "${DEV_PC_PASSWORD}" ]; then
    SSHPASS="${DEV_PC_PASSWORD}" sshpass -e ssh "${SSH_BASE_OPTS[@]}" "$@"
  else
    ssh "${SSH_BASE_OPTS[@]}" "$@"
  fi
}

_scp() {
  if [ -n "${DEV_PC_PASSWORD}" ]; then
    SSHPASS="${DEV_PC_PASSWORD}" sshpass -e scp "${SSH_BASE_OPTS[@]}" "$@"
  else
    scp "${SSH_BASE_OPTS[@]}" "$@"
  fi
}

_rsync_from_dev() {
  local src="$1" dest="$2"
  if [ -n "${DEV_PC_PASSWORD}" ]; then
    SSHPASS="${DEV_PC_PASSWORD}" rsync -avz --delete \
      -e "sshpass -e ssh ${SSH_BASE_OPTS[*]}" \
      "${src}" "${dest}"
  else
    rsync -avz --delete \
      -e "ssh ${SSH_BASE_OPTS[*]}" \
      "${src}" "${dest}"
  fi
}

remote_download_wheels() {
  local remote_home remote_staging remote_wheels remote_script

  echo "Fetching wheels via ${DEV_SPEC} (dev PC downloads, this Pi pulls)…"

  remote_home="$(_ssh "${DEV_SPEC}" 'printf %s "$HOME"')"
  remote_staging="${remote_home}/${REMOTE_STAGING}"
  remote_wheels="${remote_staging}/offline-wheels"
  remote_script="${remote_staging}/offline-wheels-common.sh"

  _ssh "${DEV_SPEC}" "mkdir -p '${remote_staging}'"

  _scp "${SCRIPT_DIR}/offline-wheels-common.sh" \
    "${DEV_SPEC}:${remote_script}"

  # Quote each requirement so remote shell does not treat >= as redirection.
  local remote_pkgs="" p
  for p in "${PACKAGES[@]}"; do
    remote_pkgs+=" $(printf '%q' "${p}")"
  done

  _ssh "${DEV_SPEC}" \
    "chmod +x '${remote_script}' && OFFLINE_WHEELS_DIR='${remote_wheels}' '${remote_script}' download${remote_pkgs}"

  mkdir -p "${WHEELS_DIR}"
  _rsync_from_dev "${DEV_SPEC}:${remote_wheels}/" "${WHEELS_DIR}/"
}

case "${1:-}" in
  -h|--help|help|"") usage; exit 0 ;;
esac

while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help|help) usage; exit 0 ;;
    --from=*) parse_dev_spec "${1#--from=}"; shift ;;
    --from)
      [ $# -ge 2 ] || die "--from requires host"
      parse_dev_spec "$2"
      shift 2
      ;;
    --install) DO_INSTALL=1; shift ;;
    -*) die "unknown option: $1 (try --help)" ;;
    *) PACKAGES+=("$1"); shift ;;
  esac
done

if [ -z "${DEV_HOST}" ]; then
  echo "Error: --from=<dev-pc-ip-or-host> is required" >&2
  usage >&2
  exit 1
fi

if [ "${#PACKAGES[@]}" -eq 0 ]; then
  PACKAGES=(matplotlib)
fi

DEV_SPEC="${DEV_USER}@${DEV_HOST}"

setup_ssh_session
remote_download_wheels
cleanup_ssh_session
trap - EXIT

if [ "${DO_INSTALL}" -eq 1 ]; then
  exec "${SCRIPT_DIR}/offline-install-packages.sh" "${PACKAGES[@]}"
else
  echo ""
  echo "Wheels ready in ${WHEELS_DIR}"
  echo "Install:"
  echo "  ~/deploy_pi5/offline-install-packages.sh ${PACKAGES[*]}"
  echo ""
  echo "Or re-run with --install:"
  echo "  $0 --from=${DEV_SPEC} ${PACKAGES[*]} --install"
fi
