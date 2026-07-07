#!/usr/bin/env bash
# Download Python wheels on a machine with internet, push to a drone Pi, optionally install.
#
# Usage (on dev PC):
#   ./push-offline-packages.sh pi@rlcat3 matplotlib
#   ./push-offline-packages.sh 192.168.0.141 matplotlib plotly
#   ./push-offline-packages.sh pi@rlcat3 matplotlib --install
#
# On the Pi (no internet), use pull-offline-packages.sh instead:
#   ./pull-offline-packages.sh --from=user@192.168.0.10 matplotlib --install
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=offline-wheels-common.sh
source "${SCRIPT_DIR}/offline-wheels-common.sh"

WHEELS_DIR="${OFFLINE_WHEELS_DIR:-${SCRIPT_DIR}/offline-wheels}"

REMOTE_USER="${PUSH_TO_USER:-pi}"
REMOTE_HOST=""
REMOTE_SPEC=""
PACKAGES=()
DO_INSTALL=0

usage() {
  cat <<EOF
Usage:
  $0 <pi-host|user@pi-host> <package> [package ...] [--install]
  $0 --help

Run on dev PC with internet. Downloads wheels for Pi (${PI_WHEEL_PLATFORM},
Python ${PI_WHEEL_PYTHON}), copies to the drone Pi, optionally installs.

Examples:
  $0 pi@rlcat3 matplotlib --install
  $0 192.168.0.141 matplotlib plotly --install

On the Pi (no internet) — pull from this dev PC instead:
  ~/deploy_pi5/pull-offline-packages.sh --from=\$(whoami)@\$(hostname -I | awk '{print \$1}') matplotlib --install

Wheels saved under: ${WHEELS_DIR}
On Pi, manual install: ~/deploy_pi5/offline-install-packages.sh matplotlib

Environment:
  OFFLINE_WHEELS_DIR       Local download directory
  PI_WHEEL_PLATFORM        Wheel platform tag (default: ${PI_WHEEL_PLATFORM})
  PI_WHEEL_PYTHON          Python version for wheels (default: ${PI_WHEEL_PYTHON})
EOF
}

parse_remote() {
  local spec="$1"
  if [[ "${spec}" == *@* ]]; then
    REMOTE_USER="${spec%%@*}"
    REMOTE_HOST="${spec#*@}"
  else
    REMOTE_HOST="${spec}"
  fi
  [ -n "${REMOTE_HOST}" ] || { echo "Error: remote host required" >&2; exit 1; }
}

push_wheels() {
  echo "Pushing wheels to ${REMOTE_USER}@${REMOTE_HOST}:~/deploy_pi5/offline-wheels/ …"
  ssh "${REMOTE_USER}@${REMOTE_HOST}" "mkdir -p ~/deploy_pi5/offline-wheels"
  rsync -avz --delete "${WHEELS_DIR}/" \
    "${REMOTE_USER}@${REMOTE_HOST}:~/deploy_pi5/offline-wheels/"
  rsync -avz \
    "${SCRIPT_DIR}/offline-install-packages.sh" \
    "${SCRIPT_DIR}/pull-offline-packages.sh" \
    "${SCRIPT_DIR}/offline-wheels-common.sh" \
    "${REMOTE_USER}@${REMOTE_HOST}:~/deploy_pi5/"
}

remote_install() {
  echo "Installing on ${REMOTE_USER}@${REMOTE_HOST}…"
  ssh "${REMOTE_USER}@${REMOTE_HOST}" \
    "chmod +x ~/deploy_pi5/offline-install-packages.sh && ~/deploy_pi5/offline-install-packages.sh ${PACKAGES[*]}"
}

case "${1:-}" in
  -h|--help|help|"") usage; exit 0 ;;
esac

REMOTE_SPEC="$1"
shift

while [ $# -gt 0 ]; do
  case "$1" in
    --install) DO_INSTALL=1; shift ;;
    -h|--help|help) usage; exit 0 ;;
    -*) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
    *) PACKAGES+=("$1"); shift ;;
  esac
done

if [ "${#PACKAGES[@]}" -eq 0 ]; then
  PACKAGES=(matplotlib)
fi

parse_remote "${REMOTE_SPEC}"
offline_download_wheels "${WHEELS_DIR}" "${PACKAGES[@]}"
push_wheels

if [ "${DO_INSTALL}" -eq 1 ]; then
  remote_install
else
  echo ""
  echo "Wheels pushed. On the Pi, run:"
  echo "  ~/deploy_pi5/offline-install-packages.sh ${PACKAGES[*]}"
  echo ""
  echo "Or on the Pi (pull mode):"
  echo "  ~/deploy_pi5/pull-offline-packages.sh --from=USER@\$(hostname -I|awk '{print \$1}') ${PACKAGES[*]} --install"
fi
