#!/usr/bin/env bash
# Install companion-drone systemd service (requires sudo once).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FILES_DIR="${SCRIPT_DIR}/files"

usage() {
  cat <<EOF
Usage:
  sudo $0 <drone_id>
  $0 -h|--help

Install CatSwarm companion stack to start at boot via systemd.

Arguments:
  drone_id    Required. Positive integer matching system_manager/MultiInput/multiSetup.list

Examples:
  sudo $0 3
  sudo $0 1

Writes:
  /etc/default/companion-drone       COMPANION_DRONE_ID, conda env, paths
  /usr/local/bin/run-companion-drone.sh
  /usr/local/lib/companion-conda-env.sh
  /usr/local/bin/companion-run-in-rl.sh
  /etc/systemd/system/companion-drone.service

Logs:
  journalctl -u companion-drone -f

Remove:
  sudo $(dirname "$0")/uninstall-companion-boot.sh
  $(dirname "$0")/uninstall-companion-boot.sh --help
EOF
}

validate_drone_id() {
  local id="$1"
  if ! [[ "${id}" =~ ^[0-9]+$ ]] || [ "${id}" -lt 1 ]; then
    echo "Error: drone_id must be a positive integer (got: ${id})" >&2
    echo "Run: $0 --help" >&2
    exit 1
  fi
}

write_companion_config() {
  local drone_id="$1"
  install -d /etc/default
  cat > /etc/default/companion-drone <<EOF
# CatSwarm companion drone boot configuration
# Drone index (must match system_manager/MultiInput/multiSetup.list)
COMPANION_DRONE_ID=${drone_id}

# RL tree on the Pi
RL_ROOT=/home/pi/RL

# Conda env for all companion tmux panes
COMPANION_CONDA_ROOT=/home/pi/miniconda
COMPANION_CONDA_ENV=RL

# Seconds to wait for /dev/ttyAMA* devices at boot
COMPANION_UART_WAIT_S=30
EOF
  chmod 0644 /etc/default/companion-drone
}

case "${1:-}" in
  -h|--help|help) usage; exit 0 ;;
esac

DRONE_ID="${1:-}"
if [ -z "${DRONE_ID}" ]; then
  echo "Error: drone_id is required." >&2
  echo "Run: $0 --help" >&2
  exit 1
fi
validate_drone_id "${DRONE_ID}"

if [ "$(id -u)" -ne 0 ]; then
  echo "Run: sudo $0 ${DRONE_ID}" >&2
  exit 1
fi

install -m 0755 "${FILES_DIR}/run-companion-drone.sh" /usr/local/bin/run-companion-drone.sh
install -m 0644 "${FILES_DIR}/companion-conda-env.sh" /usr/local/lib/companion-conda-env.sh
install -m 0755 "${FILES_DIR}/companion-run-in-rl.sh" /usr/local/bin/companion-run-in-rl.sh
write_companion_config "${DRONE_ID}"

cat > /etc/systemd/system/companion-drone.service <<'EOF'
[Unit]
Description=CatSwarm companion drone stack (hardware adapter + system manager + RTK)
Documentation=file:///home/pi/RL/deployscripts/README.md
After=network-online.target mavlink-server.service
Wants=network-online.target mavlink-server.service

[Service]
Type=oneshot
RemainAfterExit=yes
User=pi
Group=pi
Environment=HOME=/home/pi
EnvironmentFile=-/etc/default/companion-drone
ExecStartPre=/bin/sleep 5
ExecStart=/usr/local/bin/run-companion-drone.sh
StandardOutput=journal
StandardError=journal
SyslogIdentifier=companion-drone

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable companion-drone.service
systemctl restart companion-drone.service
echo "companion-drone.service installed and enabled (COMPANION_DRONE_ID=${DRONE_ID})"
