#!/usr/bin/env bash
# Remove companion stack from boot (cron @reboot and optional systemd service).
set -euo pipefail

CRONTAB_USER="${CRONTAB_USER:-pi}"

usage() {
  cat <<EOF
Usage:
  $0
  sudo $0
  $0 -h|--help

Disable CatSwarm companion stack from starting at boot.

Without sudo:
  Removes @reboot crontab lines for run-companion-drone.sh and
  start_companion_drone_tmux.sh for the current user.

With sudo:
  Also disables and removes companion-drone.service, run-companion-drone.sh,
  and the companion @reboot crontab for user ${CRONTAB_USER}.

Environment:
  CRONTAB_USER   User whose crontab to edit when run as root (default: pi)

Examples:
  $0
  sudo $0
  $0 --help

Does not stop a running tmux session or delete /etc/default/companion-drone.

Re-install:
  sudo $(dirname "$0")/install-companion-boot.sh <drone_id>
EOF
}

remove_crontab() {
  local user="$1"
  local existing filtered

  if ! existing="$(crontab -u "${user}" -l 2>/dev/null)"; then
    echo "No crontab for ${user}"
    return 0
  fi

  filtered="$(printf '%s\n' "${existing}" \
    | grep -v 'run-companion-drone\.sh' \
    | grep -v 'start_companion_drone_tmux\.sh' \
    || true)"

  if [ "${existing}" = "${filtered}" ]; then
    echo "No companion @reboot crontab line for ${user}"
    return 0
  fi

  if [ -z "${filtered}" ]; then
    crontab -u "${user}" -r
    echo "Removed companion @reboot crontab (crontab was only companion entries)"
  else
    printf '%s\n' "${filtered}" | crontab -u "${user}" -
    echo "Removed companion @reboot crontab line(s) for ${user}"
  fi
}

remove_systemd() {
  if [ -f /etc/systemd/system/companion-drone.service ]; then
    systemctl disable --now companion-drone.service 2>/dev/null || true
    rm -f /etc/systemd/system/companion-drone.service
    systemctl daemon-reload
    echo "Removed companion-drone.service"
  else
    echo "companion-drone.service not installed"
  fi

  if [ -f /usr/local/bin/run-companion-drone.sh ]; then
    rm -f /usr/local/bin/run-companion-drone.sh
    echo "Removed /usr/local/bin/run-companion-drone.sh"
  fi

  if [ -f /usr/local/lib/companion-conda-env.sh ]; then
    rm -f /usr/local/lib/companion-conda-env.sh
    echo "Removed /usr/local/lib/companion-conda-env.sh"
  fi

  if [ -f /usr/local/bin/companion-run-in-rl.sh ]; then
    rm -f /usr/local/bin/companion-run-in-rl.sh
    echo "Removed /usr/local/bin/companion-run-in-rl.sh"
  fi
}

case "${1:-}" in
  -h|--help|help) usage; exit 0 ;;
  "")
    ;;
  *)
    echo "Error: unknown argument: $1" >&2
    echo "Run: $0 --help" >&2
    exit 1
    ;;
esac

if [ "$(id -u)" -eq 0 ]; then
  remove_crontab "${CRONTAB_USER}"
  remove_systemd
  echo "Companion boot startup disabled."
else
  remove_crontab "$(whoami)"
  echo "Companion @reboot crontab removed."
  echo "Run with sudo to also remove systemd service: sudo $0"
fi
