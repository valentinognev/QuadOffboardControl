#!/usr/bin/env bash
#
# Interactive mavlink-server configuration for CatSwarm Pi 5 companion.
# Writes /etc/mavlink-server/mavlink-server.conf and optionally restarts the service.
#
# Usage:
#   sudo ./mavlink-server-configuration.sh
#   sudo ./mavlink-server-configuration.sh --fleet-preset    # Pi 5 defaults, no prompts
#   sudo ./mavlink-server-configuration.sh --no-restart
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="/etc/mavlink-server"
CONFIG_FILE="${CONFIG_DIR}/mavlink-server.conf"
TEMPLATE="${SCRIPT_DIR}/files/mavlink-server.conf"
SERVICE_NAME="mavlink-server"

FLEET_PRESET=0
NO_RESTART=0

# Pi 5 companion defaults (PX4 on UART3, bridges on UDP 14540, QGC on TCP 5760,
# companion inject udp_server on 14580)
DEFAULT_SERIAL_DEVICE="/dev/ttyAMA3"
DEFAULT_SERIAL_BAUD="921600"
DEFAULT_UDP_CLIENT="127.0.0.1:14540"
DEFAULT_TCP_SERVER="0.0.0.0:5760"
DEFAULT_UDP_SERVER_PORT="14580"
DEFAULT_WEB_SERVER="0.0.0.0:8080"
DEFAULT_MAVLINK_SYSTEM_ID="1"
DEFAULT_MAVLINK_COMPONENT_ID="191"
DEFAULT_MAVLINK_HEARTBEAT_HZ="1"

log() { echo "[mavlink-config] $*"; }
die() { echo "[mavlink-config] ERROR: $*" >&2; exit 1; }

usage() {
  cat <<EOF
Usage: sudo $0 [options]

Interactive setup for mavlink-server on Raspberry Pi companion computers.

Options:
  --fleet-preset     Write Pi 5 fleet defaults without prompts
  --no-restart       Do not restart ${SERVICE_NAME}.service after writing config
  -h, --help, help   Show this help

Fleet defaults (--fleet-preset):
  Serial:     ${DEFAULT_SERIAL_DEVICE} @ ${DEFAULT_SERIAL_BAUD}
  UDP client: ${DEFAULT_UDP_CLIENT}  (hardware_adapter bridges)
  TCP server: ${DEFAULT_TCP_SERVER}  (QGroundControl)
  UDP server: 0.0.0.0:${DEFAULT_UDP_SERVER_PORT}  (companion inject / RTCM→MAVLink)
  Web UI:     ${DEFAULT_WEB_SERVER}

Config file: ${CONFIG_FILE}
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --fleet-preset) FLEET_PRESET=1; shift ;;
    --no-restart) NO_RESTART=1; shift ;;
    -h|--help|help) usage; exit 0 ;;
    *) die "unknown option: $1 (try --help)" ;;
  esac
done

if [ "$(id -u)" -ne 0 ]; then
  exec sudo -- "$0" "$@"
fi

prompt() {
  local varname="$1" prompt="$2" default="$3"
  local _val=""
  if [ "${FLEET_PRESET}" -eq 1 ]; then
    printf -v "$varname" '%s' "$default"
    return 0
  fi
  if ! [[ -t 0 ]]; then
    die "interactive terminal required (stdin is not a TTY). Use: sudo ${SCRIPT_DIR}/$(basename "$0") --fleet-preset"
  fi
  read -rp "${prompt} [${default}]: " _val || true
  printf -v "$varname" '%s' "${_val:-$default}"
}

_get_toml_kv() {
  local key="$1" file="$2" line
  line="$(grep -E "^[[:space:]]*${key}[[:space:]]*=" "${file}" 2>/dev/null | head -1 || true)"
  [ -n "${line}" ] || return 0
  sed -E 's/^[^=]+=[[:space:]]*"?([^"]*)"?/\1/' <<<"${line}"
}

load_existing() {
  local src="${CONFIG_FILE}"
  [ -f "${src}" ] || src="${TEMPLATE}"
  [ -f "${src}" ] || return 0

  WEB_SERVER="$(_get_toml_kv web_server "${src}")"
  MAVLINK_SYSTEM_ID="$(_get_toml_kv mavlink_system_id "${src}")"
  MAVLINK_COMPONENT_ID="$(_get_toml_kv mavlink_component_id "${src}")"
  MAVLINK_HEARTBEAT_HZ="$(_get_toml_kv mavlink_heartbeat_frequency "${src}")"

  SERIAL_DEVICE="$(awk '/\[\[serial\]\]/{f=1;next} f&&/device/{gsub(/.*= *"?|".*/,"");print;exit}' "${src}" 2>/dev/null || true)"
  SERIAL_BAUD="$(awk '/\[\[serial\]\]/{f=1;next} f&&/baudrate/{gsub(/[^0-9]/,"");print;exit}' "${src}" 2>/dev/null || true)"
  UDP_CLIENT_ADDR="$(awk '/\[\[udp_client\]\]/{f=1;next} f&&/address/{gsub(/.*= *"?|".*/,"");print;exit}' "${src}" 2>/dev/null || true)"
  UDP_CLIENT_PORT="$(awk '/\[\[udp_client\]\]/{f=1;next} f&&/port/{gsub(/[^0-9]/,"");print;exit}' "${src}" 2>/dev/null || true)"
  TCP_SERVER_ADDR="$(awk '/\[\[tcp_server\]\]/{f=1;next} f&&/address/{gsub(/.*= *"?|".*/,"");print;exit}' "${src}" 2>/dev/null || true)"
  TCP_SERVER_PORT="$(awk '/\[\[tcp_server\]\]/{f=1;next} f&&/port/{gsub(/[^0-9]/,"");print;exit}' "${src}" 2>/dev/null || true)"

  [ -n "${UDP_CLIENT_ADDR}" ] && [ -n "${UDP_CLIENT_PORT}" ] && UDP_CLIENT="${UDP_CLIENT_ADDR}:${UDP_CLIENT_PORT}"
  [ -n "${TCP_SERVER_ADDR}" ] && [ -n "${TCP_SERVER_PORT}" ] && TCP_SERVER="${TCP_SERVER_ADDR}:${TCP_SERVER_PORT}"
}

write_config() {
  local udp_addr udp_port tcp_addr tcp_port
  IFS=':' read -r udp_addr udp_port <<< "${UDP_CLIENT}"
  IFS=':' read -r tcp_addr tcp_port <<< "${TCP_SERVER}"

  mkdir -p "${CONFIG_DIR}"
  if [ -f "${CONFIG_FILE}" ]; then
    cp -a "${CONFIG_FILE}" "${CONFIG_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
    log "backup of previous config created"
  fi

  cat > "${CONFIG_FILE}" <<EOF
# Written by mavlink-server-configuration.sh ($(date '+%Y-%m-%d %H:%M:%S %z'))
web_server = "${WEB_SERVER}"
default_api_version = 1
log_path = "./logs"
udp_server_timeout = 10
mavlink_system_id = ${MAVLINK_SYSTEM_ID}
mavlink_component_id = ${MAVLINK_COMPONENT_ID}
mavlink_heartbeat_frequency = ${MAVLINK_HEARTBEAT_HZ}

[[serial]]
device = "${SERIAL_DEVICE}"
baudrate = ${SERIAL_BAUD}

[[udp_client]]
address = "${udp_addr}"
port = ${udp_port}

[[tcp_server]]
address = "${tcp_addr}"
port = ${tcp_port}

# Companion injects (e.g. GPS_INJECT_DATA / RTCM→MAVLink) listen here.
[[udp_server]]
port = ${UDP_SERVER_PORT}

# Optional alternate inject port (not enabled by default):
# [[udp_server]]
# port = 14581
EOF
  chmod 0644 "${CONFIG_FILE}"
  log "wrote ${CONFIG_FILE}"
}

restart_service() {
  if [ "${NO_RESTART}" -eq 1 ]; then
    log "skipped service restart (--no-restart)"
    return 0
  fi
  if ! systemctl list-unit-files "${SERVICE_NAME}.service" &>/dev/null; then
    log "service ${SERVICE_NAME} not installed; skip restart"
    return 0
  fi
  systemctl daemon-reload
  systemctl restart "${SERVICE_NAME}.service"
  sleep 1
  if systemctl is-active --quiet "${SERVICE_NAME}.service"; then
    log "${SERVICE_NAME}.service is active"
  else
    log "WARNING: ${SERVICE_NAME} not active — check: journalctl -u ${SERVICE_NAME} -n 40"
  fi
}

main() {
  log "starting ($(basename "$0"))"
  WEB_SERVER="${DEFAULT_WEB_SERVER}"
  MAVLINK_SYSTEM_ID="${DEFAULT_MAVLINK_SYSTEM_ID}"
  MAVLINK_COMPONENT_ID="${DEFAULT_MAVLINK_COMPONENT_ID}"
  MAVLINK_HEARTBEAT_HZ="${DEFAULT_MAVLINK_HEARTBEAT_HZ}"
  SERIAL_DEVICE="${DEFAULT_SERIAL_DEVICE}"
  SERIAL_BAUD="${DEFAULT_SERIAL_BAUD}"
  UDP_CLIENT="${DEFAULT_UDP_CLIENT}"
  TCP_SERVER="${DEFAULT_TCP_SERVER}"
  UDP_SERVER_PORT="${DEFAULT_UDP_SERVER_PORT}"

  load_existing

  WEB_SERVER="${WEB_SERVER:-${DEFAULT_WEB_SERVER}}"
  MAVLINK_SYSTEM_ID="${MAVLINK_SYSTEM_ID:-${DEFAULT_MAVLINK_SYSTEM_ID}}"
  MAVLINK_COMPONENT_ID="${MAVLINK_COMPONENT_ID:-${DEFAULT_MAVLINK_COMPONENT_ID}}"
  MAVLINK_HEARTBEAT_HZ="${MAVLINK_HEARTBEAT_HZ:-${DEFAULT_MAVLINK_HEARTBEAT_HZ}}"
  SERIAL_DEVICE="${SERIAL_DEVICE:-${DEFAULT_SERIAL_DEVICE}}"
  SERIAL_BAUD="${SERIAL_BAUD:-${DEFAULT_SERIAL_BAUD}}"
  UDP_CLIENT="${UDP_CLIENT:-${DEFAULT_UDP_CLIENT}}"
  TCP_SERVER="${TCP_SERVER:-${DEFAULT_TCP_SERVER}}"
  UDP_SERVER_PORT="${UDP_SERVER_PORT:-${DEFAULT_UDP_SERVER_PORT}}"

  echo ""
  log "=== mavlink-server configuration ==="
  if [ "${FLEET_PRESET}" -eq 1 ]; then
    log "using Pi 5 fleet preset (non-interactive)"
  else
    log "press Enter to keep [default] values"
  fi
  echo ""

  prompt WEB_SERVER "Web server (host:port)" "${WEB_SERVER}"
  prompt MAVLINK_SYSTEM_ID "MAVLink system ID" "${MAVLINK_SYSTEM_ID}"
  prompt MAVLINK_COMPONENT_ID "MAVLink component ID" "${MAVLINK_COMPONENT_ID}"
  prompt MAVLINK_HEARTBEAT_HZ "Heartbeat frequency (Hz)" "${MAVLINK_HEARTBEAT_HZ}"
  prompt SERIAL_DEVICE "PX4 serial device (Pi 5: /dev/ttyAMA3)" "${SERIAL_DEVICE}"
  prompt SERIAL_BAUD "PX4 serial baud" "${SERIAL_BAUD}"
  prompt UDP_CLIENT "UDP client for companion bridges (addr:port)" "${UDP_CLIENT}"
  prompt TCP_SERVER "TCP server for QGC (addr:port)" "${TCP_SERVER}"

  echo ""
  log "summary:"
  echo "  serial:     ${SERIAL_DEVICE} @ ${SERIAL_BAUD}"
  echo "  udp_client: ${UDP_CLIENT}"
  echo "  tcp_server: ${TCP_SERVER}"
  echo "  udp_server: port ${UDP_SERVER_PORT}"
  echo "  web_server: ${WEB_SERVER}"
  echo "  mavlink id: system=${MAVLINK_SYSTEM_ID} component=${MAVLINK_COMPONENT_ID}"
  echo ""

  if [ "${FLEET_PRESET}" -eq 0 ]; then
    if ! [[ -t 0 ]]; then
      die "interactive terminal required. Use --fleet-preset for non-interactive setup."
    fi
    read -rp "Write config and restart service? [Y/n]: " _confirm || true
    if [[ "${_confirm}" =~ ^[Nn]$ ]]; then
      log "cancelled"
      exit 0
    fi
  fi

  write_config
  restart_service
  echo ""
  log "done. Logs: journalctl -u ${SERVICE_NAME} -f"
}

main "$@"
