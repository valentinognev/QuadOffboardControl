#!/usr/bin/env bash
# Ensure mavlink-server listens on UDP 14580 for GPS_RTCM_DATA / companion inject.
# No-op if already present. Requires passwordless sudo or interactive sudo.
#
# Usage:
#   ensure_mavlink_rtcm_udp.sh
#   source … && companion_ensure_mavlink_rtcm_udp

companion_ensure_mavlink_rtcm_udp() {
  local cfg="${COMPANION_MAVLINK_SERVER_CONF:-/etc/mavlink-server/mavlink-server.conf}"
  local port="${COMPANION_MAVLINK_RTCM_UDP_PORT:-14580}"
  local need=0
  local tmp

  if [[ ! -f "${cfg}" ]]; then
    echo "companion_ensure_mavlink_rtcm_udp: missing ${cfg}" >&2
    return 1
  fi

  # Live process must expose udpserver://…:port
  if ! pgrep -a '[m]avlink-server' 2>/dev/null | grep -q "udpserver://[^ ]*:${port}"; then
    need=1
  fi
  # Config must have [[udp_server]] with address + matching port (parser requires address).
  if ! awk -v p="${port}" '
    BEGIN { ok=0; inu=0; addr=""; pr="" }
    /^\[\[udp_server\]\]/ { inu=1; addr=""; pr=""; next }
    inu && /^\[\[/ {
      if (addr != "" && pr == p) ok=1
      inu=0
    }
    inu && $1 ~ /^address/ { gsub(/[" ]/, "", $3); addr=$3 }
    inu && $1 ~ /^port/ { gsub(/[" ]/, "", $3); pr=$3 }
    END {
      if (inu && addr != "" && pr == p) ok=1
      exit(ok ? 0 : 1)
    }
  ' "${cfg}"; then
    need=1
  fi

  if [[ "${need}" -eq 0 ]]; then
    echo "companion_ensure_mavlink_rtcm_udp: udpserver :${port} OK" >&2
    return 0
  fi

  echo "companion_ensure_mavlink_rtcm_udp: adding [[udp_server]] 0.0.0.0:${port} to ${cfg}" >&2
  tmp="$(mktemp)"
  # Drop existing udp_server tables, append a correct one.
  awk '
    BEGIN { skip=0 }
    /^\[\[udp_server\]\]/ { skip=1; next }
    skip && /^\[\[/ { skip=0 }
    skip && /^\[/ && $0 !~ /^\[\[/ { skip=0 }
    skip { next }
    { print }
  ' "${cfg}" > "${tmp}"
  # strip trailing blank lines then append
  sed -i -e :a -e '/^\n*$/{$d;N;ba' -e '}' "${tmp}" 2>/dev/null || true
  printf '\n# Companion injects (GPS_RTCM_DATA / RTCM→MAVLink)\n[[udp_server]]\naddress = "0.0.0.0"\nport = %s\n' "${port}" >> "${tmp}"

  if ! sudo -n cp "${tmp}" "${cfg}" 2>/dev/null; then
    if ! sudo cp "${tmp}" "${cfg}"; then
      rm -f "${tmp}"
      echo "companion_ensure_mavlink_rtcm_udp: sudo failed writing ${cfg}" >&2
      return 1
    fi
  fi
  rm -f "${tmp}"
  if ! sudo -n systemctl restart mavlink-server 2>/dev/null; then
    sudo systemctl restart mavlink-server || {
      echo "companion_ensure_mavlink_rtcm_udp: restart mavlink-server failed" >&2
      return 1
    }
  fi
  sleep 1
  if pgrep -a '[m]avlink-server' 2>/dev/null | grep -q "udpserver://[^ ]*:${port}"; then
    echo "companion_ensure_mavlink_rtcm_udp: mavlink-server now has udpserver :${port}" >&2
    return 0
  fi
  echo "companion_ensure_mavlink_rtcm_udp: WARN process still missing udpserver :${port}" >&2
  return 1
}

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  companion_ensure_mavlink_rtcm_udp "$@"
fi
