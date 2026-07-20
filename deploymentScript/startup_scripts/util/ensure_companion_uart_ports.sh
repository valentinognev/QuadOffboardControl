#!/usr/bin/env bash
# Migrate + validate companion UART roles (NMEA→PX4 on UART0) during deploy/update/boot.
#
# Usage:
#   ensure_companion_uart_ports.sh           # migrate + warn if devices missing
#   COMPANION_VALIDATE_UART_STRICT=1 ...    # exit 1 if required tty nodes missing
#
# Safe to run before reboot (migrates ~/.config/companion-gps even if overlays
# have not created /dev/ttyAMA* yet — missing devices are warnings unless STRICT).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=companion_gps_module.sh
source "${SCRIPT_DIR}/companion_gps_module.sh"

echo "ensure_companion_uart_ports: state=$(companion_gps_state_file)" >&2

# Load any saved preference, then migrate/validate/persist.
if companion_gps_load_saved; then
    echo "ensure_companion_uart_ports: loaded saved companion-gps" >&2
else
    echo "ensure_companion_uart_ports: no saved companion-gps yet (using defaults)" >&2
    : "${COMPANION_GPS_MODULE:=ea}"
fi

companion_gps_ensure_ports
rc=$?

echo "ensure_companion_uart_ports: COMPANION_PX4_GPS_PORT=${COMPANION_PX4_GPS_PORT}" >&2
echo "ensure_companion_uart_ports: ROVER_PORT=${ROVER_PORT:-} ROVER_PORT_UART=${ROVER_PORT_UART:-}" >&2
exit "${rc}"
