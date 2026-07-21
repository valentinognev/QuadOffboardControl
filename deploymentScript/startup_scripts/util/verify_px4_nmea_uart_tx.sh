#!/usr/bin/env bash
# Verify NMEA→PX4 UART0 actually toggles GPIO14 (header pin 8).
#
# Usage:
#   verify_px4_nmea_uart_tx.sh           # sample live emulate if running, else TX burst
#   COMPANION_VALIDATE_UART_STRICT=1 …  # exit 1 on failure
set -euo pipefail

STRICT="${COMPANION_VALIDATE_UART_STRICT:-0}"
PORT="${COMPANION_PX4_GPS_PORT:-/dev/ttyAMA0}"
PYTHON="${COMPANION_PYTHON:-python3}"
if [ -x "${HOME}/miniconda/envs/RL/bin/python" ]; then
  PYTHON="${HOME}/miniconda/envs/RL/bin/python"
fi

echo "verify_px4_nmea_uart_tx: port=${PORT}  probe=header pin 8 (GPIO14 TXD0)" >&2
echo "  (legacy NMEA was UART4 / GPIO12 / header pin 32 — do not probe there)" >&2

if ! command -v pinctrl >/dev/null 2>&1; then
  echo "verify_px4_nmea_uart_tx: pinctrl missing — skip" >&2
  exit 0
fi

pin="$(pinctrl get 14 2>/dev/null || true)"
if ! printf '%s' "${pin}" | grep -q 'TXD0'; then
  echo "verify_px4_nmea_uart_tx: FAIL GPIO14 not TXD0 (${pin:-none})" >&2
  echo "  reboot after uart0-pi5 overlay; remove dtparam=uart0=on" >&2
  [[ "${STRICT}" == "1" ]] && exit 1
  exit 0
fi

# Prefer non-destructive live sample if emulate already owns the port.
if pgrep -f 'emulate_gps_to_px4.py' >/dev/null 2>&1; then
  echo "verify_px4_nmea_uart_tx: sampling GPIO14 during live emulate…" >&2
  counts="$("${PYTHON}" - <<'PY'
import subprocess, time
hi = lo = 0
t0 = time.time()
while time.time() - t0 < 2.5:
    out = subprocess.check_output(["pinctrl", "get", "14"], text=True)
    if "| lo" in out:
        lo += 1
    else:
        hi += 1
print(f"{hi} {lo}")
PY
)"
  hi="${counts%% *}"
  lo="${counts##* }"
  echo "verify_px4_nmea_uart_tx: GPIO14 samples hi=${hi} lo=${lo}" >&2
  if [[ "${lo}" -lt 5 ]]; then
    echo "verify_px4_nmea_uart_tx: FAIL little/no TX activity on GPIO14" >&2
    echo "  check emulate is forwarding NMEA; probe physical pin 8, not pin 32" >&2
    [[ "${STRICT}" == "1" ]] && exit 1
    exit 0
  fi
  echo "verify_px4_nmea_uart_tx: PASS (live TX on GPIO14 / pin 8)" >&2
  exit 0
fi

# Exclusive burst test (port free).
if [[ ! -e "${PORT}" ]]; then
  echo "verify_px4_nmea_uart_tx: NOTE ${PORT} missing (reboot for overlays?)" >&2
  [[ "${STRICT}" == "1" ]] && exit 1
  exit 0
fi

echo "verify_px4_nmea_uart_tx: exclusive 0x00 burst on ${PORT}…" >&2
counts="$("${PYTHON}" - <<PY
import serial, subprocess, threading, time
port = "${PORT}"
stop = False
hi = lo = 0

def sampler():
    global hi, lo
    while not stop:
        out = subprocess.check_output(["pinctrl", "get", "14"], text=True)
        if "| lo" in out:
            lo += 1
        else:
            hi += 1

th = threading.Thread(target=sampler, daemon=True)
th.start()
ser = serial.Serial(port, 115200, timeout=0, write_timeout=2, rtscts=False, dsrdtr=False)
t0 = time.time()
while time.time() - t0 < 1.5:
    ser.write(bytes([0x00]) * 1024)
    ser.flush()
stop = True
th.join(timeout=2)
ser.close()
print(f"{hi} {lo}")
PY
)"
hi="${counts%% *}"
lo="${counts##* }"
echo "verify_px4_nmea_uart_tx: GPIO14 samples hi=${hi} lo=${lo}" >&2
if [[ "${lo}" -lt 20 ]]; then
  echo "verify_px4_nmea_uart_tx: FAIL ${PORT} did not toggle GPIO14" >&2
  [[ "${STRICT}" == "1" ]] && exit 1
  exit 0
fi
echo "verify_px4_nmea_uart_tx: PASS (burst TX on GPIO14 / pin 8)" >&2
exit 0
