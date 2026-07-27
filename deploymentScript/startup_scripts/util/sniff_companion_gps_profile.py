#!/usr/bin/env python3
"""Sniff companion rover: F9P (ACM) then LC29H EA/DA.

Prints one token on stdout:
  usb_f9p|/dev/ttyACMN | usb_ea | usb_da | uart_da | none
"""

from __future__ import annotations

import argparse
import glob
import sys
import time

UBX_SYNC = b"\xb5\x62"


def _nmea_packet(body: str) -> bytes:
    cs = 0
    for c in body:
        cs ^= ord(c)
    return f"${body}*{cs:02X}\r\n".encode("ascii")


def _ubx_checksum(payload: bytes) -> tuple[int, int]:
    ck_a = 0
    ck_b = 0
    for byte in payload:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def _ubx_frame(cls: int, msg_id: int, payload: bytes = b"") -> bytes:
    length = len(payload)
    header = bytes([cls, msg_id, length & 0xFF, (length >> 8) & 0xFF])
    body = header + payload
    ck_a, ck_b = _ubx_checksum(body)
    return UBX_SYNC + body + bytes([ck_a, ck_b])


def _parse_ubx_frames(buf: bytes) -> list[tuple[int, int]]:
    """Return list of (cls, msg_id) for complete frames in buf."""
    out: list[tuple[int, int]] = []
    i = 0
    while True:
        j = buf.find(UBX_SYNC, i)
        if j < 0 or j + 6 > len(buf):
            break
        cls = buf[j + 2]
        msg_id = buf[j + 3]
        length = buf[j + 4] | (buf[j + 5] << 8)
        end = j + 6 + length + 2
        if end > len(buf):
            break
        out.append((cls, msg_id))
        i = end
    return out


def _probe_f9p(port: str, baud: int = 115200, timeout_s: float = 1.2) -> bool:
    try:
        import serial
    except ImportError:
        print("sniff_companion_gps_profile: pyserial required", file=sys.stderr)
        return False
    try:
        with serial.Serial(port, baud, timeout=0.1) as ser:
            ser.rtscts = False
            ser.dsrdtr = False
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
            ser.write(_ubx_frame(0x0A, 0x04))  # MON-VER poll
            deadline = time.monotonic() + timeout_s
            buf = b""
            while time.monotonic() < deadline:
                chunk = ser.read(ser.in_waiting or 256)
                if not chunk:
                    time.sleep(0.05)
                    continue
                buf += chunk
                for cls, msg_id in _parse_ubx_frames(buf):
                    if cls == 0x0A and msg_id == 0x04:
                        return True
    except Exception as exc:
        print(f"sniff_companion_gps_profile: F9P probe {port} failed: {exc}", file=sys.stderr)
        return False
    return False


def _profile_from_text(raw: bytes) -> str:
    flat = raw.decode("ascii", "replace").upper().replace(" ", "")
    if "LC29HEA" in flat:
        return "ea"
    if "LC29HDA" in flat:
        return "da"
    return "unknown"


def _query_verno(port: str, baud: int, listen_s: float = 0.7) -> str:
    try:
        import serial
    except ImportError:
        print("sniff_companion_gps_profile: pyserial required", file=sys.stderr)
        return "unknown"
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0.12,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        ser.rtscts = False
        ser.dsrdtr = False
    except Exception as exc:
        print(f"sniff_companion_gps_profile: open {port}@{baud} failed: {exc}", file=sys.stderr)
        return "unknown"
    buf = bytearray()
    try:
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        ser.write(_nmea_packet("PQTMVERNO"))
        ser.flush()
        t0 = time.monotonic()
        while time.monotonic() - t0 < listen_s:
            try:
                chunk = ser.read(2048)
            except Exception:
                break
            if chunk:
                buf.extend(chunk)
            else:
                time.sleep(0.02)
    finally:
        try:
            ser.close()
        except Exception:
            pass
    return _profile_from_text(bytes(buf))


def sniff_f9p(acm_glob: str = "/dev/ttyACM*") -> str | None:
    ports = sorted(glob.glob(acm_glob))
    for port in ports:
        if _probe_f9p(port):
            print(f"sniff_companion_gps_profile: {port}@115200 → F9P", file=sys.stderr)
            return port
    return None


def sniff_lc29h(usb_port: str, uart_port: str) -> str:
    for baud in (460800, 115200):
        prof = _query_verno(usb_port, baud)
        if prof == "ea":
            print(f"sniff_companion_gps_profile: {usb_port}@{baud} → EA", file=sys.stderr)
            return "usb_ea"
        if prof == "da":
            print(f"sniff_companion_gps_profile: {usb_port}@{baud} → DA", file=sys.stderr)
            return "usb_da"
    prof = _query_verno(uart_port, 115200)
    if prof == "da":
        print(f"sniff_companion_gps_profile: {uart_port}@115200 → DA", file=sys.stderr)
        return "uart_da"
    if prof == "ea":
        print(
            f"sniff_companion_gps_profile: {uart_port}@115200 → EA (unexpected on UART)",
            file=sys.stderr,
        )
        return "usb_ea"
    print("sniff_companion_gps_profile: no LC29H VERNO on USB/UART", file=sys.stderr)
    return "none"


def sniff(usb_port: str, uart_port: str, acm_glob: str = "/dev/ttyACM*") -> str:
    f9p_port = sniff_f9p(acm_glob)
    if f9p_port:
        return f"usb_f9p|{f9p_port}"
    return sniff_lc29h(usb_port, uart_port)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--usb-port", default="/dev/ttyUSB0")
    p.add_argument("--uart-port", default="/dev/ttyAMA4")
    p.add_argument("--acm-glob", default="/dev/ttyACM*")
    args = p.parse_args()
    token = sniff(args.usb_port, args.uart_port, args.acm_glob)
    print(token)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
