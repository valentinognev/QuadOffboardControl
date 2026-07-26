#!/usr/bin/env python3
"""Sniff LC29H EA/DA on USB then UART. Prints one token: usb_ea|usb_da|uart_da|none."""

from __future__ import annotations

import argparse
import sys
import time


def _nmea_packet(body: str) -> bytes:
    cs = 0
    for c in body:
        cs ^= ord(c)
    return f"${body}*{cs:02X}\r\n".encode("ascii")


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
        print("sniff_lc29h_profile: pyserial required", file=sys.stderr)
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
        print(f"sniff_lc29h_profile: open {port}@{baud} failed: {exc}", file=sys.stderr)
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


def sniff(usb_port: str, uart_port: str) -> str:
    # USB: try EA baud first, then DA.
    for baud in (460800, 115200):
        prof = _query_verno(usb_port, baud)
        if prof == "ea":
            print(f"sniff_lc29h_profile: {usb_port}@{baud} → EA", file=sys.stderr)
            return "usb_ea"
        if prof == "da":
            print(f"sniff_lc29h_profile: {usb_port}@{baud} → DA", file=sys.stderr)
            return "usb_da"
    # UART DA path
    prof = _query_verno(uart_port, 115200)
    if prof == "da":
        print(f"sniff_lc29h_profile: {uart_port}@115200 → DA", file=sys.stderr)
        return "uart_da"
    if prof == "ea":
        # Unusual but treat UART EA as USB-style EA defaults if somehow present
        print(f"sniff_lc29h_profile: {uart_port}@115200 → EA (unexpected on UART)", file=sys.stderr)
        return "usb_ea"
    print("sniff_lc29h_profile: no LC29H VERNO on USB/UART", file=sys.stderr)
    return "none"


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--usb-port", default="/dev/ttyUSB0")
    p.add_argument("--uart-port", default="/dev/ttyAMA4")
    args = p.parse_args()
    token = sniff(args.usb_port, args.uart_port)
    print(token)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
