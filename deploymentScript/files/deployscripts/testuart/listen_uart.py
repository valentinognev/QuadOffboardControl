#!/usr/bin/env python3
"""
Listen on every hardware USART port on the system and print received data.

Scans /dev/ttyAMA* and /dev/ttyS* (typical Pi PL011 / 8250 UART nodes), opens
each port at 8N1, listens for a fixed duration, then prints whatever arrived.

Uses termios — no pyserial dependency (same approach as testuart.py).
"""

from __future__ import annotations

import argparse
import glob
import os
import sys
import termios

from testuart import TermiosUart


def discover_uart_ports() -> list[str]:
    """Return sorted unique hardware UART device nodes present on this system."""
    ports: set[str] = set()
    for pattern in ("/dev/ttyAMA*", "/dev/ttyS*"):
        ports.update(glob.glob(pattern))
    return sorted(p for p in ports if os.path.exists(p))


def format_data(data: bytes) -> str:
    if not data:
        return "(no data)"
    try:
        text = data.decode("utf-8", errors="replace")
        if text.isprintable() or "\n" in text or "\r" in text:
            return repr(text)
    except Exception:
        pass
    return repr(data)


def listen_port(device: str, baud: int, duration_s: float) -> tuple[bytes | None, str | None]:
    """Open ``device``, listen for ``duration_s`` seconds, return (data, error)."""
    try:
        uart = TermiosUart(device, baud)
    except OSError as exc:
        return None, str(exc)
    try:
        uart.flush_buffers(termios.TCIOFLUSH)
        data = uart.read_some(65536, duration_s)
        return data, None
    finally:
        uart.close()


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.strip())
    p.add_argument(
        "--ports",
        nargs="+",
        metavar="DEV",
        default=None,
        help="UART devices to listen on (default: auto-discover ttyAMA* and ttyS*)",
    )
    p.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="seconds to listen on each port (default: 3)",
    )
    p.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="baud rate 8N1 (default: 115200)",
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    ports = args.ports if args.ports else discover_uart_ports()

    if not ports:
        print("No UART ports found (/dev/ttyAMA*, /dev/ttyS*)", file=sys.stderr)
        return 1

    print(f"Listening {args.duration:.1f}s @ {args.baud} baud on {len(ports)} port(s)\n")

    any_fail = False
    for dev in ports:
        print(f"=== {dev} ===")
        data, err = listen_port(dev, args.baud, args.duration)
        if err is not None:
            print(f"  ERROR: cannot open or read ({err})")
            any_fail = True
        elif data:
            print(f"  {len(data)} byte(s): {format_data(data)}")
        else:
            print("  (no data)")
        print()

    return 1 if any_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
