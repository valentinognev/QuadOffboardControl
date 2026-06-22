#!/usr/bin/env python3
"""
Per-port UART loopback test for Raspberry Pi (uses termios — no pip dependency).

Expected wiring (each port tested separately):
  On each UART: TX ──► RX (hardware loopback on that header pair)

Default devices match a typical Pi 5 triple-UART `config.txt`
(`uart0-pi5`, `uart2-pi5`, `uart3-pi5` → often ttyAMA0, ttyAMA2, ttyAMA3).
Override with --ports if your nodes differ.
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time

# Some Python builds omit IFLAG/OFLAG/CFLAG/LFLAG/CC on termios; tcattr list
# order is fixed (see termios.tcgetattr).
_TI = getattr(termios, "IFLAG", 0)
_TO = getattr(termios, "OFLAG", 1)
_TC = getattr(termios, "CFLAG", 2)
_TL = getattr(termios, "LFLAG", 3)
_TCC = getattr(termios, "CC", 6)
_ISPEED = getattr(termios, "ISPEED", 4)
_OSPEED = getattr(termios, "OSPEED", 5)


class TermiosUart:
    """Minimal raw 8N1 UART using Linux termios (no pyserial required)."""

    def __init__(self, device: str, baud: int) -> None:
        try:
            rate = getattr(termios, "B%d" % baud)
        except AttributeError as e:  # pragma: no cover
            raise ValueError(f"Unsupported baud rate: {baud}") from e
        flags = os.O_RDWR | os.O_NOCTTY
        self.fd = os.open(device, flags)
        self._path = device
        attrs = termios.tcgetattr(self.fd)
        attrs[_TI] = 0
        attrs[_TO] = 0
        attrs[_TL] = 0
        attrs[_TC] &= ~(
            termios.PARENB | termios.CSTOPB | termios.CRTSCTS | termios.CSIZE
        )
        attrs[_TC] |= termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[_TCC][termios.VMIN] = 0
        attrs[_TCC][termios.VTIME] = 0
        if hasattr(termios, "cfsetispeed"):
            termios.cfsetispeed(attrs, rate)
            termios.cfsetospeed(attrs, rate)
        else:  # pragma: no cover — minimal termios (e.g. some conda builds)
            attrs[_ISPEED] = rate
            attrs[_OSPEED] = rate
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def close(self) -> None:
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def flush_buffers(self, which: int = termios.TCIOFLUSH) -> None:
        termios.tcflush(self.fd, which)

    def write(self, data: bytes) -> None:
        with memoryview(data) as mv:
            total = len(mv)
            sent = 0
            while sent < total:
                n = os.write(self.fd, mv[sent:])
                if not n:
                    raise OSError(f"tty write stalled on {self._path}")
                sent += n

    def read(self, max_bytes: int, idle_timeout_s: float) -> bytes:
        """Read until ``max_bytes`` or idle timeout with no incoming data."""
        buf = bytearray()
        last = time.monotonic()
        while len(buf) < max_bytes:
            wait = idle_timeout_s - (time.monotonic() - last)
            if wait <= 0:
                break
            r, _, _ = select.select([self.fd], [], [], min(wait, 0.05))
            if self.fd in r:
                chunk = os.read(self.fd, max_bytes - len(buf))
                if chunk:
                    buf.extend(chunk)
                    last = time.monotonic()
                    continue
            if len(buf):
                break
        return bytes(buf)

    def read_some(self, max_bytes: int, overall_timeout_s: float) -> bytes:
        deadline = time.monotonic() + overall_timeout_s
        buf = bytearray()
        idle = min(0.2, overall_timeout_s * 0.1 + 1e-3)
        while len(buf) < max_bytes:
            chunk = self.read(max_bytes - len(buf), idle)
            buf.extend(chunk)
            if len(buf) >= max_bytes:
                break
            if time.monotonic() >= deadline:
                break
        return bytes(buf)


def test_loopback(
    device: str, baud: int, payload: bytes, timeout: float
) -> tuple[bool, str]:
    """Open ``device``, flush, send ``payload``, expect identical echo."""
    try:
        u = TermiosUart(device, baud)
    except OSError as e:
        return False, f"cannot open ({e})"
    try:
        u.flush_buffers(termios.TCIOFLUSH)
        time.sleep(0.02)
        u.write(payload)
        received = u.read_some(len(payload), timeout)
        if received == payload:
            return True, f"echo OK ({len(payload)} bytes)"
        return (
            False,
            f"echo mismatch: expected {len(payload)} bytes, got {len(received)}\n"
            f"  sent:     {payload!r}\n"
            f"  received: {received!r}",
        )
    finally:
        u.close()


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.strip())
    p.add_argument(
        "--ports",
        nargs="+",
        metavar="DEV",
        default=[
            "/dev/ttyAMA0",
            "/dev/ttyAMA2",
            "/dev/ttyAMA3",
        ],
        help="UART device nodes to test (each must have TX looped to its own RX)",
    )
    p.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="baud rate (8N1)",
    )
    p.add_argument(
        "--message",
        default="Hello-UART-loop\n",
        help="payload sent on each port (must echo back unchanged)",
    )
    p.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="seconds to wait for full echo on each port",
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv or sys.argv[1:])
    payload = args.message.encode("utf-8", errors="strict")

    any_fail = False
    for dev in args.ports:
        ok, detail = test_loopback(dev, args.baud, payload, args.timeout)
        prefix = "OK" if ok else "FAIL"
        print(f"{prefix}: {dev} — {detail}")
        if not ok:
            any_fail = True

    return 1 if any_fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
