#!/usr/bin/env python3
"""Validate CommModules air soft-config against the rover timing profile.

Used by companion New/Update deploy so GS-like radio NVS (D1 RF regression)
cannot persist. Idempotent: already-correct modules are read-only no-ops.

Does NOT apply the full ModuleCfg ``rover`` profile (that hardcodes unit_id=1).
Writes only the four timing props + the companion drone unit_id, then
apply_role / apply_radio / save_nvs.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from typing import Any

# Protocol v6 (ModuleCfg / HA soft_setup)
MAGIC_SETUP = 0xEEEE
OP_WRITE = 0
OP_READ = 1
FRAME_LEN = 10
BAUD = 921600

PROP_UNIT_ID = 0x000A
PROP_APPLY_RADIO = 0x000C
PROP_SAVE_NVS = 0x000D
PROP_APPLY_ROLE = 0x000F
PROP_RX_ACQUIRE_MS = 0x0020
PROP_RX_HOLD_MS = 0x0021
PROP_RX_UPLINK_MS = 0x0022
PROP_SYNC_HOLD_SLOTS = 0x002F

# Known-good air/rover timing (module_config.json profiles.rover + D2 live).
# GS-like bad: rx_uplink_ms=8, rx_acquire_ms=1, rx_hold_ms=1, sync_hold_slots=2
ROVER_TIMING: dict[str, int] = {
    "rx_uplink_ms": 1,
    "rx_acquire_ms": 12,
    "rx_hold_ms": 4,
    "sync_hold_slots": 4,
}

PROP_IDS: dict[str, int] = {
    "unit_id": PROP_UNIT_ID,
    "rx_uplink_ms": PROP_RX_UPLINK_MS,
    "rx_acquire_ms": PROP_RX_ACQUIRE_MS,
    "rx_hold_ms": PROP_RX_HOLD_MS,
    "sync_hold_slots": PROP_SYNC_HOLD_SLOTS,
}


def expected_rover_props(unit_id: int) -> dict[str, int]:
    """Expected soft-config for an air radio with this companion drone id."""
    uid = int(unit_id)
    if uid < 1 or uid > 15:
        raise ValueError(f"unit_id must be 1..15, got {uid}")
    out = dict(ROVER_TIMING)
    out["unit_id"] = uid
    return out


def diff_needs_apply(
    actual: dict[str, int],
    expected: dict[str, int],
) -> dict[str, tuple[int, int]]:
    """Return name → (actual, expected) for every mismatched key in expected."""
    mismatches: dict[str, tuple[int, int]] = {}
    for name, want in expected.items():
        got = actual.get(name)
        if got is None or int(got) != int(want):
            mismatches[name] = (int(got) if got is not None else -1, int(want))
    return mismatches


def _xor8(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x & 0xFF


def make_setup_frame(op: int, prop_id: int, value: int = 0) -> bytes:
    body = struct.pack("<BHI", op & 0xFF, prop_id & 0xFFFF, value & 0xFFFFFFFF)
    return struct.pack("<H", MAGIC_SETUP) + body + bytes([_xor8(body)])


def parse_setup_frame(data: bytes) -> tuple[int, int, int] | None:
    if len(data) < FRAME_LEN:
        return None
    magic, op, prop_id, value, chk = struct.unpack_from("<HBHIB", data, 0)
    if magic != MAGIC_SETUP:
        return None
    if _xor8(data[2:9]) != chk:
        return None
    return op, prop_id, value


def _write_prop(ser: Any, prop_id: int, value: int) -> None:
    ser.write(make_setup_frame(OP_WRITE, prop_id, value & 0xFFFFFFFF))
    ser.flush()


def _read_prop(ser: Any, prop_id: int, timeout_s: float = 1.5) -> int:
    if hasattr(ser, "reset_input_buffer"):
        ser.reset_input_buffer()
    ser.write(make_setup_frame(OP_READ, prop_id, 0))
    ser.flush()
    deadline = time.time() + timeout_s
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(64)
        if chunk:
            buf.extend(chunk)
            i = 0
            while i + FRAME_LEN <= len(buf):
                if buf[i] == 0xEE and buf[i + 1] == 0xEE:
                    parsed = parse_setup_frame(bytes(buf[i : i + FRAME_LEN]))
                    if parsed and parsed[0] == OP_READ and parsed[1] == prop_id:
                        return parsed[2] & 0xFFFFFFFF
                    i += 2
                else:
                    i += 1
            if len(buf) > 512:
                buf = buf[-64:]
        else:
            time.sleep(0.01)
    raise TimeoutError(f"no READ reply for prop 0x{prop_id:04X}")


def read_props(ser: Any, names: list[str] | None = None) -> dict[str, int]:
    keys = names or list(PROP_IDS.keys())
    out: dict[str, int] = {}
    for name in keys:
        prop_id = PROP_IDS[name]
        raw = _read_prop(ser, prop_id)
        out[name] = int(raw) & 0xFF if name == "unit_id" else int(raw)
        time.sleep(0.02)
    return out


def apply_rover_soft_config(ser: Any, unit_id: int) -> None:
    """Write rover timing + unit_id, then apply_role / apply_radio / save_nvs."""
    expected = expected_rover_props(unit_id)
    # Timing first, then role, then radio + NVS (matches ModuleCfg / soft_setup).
    for name in ("rx_uplink_ms", "rx_acquire_ms", "rx_hold_ms", "sync_hold_slots"):
        _write_prop(ser, PROP_IDS[name], expected[name])
        time.sleep(0.05)
    _write_prop(ser, PROP_UNIT_ID, expected["unit_id"])
    time.sleep(0.05)
    _write_prop(ser, PROP_APPLY_ROLE, 1)
    time.sleep(0.05)
    _write_prop(ser, PROP_APPLY_RADIO, 1)
    time.sleep(0.05)
    _write_prop(ser, PROP_SAVE_NVS, 1)
    time.sleep(0.1)


def format_props(props: dict[str, int]) -> str:
    order = (
        "unit_id",
        "rx_uplink_ms",
        "rx_acquire_ms",
        "rx_hold_ms",
        "sync_hold_slots",
    )
    parts = [f"{k}={props[k]}" for k in order if k in props]
    return " ".join(parts)


def run(
    *,
    port: str,
    unit_id: int,
    dry_run: bool = False,
    apply: bool = True,
) -> int:
    """
    Returns:
      0 — validated OK or corrected
      2 — soft-fail (missing port / pyserial / no reply)
      1 — hard error (bad args after open succeeded inconsistently)
    """
    prefix = "ensure_companion_radio_soft_config"
    try:
        expected = expected_rover_props(unit_id)
    except ValueError as exc:
        print(f"{prefix}: ERROR: {exc}", file=sys.stderr)
        return 1

    try:
        import serial  # type: ignore
    except ImportError:
        print(f"{prefix}: WARNING: pyserial missing — skip radio soft-config", file=sys.stderr)
        return 2

    import os

    if not os.path.exists(port):
        print(f"{prefix}: WARNING: missing {port} — skip radio soft-config", file=sys.stderr)
        return 2

    try:
        ser = serial.Serial(port, BAUD, timeout=0.05)
    except Exception as exc:
        print(f"{prefix}: WARNING: open {port} failed: {exc}", file=sys.stderr)
        return 2

    try:
        time.sleep(0.05)
        if hasattr(ser, "reset_input_buffer"):
            ser.reset_input_buffer()
        try:
            actual = read_props(ser)
        except Exception as exc:
            print(
                f"{prefix}: WARNING: read soft-config failed ({exc}) — skip",
                file=sys.stderr,
            )
            return 2

        mismatches = diff_needs_apply(actual, expected)
        print(f"{prefix}: port={port} actual: {format_props(actual)}", flush=True)
        print(f"{prefix}: expected: {format_props(expected)}", flush=True)

        if not mismatches:
            print(f"{prefix}: validated OK (rover timing + unit_id={unit_id})", flush=True)
            return 0

        detail = ", ".join(
            f"{k}: {got}->{want}" for k, (got, want) in sorted(mismatches.items())
        )
        print(f"{prefix}: mismatch: {detail}", flush=True)

        if dry_run or not apply:
            print(f"{prefix}: dry-run — would apply rover soft-config + save_nvs", flush=True)
            return 0

        apply_rover_soft_config(ser, unit_id)
        time.sleep(0.15)
        try:
            after = read_props(ser)
        except Exception as exc:
            print(f"{prefix}: WARNING: post-apply read failed: {exc}", file=sys.stderr)
            print(f"{prefix}: corrected (unverified)", flush=True)
            return 0

        still = diff_needs_apply(after, expected)
        print(f"{prefix}: after: {format_props(after)}", flush=True)
        if still:
            print(
                f"{prefix}: WARNING: still mismatched after apply: {still}",
                file=sys.stderr,
            )
            return 2
        print(
            f"{prefix}: corrected + NVS saved (rover timing + unit_id={unit_id})",
            flush=True,
        )
        return 0
    finally:
        try:
            ser.close()
        except Exception:
            pass


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description="Validate/apply CommModules rover soft-config on air radio UART",
    )
    p.add_argument(
        "--port",
        default="/dev/ttyAMA2",
        help="Radio UART (default: /dev/ttyAMA2)",
    )
    p.add_argument(
        "--unit-id",
        type=int,
        required=True,
        help="Companion drone id (air unit_id 1..15)",
    )
    p.add_argument(
        "--dry-run",
        action="store_true",
        help="Read and report only; do not write NVS",
    )
    p.add_argument(
        "--no-apply",
        action="store_true",
        help="Alias for --dry-run",
    )
    args = p.parse_args(argv)
    return run(
        port=args.port,
        unit_id=args.unit_id,
        dry_run=bool(args.dry_run or args.no_apply),
        apply=not bool(args.dry_run or args.no_apply),
    )


if __name__ == "__main__":
    sys.exit(main())
