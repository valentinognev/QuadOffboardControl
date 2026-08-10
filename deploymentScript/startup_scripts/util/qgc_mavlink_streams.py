#!/usr/bin/env python3
"""Request HIGHRES_IMU + OPTICAL_FLOW_RAD on the QGC/mavlink-server MAVLink path.

PX4 applies SET_MESSAGE_INTERVAL per mavlink instance. Companion HA requests on
UDP :14540 do not reliably raise rates on the UART instance that mavlink-server
exports to QGC (TCP :5760). This helper targets that shared command/GCS path so
QGC MAVLink Inspector shows IMU + optical flow by default after companion boot.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable, List, Sequence, Tuple

# msgid, default_hz placeholder unused in tuple name, display name
# Rates filled by stream_interval_requests(hz=...).
QGC_DEFAULT_STREAMS: Sequence[Tuple[int, int, str]] = (
    (105, 50, "HIGHRES_IMU"),
    (106, 50, "OPTICAL_FLOW_RAD"),
)

DEFAULT_STREAM_HZ = 50
DEFAULT_MAVLINK = "tcp:127.0.0.1:5760"
DEFAULT_TARGET_SYSTEM = 1
DEFAULT_TARGET_COMPONENT = 1
DEFAULT_REFRESH_SEC = 10.0
DEFAULT_HEARTBEAT_TIMEOUT_SEC = 15.0


def interval_us_for_hz(hz: float) -> int:
    if hz <= 0:
        raise ValueError(f"hz must be positive, got {hz}")
    return int(round(1e6 / float(hz)))


def stream_interval_requests(
    hz: float = DEFAULT_STREAM_HZ,
    streams: Sequence[Tuple[int, int, str]] = QGC_DEFAULT_STREAMS,
) -> List[Tuple[int, int, str]]:
    """Return [(msgid, interval_us, name), ...] for SET_MESSAGE_INTERVAL."""
    us = interval_us_for_hz(hz)
    return [(msgid, us, name) for msgid, _ignored_hz, name in streams]


def send_stream_interval_requests(
    mav_connection,
    requests: Iterable[Tuple[int, int, str]],
    *,
    target_system: int,
    target_component: int,
) -> None:
    """Send MAV_CMD_SET_MESSAGE_INTERVAL for each (msgid, interval_us, name)."""
    from pymavlink import mavutil  # lazy: unit tests need no pymavlink

    for msgid, interval_us, _name in requests:
        mav_connection.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(msgid),
            float(interval_us),
            0,
            0,
            0,
            0,
            0,
        )


def request_loop(
    *,
    mavlink: str = DEFAULT_MAVLINK,
    target_system: int = DEFAULT_TARGET_SYSTEM,
    target_component: int = DEFAULT_TARGET_COMPONENT,
    hz: float = DEFAULT_STREAM_HZ,
    refresh_sec: float = DEFAULT_REFRESH_SEC,
    heartbeat_timeout_sec: float = DEFAULT_HEARTBEAT_TIMEOUT_SEC,
    once: bool = False,
) -> int:
    """Connect, optionally wait for heartbeat, request streams, refresh until exit."""
    from pymavlink import mavutil

    reqs = stream_interval_requests(hz=hz)
    conn = mavutil.mavlink_connection(mavlink, autoreconnect=True)
    try:
        if mavlink.startswith("udpout:"):
            # Send-only inject path — no heartbeat; use CLI target ids.
            sysid, comp = target_system, target_component
        else:
            hb = conn.wait_heartbeat(timeout=heartbeat_timeout_sec)
            if hb is None:
                print(
                    f"qgc_mavlink_streams: no heartbeat on {mavlink} "
                    f"within {heartbeat_timeout_sec}s",
                    file=sys.stderr,
                )
                return 1
            sysid = conn.target_system or target_system
            comp = conn.target_component if conn.target_component is not None else target_component
            if sysid == 0:
                sysid = target_system
            if comp == 0:
                # PX4 often heartbeats as comp 1; 0 means "all" which is fine for commands.
                comp = target_component

        while True:
            send_stream_interval_requests(
                conn, reqs, target_system=sysid, target_component=comp
            )
            names = ", ".join(f"{n}@{hz:g}Hz" for _, _, n in reqs)
            print(
                f"qgc_mavlink_streams: requested {names} on {mavlink} "
                f"(sys={sysid} comp={comp})",
                flush=True,
            )
            if once:
                return 0
            time.sleep(refresh_sec)
    finally:
        try:
            conn.close()
        except Exception:
            pass


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description="Request HIGHRES_IMU + OPTICAL_FLOW_RAD for QGC (mavlink-server path)."
    )
    p.add_argument(
        "--mavlink",
        default=DEFAULT_MAVLINK,
        help=f"pymavlink URL (default {DEFAULT_MAVLINK}; alt udpout:127.0.0.1:14580)",
    )
    p.add_argument("--target-system", type=int, default=DEFAULT_TARGET_SYSTEM)
    p.add_argument("--target-component", type=int, default=DEFAULT_TARGET_COMPONENT)
    p.add_argument("--hz", type=float, default=DEFAULT_STREAM_HZ)
    p.add_argument("--refresh-sec", type=float, default=DEFAULT_REFRESH_SEC)
    p.add_argument("--heartbeat-timeout", type=float, default=DEFAULT_HEARTBEAT_TIMEOUT_SEC)
    p.add_argument("--once", action="store_true", help="Send once and exit")
    args = p.parse_args(argv)
    return request_loop(
        mavlink=args.mavlink,
        target_system=args.target_system,
        target_component=args.target_component,
        hz=args.hz,
        refresh_sec=args.refresh_sec,
        heartbeat_timeout_sec=args.heartbeat_timeout,
        once=args.once,
    )


if __name__ == "__main__":
    raise SystemExit(main())
