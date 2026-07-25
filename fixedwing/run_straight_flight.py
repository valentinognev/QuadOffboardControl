#!/usr/bin/env python3
"""
Start FlightGear Rascal SITL (optional) and hold a straight-line flight via OFFBOARD
velocity setpoints. Assumes the plane is already in-air from fg_spawn.env.

Usage:
  python3 fixedwing/run_straight_flight.py
  python3 fixedwing/run_straight_flight.py --no-sim --speed=30 --duration=120
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from pymavlink import mavutil

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_SIM = SCRIPT_DIR / "runSimFlightGearRascal.sh"

# Ignore position + accel + yaw/yaw_rate → velocity only (NED / body-NED).
TYPEMASK_VELOCITY_ONLY = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6


def connect(udp_port: int, timeout: float = 60.0) -> mavutil.mavfile:
    master = mavutil.mavlink_connection(f"udpin:0.0.0.0:{udp_port}")
    print(f"Waiting for heartbeat on UDP {udp_port} (timeout {timeout:.0f}s)...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            0,
        )
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg and msg.get_srcSystem() not in (0, 255):
            master.target_system = msg.get_srcSystem()
            master.target_component = msg.get_srcComponent()
            print(
                f"Heartbeat from sys={master.target_system} "
                f"comp={master.target_component}"
            )
            return master
    raise TimeoutError(f"No MAVLink heartbeat on UDP {udp_port}")


def is_armed(master: mavutil.mavfile) -> bool:
    hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=2.0)
    if not hb:
        return False
    return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def send_velocity(
    master: mavutil.mavfile,
    vx: float,
    vy: float,
    vz: float,
    frame: int,
) -> None:
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        frame,
        TYPEMASK_VELOCITY_ONLY,
        0,
        0,
        0,
        vx,
        vy,
        vz,
        0,
        0,
        0,
        0,
        0,
    )


def set_offboard(master: mavutil.mavfile) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        PX4_CUSTOM_MAIN_MODE_OFFBOARD,
        0,
        0,
        0,
        0,
        0,
    )


def arm(master: mavutil.mavfile) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def body_forward_frame() -> int:
    # Prefer BODY_NED so "straight" is aircraft-forward regardless of yaw.
    return mavutil.mavlink.MAV_FRAME_BODY_NED


def start_sim(sim_script: Path) -> subprocess.Popen:
    if not sim_script.is_file():
        raise FileNotFoundError(sim_script)
    print(f"Starting simulation: {sim_script}")
    # Detach from this process group so Ctrl+C stops only the MAVLink loop first.
    return subprocess.Popen(
        ["bash", str(sim_script)],
        cwd=str(sim_script.parent),
        start_new_session=True,
        env=os.environ.copy(),
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="OFFBOARD velocity straight-line flight for FlightGear Rascal SITL"
    )
    parser.add_argument(
        "--no-sim",
        action="store_true",
        help="Do not start runSimFlightGearRascal.sh (sim already running)",
    )
    parser.add_argument(
        "--sim",
        type=Path,
        default=DEFAULT_SIM,
        help=f"Path to sim runner (default: {DEFAULT_SIM})",
    )
    parser.add_argument(
        "--udp",
        type=int,
        default=14550,
        help="GCS MAVLink UDP port to listen on (default: 14550)",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=30.0,
        help="Forward speed m/s in body frame (default: 30)",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Setpoint rate Hz (default: 10)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to stream setpoints; 0 = until Ctrl+C (default: 0)",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=5.0,
        help="Seconds to wait after starting sim before MAVLink connect (default: 5)",
    )
    args = parser.parse_args()

    sim_proc: subprocess.Popen | None = None
    if not args.no_sim:
        sim_proc = start_sim(args.sim)
        time.sleep(max(0.0, args.warmup))

    def _cleanup(_signum=None, _frame=None) -> None:
        if sim_proc and sim_proc.poll() is None:
            print("\nStopping simulation container...")
            try:
                subprocess.run(
                    ["bash", str(args.sim), "--kill"],
                    check=False,
                    timeout=30,
                )
            except Exception as exc:  # noqa: BLE001
                print(f"Sim cleanup warning: {exc}")
        sys.exit(0)

    signal.signal(signal.SIGINT, _cleanup)
    signal.signal(signal.SIGTERM, _cleanup)

    try:
        master = connect(args.udp, timeout=120.0)
    except Exception as exc:  # noqa: BLE001
        print(f"MAVLink connect failed: {exc}", file=sys.stderr)
        _cleanup()
        return 1

    frame = body_forward_frame()
    period = 1.0 / max(args.rate, 1.0)
    vx, vy, vz = float(args.speed), 0.0, 0.0

    print("Streaming pre-OFFBOARD velocity setpoints...")
    for _ in range(20):
        send_velocity(master, vx, vy, vz, frame)
        time.sleep(0.1)

    print("Switching to OFFBOARD...")
    set_offboard(master)
    time.sleep(0.5)

    if not is_armed(master):
        print("Arming (FG in-air spawn does not imply PX4 armed)...")
        arm(master)
        time.sleep(1.0)
        if not is_armed(master):
            print("Warning: arm failed or not confirmed; continuing setpoints anyway")

    print(
        f"Holding straight flight: body-forward {args.speed} m/s "
        f"at {args.rate} Hz"
        + (f" for {args.duration}s" if args.duration > 0 else " until Ctrl+C")
    )

    t0 = time.time()
    next_t = t0
    while True:
        if args.duration > 0 and (time.time() - t0) >= args.duration:
            break
        send_velocity(master, vx, vy, vz, frame)
        next_t += period
        sleep_for = next_t - time.time()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_t = time.time()

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
