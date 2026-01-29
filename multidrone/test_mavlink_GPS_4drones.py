import time
import os
import struct
from pymavlink import mavutil

# NEW: USB/Serial output
import serial

# Force MAVLink 2.0
os.environ["MAVLINK20"] = "1"

# --- CONFIG FOR 4 DRONES ---
DRONE_PORTS = [14541, 14542, 14543, 14544]
TARGET_ALT = -10.0
HOVER_TIME = 60

# ---- NEW: USB serial output config ----
USB_PORT = "/dev/ttyUSB0"     # Windows example: "COM7"
USB_BAUD = 115200
TELEM_HZ = 10                # how often to send the struct over USB
KEEPALIVE_TIMEOUT_S = 1.0    # if no heartbeat in this time => drone not alive

# Frame format:
# [0xAA 0x55][payload_len u16][payload ...][checksum u8]
FRAME_MAGIC = b"\xAA\x55"

# Payload fields ::
# id                : uint8
# lat, lon, alt     : float32
# vn, ve, vd        : float32
# heading           : float32
# sm_current_stat   : int32
# battery_percent   : uint8
# keep_alive_mask   : uint8
# gps_3d_fix        : uint8
PAYLOAD_STRUCT_FMT = "<B f f f f f f f i B B B"

def checksum_xor(data: bytes) -> int:
    """Simple XOR checksum (fast, small)."""
    c = 0
    for b in data:
        c ^= b
    return c & 0xFF


def send_position_target(master, sys_id, x, y, z):
    master.mav.set_position_target_local_ned_send(
        0, sys_id, 1,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111111000,
        x, y, z,  # Position
        0, 0, 0,  # Velocity
        0, 0, 0,  # Acceleration
        0, 0      # Yaw, Yaw Rate
    )


def init_telemetry_state(drone_id: int):
    """Holds last-known telemetry values for one drone."""
    return {
        "id": drone_id,
        "lat": 0.0,
        "lon": 0.0,
        "alt": 0.0,   # meters (MSL or relative depending on message)
        "vn": 0.0,    # m/s
        "ve": 0.0,    # m/s
        "vd": 0.0,    # m/s
        "heading": 0.0,  # degrees
        "sm_current_stat": 0,  # you can map this to custom_mode/system_status
        "battery_percent": 0,  # 0..100
        "gps_3d_fix": 0,       # 0/1
        "last_heartbeat_ts": 0.0
    }


def poll_mavlink_updates(conn, state):
    """
    Non-blocking read of available MAVLink messages and updates 'state'.
    We’ll parse:
      - HEARTBEAT (keepalive + sm_current_stat candidate)
      - GPS_RAW_INT (fix type)
      - GLOBAL_POSITION_INT (lat/lon/alt + vx/vy/vz + hdg)
      - VFR_HUD (heading + groundspeed) [optional]
      - SYS_STATUS (battery remaining)
    """
    while True:
        msg = conn.recv_match(blocking=False)
        if msg is None:
            break

        mtype = msg.get_type()
        now = time.time()

        if mtype == "HEARTBEAT":
            state["last_heartbeat_ts"] = now
            # Candidate mapping: sm_current_stat from system_status or custom_mode
            # - system_status is a MAV_STATE-ish indicator
            # - custom_mode is PX4 mode; you can choose either
            try:
                state["sm_current_stat"] = int(getattr(msg, "custom_mode", 0))
            except Exception:
                state["sm_current_stat"] = 0

        elif mtype == "GPS_RAW_INT":
            # fix_type: 0..6, 3 = 3D fix
            fix_type = int(getattr(msg, "fix_type", 0))
            state["gps_3d_fix"] = 1 if fix_type >= 3 else 0

        elif mtype == "GLOBAL_POSITION_INT":
            # lat/lon are in 1e7 degrees, alt is mm (MSL), relative_alt mm
            lat = float(getattr(msg, "lat", 0)) / 1e7
            lon = float(getattr(msg, "lon", 0)) / 1e7

            # Choose alt source:
            # alt (MSL) in mm; relative_alt in mm above home
            # You asked "position GPS alt" -> usually MSL; adjust if you prefer relative_alt.
            alt_m = float(getattr(msg, "alt", 0)) / 1000.0

            # vx, vy, vz in cm/s in NED
            vn = float(getattr(msg, "vx", 0)) / 100.0
            ve = float(getattr(msg, "vy", 0)) / 100.0
            vd = float(getattr(msg, "vz", 0)) / 100.0

            # hdg in cdeg (0..35999); 65535 means unknown
            hdg = int(getattr(msg, "hdg", 65535))
            if hdg != 65535:
                heading_deg = float(hdg) / 100.0
                state["heading"] = heading_deg

            state["lat"] = lat
            state["lon"] = lon
            state["alt"] = alt_m
            state["vn"] = vn
            state["ve"] = ve
            state["vd"] = vd

        elif mtype == "VFR_HUD":
            # heading is degrees in VFR_HUD
            try:
                state["heading"] = float(getattr(msg, "heading", state["heading"]))
            except Exception:
                pass

        elif mtype == "SYS_STATUS":
            # battery_remaining: -1 unknown, else percent (0..100)
            br = int(getattr(msg, "battery_remaining", -1))
            if br >= 0:
                state["battery_percent"] = br


def compute_keepalive_mask(telemetry_states, connections):
    """
    Returns 4-bit mask (as int) where bit i = 1 if drone i is alive.
    We'll map bits by the order in 'connections' list.
    """
    mask = 0
    now = time.time()
    for i, d in enumerate(connections):
        sys_id = d["id"]
        st = telemetry_states.get(sys_id)
        if st is None:
            continue
        alive = (now - st["last_heartbeat_ts"]) <= KEEPALIVE_TIMEOUT_S
        if alive:
            mask |= (1 << i)
    return mask & 0x0F


def pack_and_send_over_usb(ser, state, keep_alive_mask):
    """
    Pack one drone's telemetry struct and send it as a framed binary packet.
    """
    payload = struct.pack(
        PAYLOAD_STRUCT_FMT,
        int(state["id"]) & 0xFF,
        float(state["lat"]),
        float(state["lon"]),
        float(state["alt"]),
        float(state["vn"]),
        float(state["ve"]),
        float(state["vd"]),
        float(state["heading"]),
        int(state["sm_current_stat"]),
        int(state["battery_percent"]) & 0xFF,
        int(keep_alive_mask) & 0xFF,
        int(state["gps_3d_fix"]) & 0xFF,
    )

    payload_len = len(payload)
    header = FRAME_MAGIC + struct.pack("<H", payload_len)
    csum = checksum_xor(payload)
    frame = header + payload + struct.pack("<B", csum)
    ser.write(frame)

if __name__ == "__main__":

    # 1. CONNECT TO ALL DRONES
    connections = []
    print("--- Connecting to Vehicles ---")

    for port in DRONE_PORTS:
        conn = mavutil.mavlink_connection("udpin:127.0.0.1:" + str(port))
        conn.wait_heartbeat()
        sys_id = conn.target_system
        print(f"Connected to UAV at Port {port} | System ID: {sys_id}")
        connections.append({"mav": conn, "id": sys_id, "offset_x": (DRONE_PORTS.index(port) * 5)})

    # NEW: Open USB serial port
    print(f"\n--- Opening USB serial: {USB_PORT} @ {USB_BAUD} ---")
    ser = serial.Serial(USB_PORT, USB_BAUD, timeout=0)

    # NEW: telemetry state dict per system id
    telemetry = {d["id"]: init_telemetry_state(d["id"]) for d in connections}

    # 2. INITIALIZE OFFBOARD STREAM (requires stream BEFORE switching mode)
    print("\nInitializing Offboard streams...")
    for _ in range(50):
        for drone in connections:
            send_position_target(drone["mav"], drone["id"], drone["offset_x"], 0, 0)
        time.sleep(0.1)

    # 3. ARM & SET OFFBOARD MODE
    for drone in connections:
        print(f"Arming and Setting OFFBOARD for System {drone['id']}...")
        drone["mav"].mav.command_long_send(
            drone["id"], 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        drone["mav"].mav.command_long_send(
            drone["id"], 1,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0, 1, 6, 0, 0, 0, 0, 0
        )

    # 4. SIMULTANEOUS TAKEOFF & HOVER (AND STREAM TELEMETRY OUT USB)
    print(f"\nTaking off! All drones climbing to {abs(TARGET_ALT)}m...")
    start_time = time.time()

    next_telem_send = time.time()
    telem_period = 1.0 / float(TELEM_HZ)

    while time.time() - start_time < HOVER_TIME:
        # Send setpoints
        for drone in connections:
            send_position_target(drone["mav"], drone["id"], drone["offset_x"], 0, TARGET_ALT)

        # NEW: Poll MAVLink messages and update telemetry for each drone
        for drone in connections:
            conn = drone["mav"]
            st = telemetry[drone["id"]]
            poll_mavlink_updates(conn, st)

        # NEW: Send telemetry at fixed rate
        now = time.time()
        if now >= next_telem_send:
            keep_mask = compute_keepalive_mask(telemetry, connections)

            # Send one struct per drone (4 packets per cycle)
            for drone in connections:
                st = telemetry[drone["id"]]
                pack_and_send_over_usb(ser, st, keep_mask)

            next_telem_send = now + telem_period

        time.sleep(0.1)

    # 5. LAND
    print("\nLanding all vehicles...")
    for drone in connections:
        drone["mav"].mav.command_long_send(
            drone["id"], 1,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    ser.close()
    print("Mission Complete.")