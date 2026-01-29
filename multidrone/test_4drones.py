import time
import os
from pymavlink import mavutil

# Force MAVLink 2.0
os.environ['MAVLINK20'] = '1'

# --- CONFIG FOR 4 DRONES (QGC IDs 2, 3, 4, 5) ---
# Assuming SITL instances started in order
DRONE_PORTS = [14541, 14542, 14543, 14544]
TARGET_ALT = -10.0
HOVER_TIME = 60

def send_position_target(master, sys_id, x, y, z):
    master.mav.set_position_target_local_ned_send(
        0, sys_id, 1,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111111000,
        x, y, z, # Position
        0, 0, 0, # Velocity
        0, 0, 0, # Acceleration
        0, 0     # Yaw, Yaw Rate
    )

# 1. CONNECT TO ALL DRONES
connections = []
print("--- Connecting to Vehicles ---")

for port in DRONE_PORTS:
    conn = mavutil.mavlink_connection('udpin:127.0.0.1:' + str(port))
    conn.wait_heartbeat()
    sys_id = conn.target_system
    print(f"Connected to UAV at Port {port} | System ID: {sys_id}")
    connections.append({'mav': conn, 'id': sys_id, 'offset_x': (DRONE_PORTS.index(port) * 5)})

# 2. INITIALIZE OFFBOARD STREAM
# Offboard mode requires a stream of setpoints BEFORE switching mode
print("\nInitializing Offboard streams...")
for _ in range(50):
    for drone in connections:
        send_position_target(drone['mav'], drone['id'], drone['offset_x'], 0, 0)
    time.sleep(0.1)

# 3. ARM & SET OFFBOARD MODE
for drone in connections:
    print(f"Arming and Setting OFFBOARD for System {drone['id']}...")
    # Arming
    drone['mav'].mav.command_long_send(
        drone['id'], 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    # Set Mode to OFFBOARD (PX4 Custom Mode: 6)
    drone['mav'].mav.command_long_send(
        drone['id'], 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)

# 4. SIMULTANEOUS TAKEOFF & HOVER
print(f"\nTaking off! All drones climbing to {abs(TARGET_ALT)}m...")
start_time = time.time()
while time.time() - start_time < HOVER_TIME:
    for drone in connections:
        # Give each drone a 5m offset so they don't collide
        send_position_target(drone['mav'], drone['id'], drone['offset_x'], 0, TARGET_ALT)
    time.sleep(0.1)

# 5. LAND
print("\nLanding all vehicles...")
for drone in connections:
    drone['mav'].mav.command_long_send(
        drone['id'], 1, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

print("Mission Complete.")