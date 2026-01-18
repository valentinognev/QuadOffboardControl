#!/usr/bin/env python3
import zmq
import struct
import serial
import time
import argparse
import sys
import logging

# Configuration
ZMQ_TOPIC_DATA = b"FLIGHT_DATA"
ZMQ_PORT = 7790
DEFAULT_SERIAL_BAUDRATE = 115200

# Input ZMQ Format (Derived from mavlink_to_ZMQ.c)
# 4s: Magic "FLIG"
# I: Version
# I: Message Count
# 64d: 64 doubles
# I: Custom Mode ID
# I: Mode
# B: Bools
# I: Gathered Flags
ZMQ_STRUCT_FMT = '<4sII63dIIBI'
ZMQ_STRUCT_SIZE = struct.calcsize(ZMQ_STRUCT_FMT)

# Output Serial Format (Derived from simulate_serial_stream.py / drone_state.py)
# h: id
# f: lat
# f: lon
# f: alt
# f: vn
# f: ve
# f: vd
# f: heading
# h: sm_current_stat
# h: battery_percentages
# h: bitfield (gps_fix << 4 | keep_alive)
SERIAL_STRUCT_FMT = '<hfffffffhhh'

# Indices in the 63-double array for ZMQ data
IDX_HEADING = 16
IDX_VN = 45
IDX_VE = 46
IDX_VD = 47
IDX_LAT = 53
IDX_LON = 54
IDX_ALT = 55

def get_battery_percentage(zmq_data_dict):
    """
    Extract battery percentage.
    Currently hardcoded as the ZMQ packet doesn't have explicit battery info 
    mapped in a way we want yet, or we use a placeholder.
    """
    return 100

def get_drones_keep_alive(zmq_data_dict):
    """
    Get drones keep alive status.
    Placeholder: 15 (binary 1111) assuming all 4 drones are alive/mocked.
    """
    return 15

def get_gps_fix(zmq_data_dict):
    """
    Get GPS fix status.
    Placeholder: 1 (3D Fix).
    Could be derived from gathered flags or status text in future.
    """
    return 1

def unpack_zmq_message(msg_bytes):
    """
    Unpack the raw ZMQ bytes into a structured dict or list.
    Returns None if message size doesn't match.
    """
    if len(msg_bytes) != ZMQ_STRUCT_SIZE:
        logging.warning(f"Received message of size {len(msg_bytes)}, expected {ZMQ_STRUCT_SIZE}")
        return None

    unpacked = struct.unpack(ZMQ_STRUCT_FMT, msg_bytes)
    
    # unpacked structure:
    # 0: magic
    # 1: version
    # 2: count
    # 3-66: doubles (indices 0-63)
    # 67: custom_mode_id
    # 68: mode
    # 69: bools
    # 70: gathered
    
    doubles = unpacked[3:67]
    custom_mode_id = unpacked[67]
    
    data = {
        'lat': doubles[IDX_LAT],
        'lon': doubles[IDX_LON],
        'alt': doubles[IDX_ALT],
        'vn': doubles[IDX_VN],
        've': doubles[IDX_VE],
        'vd': doubles[IDX_VD],
        'heading': doubles[IDX_HEADING],
        'custom_mode_id': custom_mode_id
    }
    return data

def main():
    parser = argparse.ArgumentParser(description="Bridge ZMQ Flight Data to USB Serial")
    parser.add_argument("--serial-port", required=True, help="Serial port to write to (e.g., /dev/ttyUSB0)")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_SERIAL_BAUDRATE, help="Serial baudrate")
    parser.add_argument("--drone-id", type=int, default=1, help="Drone ID to broadcast")
    parser.add_argument("--zmq-host", default="127.0.0.1", help="ZMQ host")
    parser.add_argument("--zmq-port", type=int, default=ZMQ_PORT, help="ZMQ port to subscribe to")
    
    args = parser.parse_args()
    
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    # 1. Setup Serial
    logging.info(f"Opening serial port {args.serial_port} at {args.baudrate} baud...")
    try:
        ser = serial.Serial(args.serial_port, args.baudrate, timeout=1)
    except Exception as e:
        logging.error(f"Failed to open serial port: {e}")
        sys.exit(1)
        
    logging.info("Serial port opened.")

    # 2. Setup ZMQ
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    zmq_addr = f"tcp://{args.zmq_host}:{args.zmq_port}"
    logging.info(f"Connecting to ZMQ at {zmq_addr}...")
    subscriber.connect(zmq_addr)
    subscriber.setsockopt(zmq.SUBSCRIBE, ZMQ_TOPIC_DATA)
    
    logging.info("Bridge started. Waiting for ZMQ messages...")
    
    msg_count = 0
    try:
        while True:
            # Receive ZMQ message
            try:
                # content is expected to be [TOPIC][DATA] in a single frame due to zmq_wrapper.c implementation
                frames = subscriber.recv_multipart()
            except zmq.ZMQError as e:
                logging.error(f"ZMQ Error: {e}")
                continue
            
            # Debug logging (rate limited)
            msg_count += 1
            if msg_count % 100 == 0:
                logging.info(f"Processed {msg_count} messages. Last size: {len(frames[0])}")
            
            msg = None
            
            msg = None
            
            if len(frames) == 1:
                # Single frame: Topic + Data concatenated
                full_msg = frames[0]
                if full_msg.startswith(ZMQ_TOPIC_DATA):
                    msg = full_msg[len(ZMQ_TOPIC_DATA):]
                else:
                    continue
            elif len(frames) == 2:
                # Two frames: [Topic, Data]
                topic, data_frame = frames
                if topic == ZMQ_TOPIC_DATA:
                    msg = data_frame
                else:
                    continue
            else:
                logging.warning(f"Received unexpected number of frames: {len(frames)}")
                continue
                
            # Unpack
            data = unpack_zmq_message(msg)
            if data is None:
                continue
                
            # Extract additional/derived fields
            battery = get_battery_percentage(data)
            keep_alive = get_drones_keep_alive(data)
            gps_fix = get_gps_fix(data)
            
            # Construct Bitfield
            # bits: 4(gps) 3 2 1 0(keep_alive)
            bitfield = (gps_fix << 4) | (keep_alive & 0x0F)
            
            # Debug log values every 100 packets
            if msg_count % 100 == 0:
                logging.info(f"ID: {args.drone_id} | Pos: ({data['lat']:.5f}, {data['lon']:.5f}, {data['alt']:.2f}) | Vel: ({data['vn']:.2f}, {data['ve']:.2f}, {data['vd']:.2f}) | Hdg: {data['heading']:.2f}")

            # Pack for Serial using device_utils.py protocol (64 bytes)
            # Structure:
            # Sync: u16 (0xABCD)
            # Pos: 3f (lat, lon, alt)
            # Vel: 3f (vn, ve, vd)
            # Hdg: 3f (heading, 0, 0)
            # Time: i64 (time_ns/1000)
            # ID: u16
            # State: u16
            # Spare: 7h (14 bytes)
            
            SYNC_MARKER = 0xABCD
            current_time = int(time.time() * 1000000) # microseconds
            
            # Note: The original protocol uses float32 arrays.
            # We map:
            # Pos -> Lat, Lon, Alt
            # Vel -> Vn, Ve, Vd
            # Heading -> Heading, 0, 0 (filling vector)
            
            try:
                packet = struct.pack(
                    '<H3f3f3fqHH14s',
                    SYNC_MARKER,
                    data['lat'], data['lon'], data['alt'],         # Pos
                    data['vn'], data['ve'], data['vd'],            # Vel
                    data['heading'], 0.0, 0.0,                     # Heading
                    current_time,                                  # Time
                    args.drone_id,                                 # Sender ID
                    data['custom_mode_id'],                        # State
                    b'\x00' * 14                                   # Spare
                )
                
                # Send (header is inside the packet now)
                ser.write(packet)
                ser.flush()
                
            except struct.error as e:
                logging.error(f"Packing error: {e}")
                
    except KeyboardInterrupt:
        logging.info("Stopping bridge...")
    finally:
        ser.close()
        subscriber.close()
        context.term()

if __name__ == "__main__":
    main()
