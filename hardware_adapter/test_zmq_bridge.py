import zmq
import struct
import time
import subprocess
import os
import pty
import sys
import threading
import signal

# Configuration
ZMQ_TOPIC_DATA = b"FLIGHT_DATA"
ZMQ_PORT = 7799 # Test port
BRIDGE_SCRIPT = "/home/lahav/rl_dockerfile/QuadOffboardControl/hardware_adapter/zmq_to_serial_bridge.py"

# ZMQ Struct Format (from zmq_to_serial_bridge.py)
ZMQ_STRUCT_FMT = '<4sII63dIIBI'
SERIAL_STRUCT_FMT = '<hfffffffhhh'

def create_mock_zmq_data(heading=3.14, lat=32.0):
    magic = b"FLIG"
    version = 1
    count = 100
    
    ZMQ_STRUCT_FMT = '<4sII63dIIBI'
    # ...
    
    doubles = [0.0] * 63
    # Set relevant fields (updated indices)
    doubles[16] = heading
    doubles[45] = 1.0 # VN
    doubles[46] = 2.0 # VE
    doubles[47] = -0.5 # VD
    doubles[53] = lat
    doubles[54] = 34.0 # Lon
    doubles[55] = 100.0 # Alt
    
    custom_mode_id = 99
    mode = 0
    bools = 0
    gathered = 0
    
    return struct.pack(ZMQ_STRUCT_FMT, magic, version, count, *doubles, custom_mode_id, mode, bools, gathered)

def zmq_publisher_thread(stop_event):
    context = zmq.Context()
    pub = context.socket(zmq.PUB)
    pub.bind(f"tcp://*:{ZMQ_PORT}")
    
    print(f"Publisher bound to {ZMQ_PORT}")
    
    while not stop_event.is_set():
        data = create_mock_zmq_data()
        # Simulate C publisher: Send Topic + Data in one frame
        # pub.send_multipart([ZMQ_TOPIC_DATA, data]) 
        pub.send(ZMQ_TOPIC_DATA + data)
        time.sleep(0.1)
    
    pub.close()
    context.term()

def run_bridge_test():
    # Create PTY pair
    master, slave = pty.openpty()
    s_name = os.ttyname(slave)
    print(f"Test Serial Port: {s_name}")

    # Start Publisher Thread
    stop_event = threading.Event()
    pub_thread = threading.Thread(target=zmq_publisher_thread, args=(stop_event,))
    pub_thread.start()
    
    # Start Bridge Process
    cmd = [
        sys.executable, 
        BRIDGE_SCRIPT,
        "--serial-port", s_name,
        "--zmq-host", "127.0.0.1",
        "--zmq-port", str(ZMQ_PORT),
        "--drone-id", "5"
    ]
    
    print(f"Starting bridge: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd, stdout=sys.stdout, stderr=sys.stderr)
    
    try:
        # Read from master PTY
        # Each packet is 36 bytes (SERIAL_STRUCT_FMT)
        print("Reading from master PTY...")
        data_buffer = b""
        start_time = time.time()
        
        while time.time() - start_time < 5:
            try:
                chunk = os.read(master, 1024)
                if not chunk:
                    break
                data_buffer += chunk
                
                if len(data_buffer) >= 36:
                    packet = data_buffer[:36]
                    data_buffer = data_buffer[36:]
                    
                    unpacked = struct.unpack(SERIAL_STRUCT_FMT, packet)
                    print(f"Received Packet: {unpacked}")
                    
                    # Verify fields
                    # id, lat, lon, alt, vn, ve, vd, heading, sm, bat, bit
                    # 5, 32.0, 34.0, 100.0, 1.0, 2.0, -0.5, 3.14, 99, 100, ...
                    
                    if unpacked[0] == 5 and abs(unpacked[1] - 32.0) < 0.001:
                        print("SUCCESS: Data verified!")
                        return True
            except OSError as e:
                print(f"Read error: {e}")
                break
                
        print("Timeout or failed to receive valid data.")
        return False
        
    finally:
        print("Cleaning up...")
        stop_event.set()
        pub_thread.join()
        proc.terminate()
        proc.wait()
        os.close(master)
        os.close(slave)

if __name__ == "__main__":
    success = run_bridge_test()
    if not success:
        sys.exit(1)
