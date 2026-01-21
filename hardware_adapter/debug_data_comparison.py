#!/usr/bin/env python3
"""
Debug Data Comparison - Compares ZMQ source data with Serial received data

Subscribes to ZMQ FLIGHT_DATA and reads from serial simultaneously,
comparing packet contents by matching position values to detect lag and corruption.

Usage:
    python3 debug_data_comparison.py --serial-port /dev/ttyUSB1 --duration 30
"""

import zmq
import struct
import serial
import time
import argparse
import sys
import threading
from collections import deque
import statistics
import math

# ZMQ Configuration
ZMQ_TOPIC_DATA = b"FLIGHT_DATA"
ZMQ_PORT = 7790
ZMQ_STRUCT_FMT = '<4sII63dIIBI'
ZMQ_STRUCT_SIZE = struct.calcsize(ZMQ_STRUCT_FMT)

# Serial Configuration (from zmq_to_serial_bridge.py / serial_bridge.py)
SERIAL_STRUCT_FMT = '<H3f3f3fqHH14s'
SERIAL_STRUCT_SIZE = 64
SYNC_MARKER = 0xABCD
SYNC_BYTES = b'\xcd\xab'

# Indices in ZMQ 63-double array
IDX_HEADING = 16
IDX_VN = 45
IDX_VE = 46
IDX_VD = 47
IDX_LAT = 53
IDX_LON = 54
IDX_ALT = 55


class DataStore:
    """Thread-safe storage for comparing ZMQ and Serial data by position."""
    
    def __init__(self, history_size=500):
        self.zmq_history = deque(maxlen=history_size)  # (recv_time, lat, lon, alt, vn, ve, vd, hdg)
        self.lock = threading.Lock()
        
        # Stats
        self.zmq_count = 0
        self.serial_count = 0
        self.matched_count = 0
        self.position_errors = []
        self.velocity_errors = []
        self.lag_samples = []  # How far back in history we found the match
        self.last_print_time = 0
        
    def add_zmq(self, data):
        with self.lock:
            self.zmq_count += 1
            self.zmq_history.append({
                'recv_time': time.time(),
                'lat': data['lat'],
                'lon': data['lon'],
                'alt': data['alt'],
                'vn': data['vn'],
                've': data['ve'],
                'vd': data['vd'],
                'heading': data['heading'],
            })
    
    def find_match_for_serial(self, serial_data):
        """Find the closest matching ZMQ packet by position."""
        with self.lock:
            self.serial_count += 1
            
            if len(self.zmq_history) == 0:
                return None
            
            best_idx = -1
            best_dist = float('inf')
            
            # Search through history for best position match
            for i, zmq_d in enumerate(self.zmq_history):
                # Position distance (using scaled coordinates)
                lat_diff = (zmq_d['lat'] - serial_data['lat']) * 111000  # ~meters
                lon_diff = (zmq_d['lon'] - serial_data['lon']) * 111000 * math.cos(math.radians(zmq_d['lat']))
                alt_diff = zmq_d['alt'] - serial_data['alt']
                
                dist = math.sqrt(lat_diff**2 + lon_diff**2 + alt_diff**2)
                
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
            
            if best_idx >= 0 and best_dist < 10.0:  # Within 10 meters
                zmq_d = self.zmq_history[best_idx]
                self.matched_count += 1
                
                # Calculate how far back in history (lag indicator)
                lag_index = len(self.zmq_history) - 1 - best_idx
                self.lag_samples.append(lag_index)
                
                # Calculate velocity error
                vel_err = math.sqrt(
                    (zmq_d['vn'] - serial_data['vn'])**2 +
                    (zmq_d['ve'] - serial_data['ve'])**2 +
                    (zmq_d['vd'] - serial_data['vd'])**2
                )
                self.velocity_errors.append(vel_err)
                self.position_errors.append(best_dist)
                
                return {
                    'match_idx': best_idx,
                    'lag': lag_index,
                    'pos_err': best_dist,
                    'vel_err': vel_err,
                    'zmq': zmq_d,
                    'serial': serial_data
                }
            
            return None
    
    def get_stats(self):
        with self.lock:
            result = {
                'zmq_count': self.zmq_count,
                'serial_count': self.serial_count,
                'matched': self.matched_count,
            }
            if self.lag_samples:
                result['lag_mean'] = statistics.mean(self.lag_samples)
                result['lag_max'] = max(self.lag_samples)
                result['lag_stdev'] = statistics.stdev(self.lag_samples) if len(self.lag_samples) > 1 else 0
            if self.position_errors:
                result['pos_err_mean'] = statistics.mean(self.position_errors)
                result['pos_err_max'] = max(self.position_errors)
            if self.velocity_errors:
                result['vel_err_mean'] = statistics.mean(self.velocity_errors)
                result['vel_err_max'] = max(self.velocity_errors)
            return result


def zmq_reader(store: DataStore, host: str, port: int, stop_event: threading.Event):
    """Read from ZMQ and add to store."""
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.connect(f"tcp://{host}:{port}")
    subscriber.setsockopt(zmq.SUBSCRIBE, ZMQ_TOPIC_DATA)
    subscriber.setsockopt(zmq.RCVTIMEO, 500)
    
    print(f"[ZMQ] Connected to tcp://{host}:{port}")
    
    while not stop_event.is_set():
        try:
            frames = subscriber.recv_multipart()
        except zmq.Again:
            continue
        except zmq.ZMQError as e:
            print(f"[ZMQ] Error: {e}")
            continue
        
        # Parse message
        msg = None
        if len(frames) == 1:
            full_msg = frames[0]
            if full_msg.startswith(ZMQ_TOPIC_DATA):
                msg = full_msg[len(ZMQ_TOPIC_DATA):]
        elif len(frames) == 2:
            topic, data_frame = frames
            if topic == ZMQ_TOPIC_DATA:
                msg = data_frame
        
        if msg is None or len(msg) != ZMQ_STRUCT_SIZE:
            continue
        
        unpacked = struct.unpack(ZMQ_STRUCT_FMT, msg)
        doubles = unpacked[3:66]
        
        data = {
            'lat': doubles[IDX_LAT],
            'lon': doubles[IDX_LON],
            'alt': doubles[IDX_ALT],
            'vn': doubles[IDX_VN],
            've': doubles[IDX_VE],
            'vd': doubles[IDX_VD],
            'heading': doubles[IDX_HEADING],
        }
        
        store.add_zmq(data)
    
    subscriber.close()
    context.term()
    print("[ZMQ] Reader stopped")


def serial_reader(store: DataStore, port_path: str, baudrate: int, stop_event: threading.Event, verbose: bool):
    """Read from serial and compare with ZMQ data."""
    try:
        ser = serial.Serial(port_path, baudrate, timeout=0.5)
    except Exception as e:
        print(f"[SERIAL] Failed to open {port_path}: {e}")
        return
    
    print(f"[SERIAL] Connected to {port_path} at {baudrate} baud")
    
    buffer = b""
    recv_count = 0
    last_status_time = time.time()
    
    while not stop_event.is_set():
        try:
            chunk = ser.read(256)
        except Exception as e:
            print(f"[SERIAL] Read error: {e}")
            break
        
        if not chunk:
            continue
        
        buffer += chunk
        
        # Process all complete packets in buffer
        while True:
            # Find sync marker
            idx = buffer.find(SYNC_BYTES)
            if idx == -1:
                buffer = buffer[-1:] if buffer else b""
                break
            
            buffer = buffer[idx:]
            
            if len(buffer) < SERIAL_STRUCT_SIZE:
                break
            
            candidate = buffer[:SERIAL_STRUCT_SIZE]
            buffer = buffer[SERIAL_STRUCT_SIZE:]
            
            try:
                unpacked = struct.unpack(SERIAL_STRUCT_FMT, candidate)
                # unpacked: sync, lat, lon, alt, vn, ve, vd, hdg0, hdg1, hdg2, time, id, state, spare
                
                recv_count += 1
                
                serial_data = {
                    'lat': unpacked[1],
                    'lon': unpacked[2],
                    'alt': unpacked[3],
                    'vn': unpacked[4],
                    've': unpacked[5],
                    'vd': unpacked[6],
                    'heading': unpacked[7],
                }
                
                match = store.find_match_for_serial(serial_data)
                
                if verbose and match:
                    print(f"[{recv_count:5d}] lag={match['lag']:3d} pos_err={match['pos_err']:.4f}m vel_err={match['vel_err']:.4f}")
                
                # Periodic status
                now = time.time()
                if now - last_status_time > 2.0:
                    stats = store.get_stats()
                    match_pct = (stats['matched'] / stats['serial_count'] * 100) if stats['serial_count'] > 0 else 0
                    lag_str = f"{stats.get('lag_mean', 0):.1f}" if 'lag_mean' in stats else "?"
                    print(f"[STATUS] Serial: {stats['serial_count']} | ZMQ: {stats['zmq_count']} | "
                          f"Match: {match_pct:.1f}% | Avg Lag: {lag_str} samples")
                    last_status_time = now
                    
            except struct.error as e:
                print(f"[SERIAL] Unpack error: {e}")
    
    ser.close()
    print("[SERIAL] Reader stopped")


def main():
    parser = argparse.ArgumentParser(description="Compare ZMQ and Serial data streams by position matching")
    parser.add_argument("--zmq-host", default="127.0.0.1", help="ZMQ host")
    parser.add_argument("--zmq-port", type=int, default=ZMQ_PORT, help="ZMQ port")
    parser.add_argument("--serial-port", required=True, help="Serial port to read from (receiver side)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--duration", type=int, default=30, help="Duration in seconds")
    parser.add_argument("--verbose", "-v", action="store_true", help="Show each packet match")
    
    args = parser.parse_args()
    
    print("=" * 70)
    print("DEBUG DATA COMPARISON (Position Matching)")
    print("=" * 70)
    print(f"ZMQ Source:    tcp://{args.zmq_host}:{args.zmq_port}")
    print(f"Serial Input:  {args.serial_port} @ {args.baudrate} baud")
    print(f"Duration:      {args.duration}s")
    print("=" * 70)
    print()
    print("This tool compares ZMQ data with serial data by matching position values.")
    print("'Lag' = how many ZMQ packets behind the serial data is (jitter indicator)")
    print()
    
    store = DataStore()
    stop_event = threading.Event()
    
    # Start reader threads
    zmq_thread = threading.Thread(target=zmq_reader, 
                                  args=(store, args.zmq_host, args.zmq_port, stop_event))
    serial_thread = threading.Thread(target=serial_reader,
                                     args=(store, args.serial_port, args.baudrate, stop_event, args.verbose))
    
    zmq_thread.start()
    serial_thread.start()
    
    try:
        time.sleep(args.duration)
    except KeyboardInterrupt:
        print("\nStopping...")
    
    stop_event.set()
    zmq_thread.join(timeout=2)
    serial_thread.join(timeout=2)
    
    stats = store.get_stats()
    
    print()
    print("=" * 70)
    print("COMPARISON RESULTS")
    print("=" * 70)
    print(f"ZMQ Messages Received:     {stats['zmq_count']}")
    print(f"Serial Messages Received:  {stats['serial_count']}")
    print(f"Successfully Matched:      {stats['matched']}")
    print()
    
    if stats['serial_count'] == 0:
        print("✗ NO SERIAL DATA RECEIVED")
        print("  Check: Is the receiving serial port correct?")
        print("  Note: You need to specify the RECEIVER port (where ground station reads)")
    elif stats['matched'] == 0:
        print("✗ NO MATCHES FOUND")
        print("  The serial data doesn't match any ZMQ positions within 10m")
        print("  This could indicate severe data corruption")
    else:
        match_pct = stats['matched'] / stats['serial_count'] * 100
        print(f"Match Rate:  {match_pct:.1f}%")
        print()
        
        if 'lag_mean' in stats:
            print("--- LAG ANALYSIS (samples behind) ---")
            print(f"  Mean Lag:   {stats['lag_mean']:.1f} samples")
            print(f"  Max Lag:    {stats['lag_max']} samples")
            print(f"  Lag StdDev: {stats['lag_stdev']:.2f}")
            print()
            
            # Interpretation
            if stats['lag_stdev'] > 10:
                print("✗ HIGH LAG VARIANCE - This indicates jitter!")
                print("  The serial data is arriving with inconsistent delays.")
                print("  Issue is likely in: zmq_to_serial_bridge, serial TX, or serial RX")
            elif stats['lag_mean'] > 50:
                print("⚠ HIGH AVERAGE LAG - Serial is significantly behind ZMQ")
                print("  Serial throughput may be too slow for the data rate")
            else:
                print("✓ Lag appears stable (low variance)")
        
        if 'vel_err_mean' in stats and stats['vel_err_max'] > 0.1:
            print()
            print("--- DATA INTEGRITY ---")
            print(f"  Velocity Error Mean: {stats['vel_err_mean']:.4f} m/s")
            print(f"  Velocity Error Max:  {stats['vel_err_max']:.4f} m/s")
            if stats['vel_err_max'] > 1.0:
                print("  ⚠ Large velocity differences detected - possible data issues")


if __name__ == "__main__":
    main()

