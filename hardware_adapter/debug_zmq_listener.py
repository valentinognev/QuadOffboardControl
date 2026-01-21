#!/usr/bin/env python3
"""
Debug ZMQ Listener - Compares simulator data to diagnose jitter

Subscribes to the FLIGHT_DATA topic on ZMQ port 7790 (same as zmq_to_serial_bridge.py)
and logs timing metrics to identify where jitter is introduced.

Usage:
    python3 debug_zmq_listener.py --duration 30
    python3 debug_zmq_listener.py --csv output.csv
"""

import zmq
import struct
import time
import argparse
import sys
import statistics
from datetime import datetime

# Configuration - must match mavlink_to_ZMQ.c and zmq_to_serial_bridge.py
ZMQ_TOPIC_DATA = b"FLIGHT_DATA"
ZMQ_PORT = 7790

# ZMQ struct format from zmq_to_serial_bridge.py
# 4s: Magic "FLIG"
# I: Version
# I: Message Count
# 63d: 63 doubles
# I: Custom Mode ID
# I: Mode
# B: Bools
# I: Gathered Flags
ZMQ_STRUCT_FMT = '<4sII63dIIBI'
ZMQ_STRUCT_SIZE = struct.calcsize(ZMQ_STRUCT_FMT)

# Indices in the 63-double array
IDX_HEADING = 16
IDX_VN = 45
IDX_VE = 46
IDX_VD = 47
IDX_LAT = 53
IDX_LON = 54
IDX_ALT = 55


class JitterAnalyzer:
    """Tracks timing statistics for jitter analysis."""
    
    def __init__(self):
        self.last_recv_time = None
        self.last_msg_count = None
        self.deltas = []
        self.dropped_count = 0
        self.duplicate_count = 0
        self.total_messages = 0
        
    def record_message(self, msg_count: int) -> dict:
        """Record a message and return timing stats."""
        now = time.time()
        self.total_messages += 1
        
        stats = {
            'recv_time': now,
            'msg_count': msg_count,
            'delta_ms': None,
            'dropped': 0,
            'duplicate': False
        }
        
        if self.last_recv_time is not None:
            delta = (now - self.last_recv_time) * 1000  # ms
            stats['delta_ms'] = delta
            self.deltas.append(delta)
            
            # Check for dropped or duplicate messages
            if self.last_msg_count is not None:
                expected = self.last_msg_count + 1
                if msg_count > expected:
                    dropped = msg_count - expected
                    stats['dropped'] = dropped
                    self.dropped_count += dropped
                elif msg_count == self.last_msg_count:
                    stats['duplicate'] = True
                    self.duplicate_count += 1
        
        self.last_recv_time = now
        self.last_msg_count = msg_count
        return stats
    
    def get_summary(self) -> dict:
        """Get summary statistics."""
        if len(self.deltas) < 2:
            return {'error': 'Not enough samples'}
        
        return {
            'total_messages': self.total_messages,
            'dropped_count': self.dropped_count,
            'duplicate_count': self.duplicate_count,
            'mean_delta_ms': statistics.mean(self.deltas),
            'stdev_delta_ms': statistics.stdev(self.deltas),
            'min_delta_ms': min(self.deltas),
            'max_delta_ms': max(self.deltas),
            'jitter_ms': max(self.deltas) - min(self.deltas)
        }


def unpack_zmq_message(msg_bytes: bytes) -> dict:
    """Unpack raw ZMQ bytes into a structured dict."""
    if len(msg_bytes) != ZMQ_STRUCT_SIZE:
        return None

    unpacked = struct.unpack(ZMQ_STRUCT_FMT, msg_bytes)
    
    # unpacked structure:
    # 0: magic, 1: version, 2: count
    # 3-65: doubles (indices 0-62)
    # 66: custom_mode_id, 67: mode, 68: bools, 69: gathered
    
    doubles = unpacked[3:66]  # 63 doubles
    
    return {
        'magic': unpacked[0],
        'version': unpacked[1],
        'msg_count': unpacked[2],
        'lat': doubles[IDX_LAT],
        'lon': doubles[IDX_LON],
        'alt': doubles[IDX_ALT],
        'vn': doubles[IDX_VN],
        've': doubles[IDX_VE],
        'vd': doubles[IDX_VD],
        'heading': doubles[IDX_HEADING],
        'custom_mode_id': unpacked[66]
    }


def main():
    parser = argparse.ArgumentParser(description="Debug ZMQ Listener for Jitter Analysis")
    parser.add_argument("--zmq-host", default="127.0.0.1", help="ZMQ host")
    parser.add_argument("--zmq-port", type=int, default=ZMQ_PORT, help="ZMQ port")
    parser.add_argument("--duration", type=int, default=0, help="Duration in seconds (0=indefinite)")
    parser.add_argument("--csv", type=str, help="Optional CSV output file")
    parser.add_argument("--quiet", action="store_true", help="Only show summary")
    
    args = parser.parse_args()
    
    # Setup ZMQ
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    zmq_addr = f"tcp://{args.zmq_host}:{args.zmq_port}"
    
    print(f"[DEBUG LISTENER] Connecting to {zmq_addr}...")
    subscriber.connect(zmq_addr)
    subscriber.setsockopt(zmq.SUBSCRIBE, ZMQ_TOPIC_DATA)
    
    # Set a receive timeout so we can check for Ctrl+C
    subscriber.setsockopt(zmq.RCVTIMEO, 1000)  # 1 second timeout
    
    analyzer = JitterAnalyzer()
    csv_file = None
    
    if args.csv:
        csv_file = open(args.csv, 'w')
        csv_file.write("timestamp,msg_count,delta_ms,lat,lon,alt,vn,ve,vd,heading,dropped,duplicate\n")
    
    print(f"[DEBUG LISTENER] Listening for FLIGHT_DATA messages...")
    print(f"[DEBUG LISTENER] Press Ctrl+C to stop and see summary\n")
    
    start_time = time.time()
    
    try:
        while True:
            # Check duration limit
            if args.duration > 0 and (time.time() - start_time) >= args.duration:
                print(f"\n[DEBUG LISTENER] Duration limit ({args.duration}s) reached.")
                break
            
            try:
                frames = subscriber.recv_multipart()
            except zmq.Again:
                # Timeout - just continue
                continue
            except zmq.ZMQError as e:
                print(f"[ERROR] ZMQ Error: {e}")
                continue
            
            # Parse message (handle single or multi-frame)
            msg = None
            if len(frames) == 1:
                full_msg = frames[0]
                if full_msg.startswith(ZMQ_TOPIC_DATA):
                    msg = full_msg[len(ZMQ_TOPIC_DATA):]
            elif len(frames) == 2:
                topic, data_frame = frames
                if topic == ZMQ_TOPIC_DATA:
                    msg = data_frame
            
            if msg is None:
                continue
            
            # Unpack data
            data = unpack_zmq_message(msg)
            if data is None:
                print(f"[WARN] Invalid message size: {len(msg)} (expected {ZMQ_STRUCT_SIZE})")
                continue
            
            # Record timing
            stats = analyzer.record_message(data['msg_count'])
            
            # Output
            if not args.quiet:
                delta_str = f"{stats['delta_ms']:.1f}ms" if stats['delta_ms'] else "---"
                warn = ""
                if stats['dropped'] > 0:
                    warn = f" [DROPPED {stats['dropped']}]"
                elif stats['duplicate']:
                    warn = " [DUPLICATE]"
                
                print(f"[{analyzer.total_messages:5d}] dt={delta_str:>8} | "
                      f"pos=({data['lat']:.5f}, {data['lon']:.5f}, {data['alt']:.2f}) | "
                      f"vel=({data['vn']:.2f}, {data['ve']:.2f}, {data['vd']:.2f}) | "
                      f"hdg={data['heading']:.1f}{warn}")
            
            # CSV output
            if csv_file:
                csv_file.write(f"{stats['recv_time']:.6f},{data['msg_count']},"
                              f"{stats['delta_ms'] or ''},"
                              f"{data['lat']},{data['lon']},{data['alt']},"
                              f"{data['vn']},{data['ve']},{data['vd']},"
                              f"{data['heading']},"
                              f"{stats['dropped']},{int(stats['duplicate'])}\n")
                
    except KeyboardInterrupt:
        print("\n[DEBUG LISTENER] Stopping...")
    
    finally:
        if csv_file:
            csv_file.close()
            print(f"[DEBUG LISTENER] CSV saved to: {args.csv}")
        
        subscriber.close()
        context.term()
        
        # Print summary
        summary = analyzer.get_summary()
        print("\n" + "=" * 60)
        print("JITTER ANALYSIS SUMMARY")
        print("=" * 60)
        
        if 'error' in summary:
            print(f"Error: {summary['error']}")
        else:
            print(f"Total Messages:    {summary['total_messages']}")
            print(f"Dropped Messages:  {summary['dropped_count']}")
            print(f"Duplicate Msgs:    {summary['duplicate_count']}")
            print(f"---")
            print(f"Mean Delta:        {summary['mean_delta_ms']:.2f} ms")
            print(f"Std Dev:           {summary['stdev_delta_ms']:.2f} ms")
            print(f"Min Delta:         {summary['min_delta_ms']:.2f} ms")
            print(f"Max Delta:         {summary['max_delta_ms']:.2f} ms")
            print(f"---")
            print(f"Jitter (max-min):  {summary['jitter_ms']:.2f} ms")
            
            # Interpretation
            if summary['stdev_delta_ms'] < 5:
                print("\n✓ ZMQ stream appears stable (low jitter)")
            elif summary['stdev_delta_ms'] < 20:
                print("\n⚠ ZMQ stream has moderate jitter")
            else:
                print("\n✗ ZMQ stream has HIGH jitter - issue is upstream")


if __name__ == "__main__":
    main()
