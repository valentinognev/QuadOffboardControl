#!/usr/bin/env python3
"""Simple ZMQ subscriber for gazebo->ZMQ bridge messages.

Listens to a ZMQ PUB endpoint, prints topic and JSON payload, and can
filter messages by vehicle/pose name.

Usage:
  python3 zmq_pose_listener.py --endpoint tcp://127.0.0.1:5556 --topic /gazebo/default/pose/info

Options:
  --endpoint   ZMQ endpoint to connect to (default tcp://127.0.0.1:5556)
  --topic      Gazebo topic to subscribe to (default subscribe to all)
  --filter     Substring to filter pose `name` fields (optional)
  --regex      Interpret filter as a regular expression instead of substring
  --pretty     Pretty-print JSON payloads
"""

import argparse
import json
import re
import sys
import time

import zmq

#listento = "tcp://127.0.0.1:5556" sadly doesn't hear anything
listento = "tcp://192.168.1.1:5556"

# on the container this is at:
# /home/valentin/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/tools/zmq_pose_listener.py

def main():
    p = argparse.ArgumentParser(description="ZMQ pose listener for Gazebo bridge")
    p.add_argument("--endpoint", default=listento,
                   help=f"ZMQ endpoint (default: {listento})")
    p.add_argument("--topic", default="",
                   help="Gazebo topic to subscribe to (default: subscribe to all)")
    p.add_argument("--filter", default=None,
                   help="Substring or regex to filter pose 'name' fields")
    p.add_argument("--regex", action="store_true", help="Treat --filter as a regex")
    p.add_argument("--pretty", action="store_true", help="Pretty-print JSON payloads")
    args = p.parse_args()

    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.connect(args.endpoint)

    # Subscribe: empty string subscribes to all topics
    sock.setsockopt_string(zmq.SUBSCRIBE, args.topic)

    print(f"Connected to {args.endpoint}, subscribed to '{args.topic or '<all>'}'")

    filter_re = True
    exact_name = 'iris'
    #changed to true so that it filters by pose names automatically
    if args.filter:
        if args.regex:
            filter_re = re.compile(args.filter)
        else:
            # case-insensitive substring search
            filter_re = re.compile(re.escape(args.filter), re.IGNORECASE)

    try:
        while True:
            parts = sock.recv_multipart()
            if len(parts) == 0:
                continue

            # first frame is topic
            topic = parts[0].decode('utf-8', errors='replace') if parts[0] else ''
            # second frame is payload (JSON) in our bridge
            payload = None
            if len(parts) > 1:
                try:
                    payload_str = parts[1].decode('utf-8', errors='replace')
                    payload = json.loads(payload_str)
                except Exception:
                    payload = parts[1].decode('utf-8', errors='replace')

            ts = time.strftime('%Y-%m-%d %H:%M:%S')

            # Filtering by pose names if requested
            if filter_re and isinstance(payload, dict):

                poses = payload.get('poses') or payload.get('pose') or []

                iris_pose = next(
        		(p for p in poses
         		if isinstance(p, dict) and p.get('name') == exact_name),
        		None
    		)

                if iris_pose is None:
                        continue

                payload = iris_pose

            # Print output
            print(f"[{ts}] Topic: {topic}")
            if payload is None:
                print("  <non-json payload>")
            else:
                if args.pretty:
                    print(json.dumps(payload, indent=2))
                else:
                    print(json.dumps(payload))
            sys.stdout.flush()

    except KeyboardInterrupt:
        print('\nStopping listener')
    finally:
        sock.close()
        ctx.term()


if __name__ == '__main__':
    main()
