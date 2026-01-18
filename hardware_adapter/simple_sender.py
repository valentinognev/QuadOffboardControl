#!/usr/bin/env python3
import serial
import struct
import time
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    args = parser.parse_args()
    
    ser = serial.Serial(args.port, 57600)
    print(f"Sending incrementing IDs to {args.port}...")
    
    counter = 0
    while True:
        # Send simple frame: Header (2) + ID (short) + Magic (short) + Padding (32 bytes)
        # Total 38 bytes
        # Packet: <HH32s
        packet = struct.pack('<HH32s', counter % 255 + 1, 0xAAAA, b'\x00'*32)
        header = b'\x77\xaa'
        ser.write(header + packet)
        print(f"Sent ID: {counter % 255 + 1}")
        counter += 1
        time.sleep(0.5)

if __name__ == "__main__":
    main()
