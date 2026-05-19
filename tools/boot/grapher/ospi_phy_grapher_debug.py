#!/usr/bin/env python3
"""
OSPI PHY Grapher Debug Tool - Diagnose communication issues

This script helps debug UART communication problems by:
1. Monitoring raw serial data
2. Detecting protocol misalignment
3. Identifying what the device is actually sending
"""

import serial
import sys
import time
from pathlib import Path


def main():
    """Monitor raw UART data."""
    import argparse

    parser = argparse.ArgumentParser(
        description='OSPI PHY Grapher Debug - Monitor raw UART data'
    )
    parser.add_argument('port', help='Serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    parser.add_argument('-t', '--timeout', type=int, default=10,
                       help='Read timeout in seconds (default: 10)')
    parser.add_argument('-n', '--lines', type=int, default=50,
                       help='Number of lines to capture (default: 50)')
    parser.add_argument('--raw', action='store_true',
                       help='Show raw hex values')

    args = parser.parse_args()

    print(f"[INFO] Connecting to {args.port} at {args.baudrate} baud...")
    print(f"[INFO] Reading {args.lines} lines with {args.timeout}s timeout")
    print(f"[INFO] Waiting for data (make sure device is running)...\n")

    try:
        ser = serial.Serial(args.port, args.baudrate, timeout=args.timeout)
        time.sleep(1)  # Allow connection to stabilize

        line_count = 0
        buffer = b''

        while line_count < args.lines:
            byte_data = ser.read(1)

            if not byte_data:
                print("[TIMEOUT] No more data received")
                break

            byte_val = byte_data[0]
            buffer += byte_data

            # Print on newline or if we have 100 bytes
            if byte_val == ord('\n') or len(buffer) >= 100:
                try:
                    # Try to decode as ASCII
                    text = buffer.decode('ascii', errors='replace')
                    print(f"[DATA] {text.rstrip()}")
                except:
                    pass

                if args.raw:
                    hex_str = ' '.join(f'{b:02X}' for b in buffer)
                    print(f"[HEX]  {hex_str}")

                line_count += 1
                buffer = b''

        ser.close()
        print(f"\n[SUCCESS] Capture complete ({line_count} lines)")

        # Diagnostics
        print("\n" + "="*70)
        print("Diagnostics:")
        print("="*70)

        if b'READY_FOR_SWEEP' in buffer or b'[' in buffer:
            print("✅ Device is sending text debug output")
            print("   → Make sure device is waiting for command (check device logs)")
            print("   → May need to wait longer for PHY sweep to start")
        elif byte_val == 0x02:
            print("✅ Detected XMODEM STX byte (0x02)")
            print("   → Device is ready to send XMODEM data")
        else:
            print(f"⚠️  Unexpected data: 0x{byte_val:02X}")
            print("   → Check device is running and UART connections are correct")
            print("   → Verify baud rate matches device configuration")

    except FileNotFoundError:
        print(f"[ERROR] Port not found: {args.port}")
        print("Available ports: Check /dev/ttyUSB* or /dev/ttyACM*")
        sys.exit(1)
    except Exception as e:
        print(f"[ERROR] Exception: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
