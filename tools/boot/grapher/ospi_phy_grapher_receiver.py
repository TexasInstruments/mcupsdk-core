#!/usr/bin/env python3
"""
OSPI PHY Grapher UART Receiver - XMODEM1k Implementation

This script receives PHY tuning data from the embedded device over UART using XMODEM1k protocol.

Protocol Overview:
1. Receive "READY_FOR_SWEEP\r\n" signal
2. Send 32-byte command header (magic 0x47524150)
3. Receive 81,920 bytes of PHY data via XMODEM1k blocks
4. Receive 16-byte status response (magic 0x47524152)
5. Save data to binary file for analysis/plotting

XMODEM1k Frame Format:
- Byte 0: STX (0x02)
- Byte 1: Block number (0-255)
- Byte 2: Block number complement (~blockNum)
- Bytes 3-1026: Data payload (1024 bytes)
- Bytes 1027-1028: CRC16 (big-endian: high byte, low byte)
Total: 1029 bytes per frame
"""

import serial
import struct
import sys
import time
from pathlib import Path


class CRC16CCITT:
    """CRC16-CCITT calculator for XMODEM protocol."""

    # CRC16-CCITT lookup table (polynomial 0x1021)
    TABLE = [
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
        0x1231, 0x0210, 0x3273, 0x2252, 0x5295, 0x42B4, 0x7297, 0x62B6,
        0x9339, 0x8318, 0xB37B, 0xA35A, 0xD39D, 0xC3BC, 0xF39F, 0xE3BE,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
        0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76F7, 0x66D6, 0x56B5, 0x4694,
        0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD7DD, 0xC7FC,
        0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
        0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
        0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
        0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
        0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
        0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
        0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
        0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
        0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
        0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
        0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
        0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
        0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
        0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
        0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
        0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
    ]

    @staticmethod
    def calc(data):
        """Calculate CRC16-CCITT for data buffer."""
        crc = 0
        for byte in data:
            crc = ((crc << 8) ^ CRC16CCITT.TABLE[(crc >> 8) ^ byte]) & 0xFFFF
        return crc & 0xFFFF


class OspiPhyGrapherReceiver:
    """OSPI PHY Grapher UART receiver using XMODEM1k protocol."""

    # Protocol constants
    MAGIC_REQUEST = 0x47524150  # "PRAG"
    MAGIC_RESPONSE = 0x47524152  # "RARG"
    OPTYPE_PHY_TUNE = 0xFD
    DATA_SIZE = 81920  # 5 × 128 × 128
    HEADER_SIZE = 32
    STATUS_SIZE = 16

    # XMODEM constants
    XMODEM_STX = 0x02
    XMODEM_EOT = 0x04
    XMODEM_ACK = 0x06
    XMODEM_NAK = 0x15
    XMODEM_BLOCK_SIZE = 1024
    XMODEM_FRAME_SIZE = 1029  # STX + BlkNum + Complement + Data + CRC (2)
    XMODEM_MAX_RETRIES = 10
    XMODEM_TIMEOUT_MS = 5000

    def __init__(self, port, baudrate=115200, timeout=5):
        """
        Initialize UART connection.

        Args:
            port: Serial port name (e.g., '/dev/ttyUSB0')
            baudrate: Baud rate (default 115200)
            timeout: Read timeout in seconds
        """
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.data = bytearray()
        print(f"[INFO] Connected to {port} at {baudrate} baud")

    def close(self):
        """Close UART connection."""
        if self.ser.is_open:
            self.ser.close()

    def read_byte(self, timeout_ms=1000):
        """
        Read single byte with timeout.

        Args:
            timeout_ms: Timeout in milliseconds

        Returns:
            Byte value or None on timeout
        """
        old_timeout = self.ser.timeout
        self.ser.timeout = timeout_ms / 1000.0
        byte_data = self.ser.read(1)
        self.ser.timeout = old_timeout

        if byte_data:
            return byte_data[0]
        return None

    def read_bytes(self, count, timeout_ms=1000):
        """
        Read multiple bytes with timeout.

        Args:
            count: Number of bytes to read
            timeout_ms: Timeout in milliseconds

        Returns:
            Bytes or None on timeout
        """
        old_timeout = self.ser.timeout
        self.ser.timeout = timeout_ms / 1000.0
        data = self.ser.read(count)
        self.ser.timeout = old_timeout

        if len(data) == count:
            return data
        return None

    def write_bytes(self, data):
        """Write bytes to UART."""
        self.ser.write(data)

    def wait_ready_signal(self):
        """Wait for 'READY_FOR_SWEEP\\r\\n' signal from device, ignoring bootloader logs."""
        print("[INFO] Waiting for ready signal (ignoring bootloader logs)...")

        # Read until we find the ready message
        buffer = b''
        ready_msg = b'READY_FOR_SWEEP\r\n'
        timeout_count = 0
        line_buffer = b''

        while True:
            byte = self.read_byte(timeout_ms=1000)
            if byte is None:
                timeout_count += 1
                if timeout_count > 30:  # 30 second timeout
                    print("[ERROR] Timeout waiting for ready signal")
                    return False
                continue

            buffer += bytes([byte])
            line_buffer += bytes([byte])

            # Print complete lines (for debugging bootloader output)
            if byte == ord(b'\n'):
                try:
                    line_text = line_buffer.decode('ascii', errors='ignore').strip()
                    if line_text and 'READY_FOR_SWEEP' not in line_text:
                        print(f"[SKIP] {line_text}")
                except:
                    pass
                line_buffer = b''

            # Check if we have the ready message
            if ready_msg in buffer:
                print(f"[SUCCESS] Received ready signal!")
                # Flush any remaining data in the receive buffer
                self.ser.reset_input_buffer()
                return True

    def send_command(self):
        """Send 32-byte command header to device."""
        time.sleep(0.5)  # Wait for device to be ready

        cmd = bytearray(32)

        # Bytes 0-3: Magic number (little-endian)
        magic = self.MAGIC_REQUEST
        cmd[0] = (magic >> 0) & 0xFF
        cmd[1] = (magic >> 8) & 0xFF
        cmd[2] = (magic >> 16) & 0xFF
        cmd[3] = (magic >> 24) & 0xFF

        # Byte 4: Operation type
        cmd[4] = self.OPTYPE_PHY_TUNE

        # Bytes 12-15: Data size (little-endian)
        size = self.DATA_SIZE
        cmd[12] = (size >> 0) & 0xFF
        cmd[13] = (size >> 8) & 0xFF
        cmd[14] = (size >> 16) & 0xFF
        cmd[15] = (size >> 24) & 0xFF

        self.ser.write(cmd)
        self.ser.flush()  # Ensure data is sent
        time.sleep(1.0)  # Wait 1 second for device to receive bytes

        print(f"[SUCCESS] Sent command: magic=0x{magic:08X}, optype=0x{cmd[4]:02X}, size={size}")

    def receive_xmodem_data(self):
        """
        Receive data via XMODEM1k protocol.

        Returns:
            True on success, False on error
        """
        print(f"[INFO] Receiving XMODEM1k data ({self.DATA_SIZE} bytes, {self.DATA_SIZE // self.XMODEM_BLOCK_SIZE} blocks)...")

        self.data = bytearray()
        block_num = 1
        received_blocks = 0

        while len(self.data) < self.DATA_SIZE:
            # Read frame header
            frame_header = self.read_bytes(3, timeout_ms=self.XMODEM_TIMEOUT_MS)
            if frame_header is None:
                print(f"[ERROR] Timeout reading frame header")
                return False

            # Verify header length
            if len(frame_header) < 3:
                print(f"[ERROR] Incomplete frame header: got {len(frame_header)} bytes, expected 3")
                return False

            # Check STX
            if frame_header[0] != self.XMODEM_STX:
                print(f"[ERROR] Invalid STX: 0x{frame_header[0]:02X}")
                return False

            frame_block_num = frame_header[1]
            frame_block_complement = frame_header[2]

            # Verify block number
            if (frame_block_num ^ frame_block_complement) != 0xFF:
                print(f"[ERROR] Block number mismatch: {frame_block_num} vs ~{frame_block_complement}")
                self.write_bytes(bytes([self.XMODEM_NAK]))
                continue

            # Read data payload
            frame_data = self.read_bytes(self.XMODEM_BLOCK_SIZE, timeout_ms=self.XMODEM_TIMEOUT_MS)
            if frame_data is None:
                print(f"[ERROR] Timeout reading frame data")
                self.write_bytes(bytes([self.XMODEM_NAK]))
                continue

            # Verify data length
            if len(frame_data) < self.XMODEM_BLOCK_SIZE:
                print(f"[ERROR] Incomplete frame data: got {len(frame_data)} bytes, expected {self.XMODEM_BLOCK_SIZE}")
                self.write_bytes(bytes([self.XMODEM_NAK]))
                continue

            # Read CRC (big-endian)
            frame_crc_bytes = self.read_bytes(2, timeout_ms=self.XMODEM_TIMEOUT_MS)
            if frame_crc_bytes is None:
                print(f"[ERROR] Timeout reading CRC")
                self.write_bytes(bytes([self.XMODEM_NAK]))
                continue

            # Verify CRC length
            if len(frame_crc_bytes) < 2:
                print(f"[ERROR] Incomplete CRC: got {len(frame_crc_bytes)} bytes, expected 2")
                self.write_bytes(bytes([self.XMODEM_NAK]))
                continue

            frame_crc = (frame_crc_bytes[0] << 8) | frame_crc_bytes[1]

            # Calculate and verify CRC
            calc_crc = CRC16CCITT.calc(frame_data)
            if calc_crc != frame_crc:
                print(f"[WARN] CRC mismatch for block {frame_block_num}: calc=0x{calc_crc:04X}, recv=0x{frame_crc:04X}")
                self.write_bytes(bytes([self.XMODEM_NAK]))
                continue

            # Block accepted - append data
            to_add = min(self.XMODEM_BLOCK_SIZE, self.DATA_SIZE - len(self.data))
            self.data.extend(frame_data[:to_add])

            received_blocks += 1
            progress = (len(self.data) / self.DATA_SIZE) * 100
            print(f"[INFO] Block {frame_block_num}: received {len(self.data)}/{self.DATA_SIZE} bytes ({progress:.1f}%)")

            # Send ACK
            self.write_bytes(bytes([self.XMODEM_ACK]))

        # Wait for EOT
        eot = self.read_byte(timeout_ms=self.XMODEM_TIMEOUT_MS)
        if eot is None:
            print(f"[ERROR] Timeout waiting for EOT")
            return False
        if eot != self.XMODEM_EOT:
            print(f"[ERROR] Expected EOT (0x04), got 0x{eot:02X}")
            return False

        print(f"[INFO] Received EOT")

        # Send final ACK
        self.write_bytes(bytes([self.XMODEM_ACK]))

        print(f"[SUCCESS] XMODEM1k reception complete: {len(self.data)} bytes in {received_blocks} blocks")
        return True

    def receive_status(self):
        """
        Receive and validate 16-byte status response.

        Returns:
            Status code or None on error
        """
        print("[INFO] Waiting for status response...")

        status_data = self.read_bytes(self.STATUS_SIZE, timeout_ms=self.XMODEM_TIMEOUT_MS)
        if status_data is None:
            print("[ERROR] Timeout reading status")
            return None

        # Extract magic (little-endian)
        magic = (status_data[0] |
                (status_data[1] << 8) |
                (status_data[2] << 16) |
                (status_data[3] << 24))

        if magic != self.MAGIC_RESPONSE:
            print(f"[ERROR] Invalid status magic: 0x{magic:08X} (expected 0x{self.MAGIC_RESPONSE:08X})")
            return None

        # Extract status code (little-endian)
        status_code = (status_data[4] |
                      (status_data[5] << 8) |
                      (status_data[6] << 16) |
                      (status_data[7] << 24))

        print(f"[SUCCESS] Status received: 0x{status_code:08X}")
        return status_code

    def save_data(self, filename):
        """Save received data to binary file."""
        with open(filename, 'wb') as f:
            f.write(self.data)
        print(f"[SUCCESS] Data saved to {filename} ({len(self.data)} bytes)")

    def run(self, output_file="ospi_phy_grapher_data.bin"):
        """
        Execute complete reception sequence.

        Args:
            output_file: Output filename for received data

        Returns:
            True on success, False on error
        """
        try:
            # Step 1: Wait for ready signal
            if not self.wait_ready_signal():
                return False

            # Give device time to settle after sending ready signal
            time.sleep(1.0)
            print("[DEBUG] Waiting 1 second for device to be ready...")

            # Step 2: Send command
            self.send_command()
            # Wait for PHY sweep to complete (~30 sec) + device processing + margin
            print("[DEBUG] Waiting for PHY sweep to complete (~5 seconds)...")
            time.sleep(5)

            # Step 3: Receive XMODEM data
            if not self.receive_xmodem_data():
                return False

            # Step 4: Receive status
            status = self.receive_status()
            if status is None:
                return False

            # Step 5: Save data
            self.save_data(output_file)

            print("[SUCCESS] Complete OSPI PHY Grapher UART reception successful!")
            return True

        except Exception as e:
            import traceback
            print(f"[ERROR] Exception during reception: {e}")
            traceback.print_exc()
            return False


def main():
    """Command-line interface for receiver."""
    import argparse

    parser = argparse.ArgumentParser(
        description='OSPI PHY Grapher UART Receiver - XMODEM1k Implementation',
        epilog='''
IMPORTANT: Use CONFIG_UART1 port (not CONFIG_UART0)!
- CONFIG_UART0: Debug logs
- CONFIG_UART1: Binary data transfer (this script should connect here)

Example:
  python3 ospi_phy_grapher_receiver.py /dev/ttyUSB2 -o data.bin
  # /dev/ttyUSB2 should be CONFIG_UART1 (data port)
  # Monitor CONFIG_UART0 (debug) on a separate terminal for diagnostics
        '''
    )
    parser.add_argument('port', help='Serial port for DATA (CONFIG_UART1, e.g., /dev/ttyUSB2)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    parser.add_argument('-o', '--output', default='ospi_phy_grapher_data.bin',
                       help='Output file (default: ospi_phy_grapher_data.bin)')
    parser.add_argument('-t', '--timeout', type=int, default=5,
                       help='Read timeout in seconds (default: 5)')

    args = parser.parse_args()

    # Create receiver and run
    receiver = OspiPhyGrapherReceiver(args.port, args.baudrate, args.timeout)

    try:
        success = receiver.run(args.output)
        sys.exit(0 if success else 1)
    finally:
        receiver.close()


if __name__ == '__main__':
    main()
