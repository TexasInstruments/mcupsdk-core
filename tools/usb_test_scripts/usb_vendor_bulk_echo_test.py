import usb.core
import usb.backend.libusb1
import sys
import time
import os
import threading

# Path to libusb-1.0.dll - assumed to be in the same directory as this script.
# Change this path based on your setup if the DLL is located elsewhere.
_script_dir     = os.path.dirname(os.path.abspath(__file__))
LIBUSB_DLL_PATH = os.path.join(_script_dir, "libusb-1.0.dll")

backend = usb.backend.libusb1.get_backend(find_library=lambda x: LIBUSB_DLL_PATH)

if backend is None:
    print("ERROR: Could not load libusb backend.")
    print(f"Ensure libusb-1.0.dll is present at: {LIBUSB_DLL_PATH}")
    sys.exit(1)

# USB device identifiers for AM261x Vendor Bulk example
USB_VID = 0x0451   # Texas Instruments
USB_PID = 0x6170   # AM261x Vendor
EP_OUT  = 0x01     # Bulk OUT endpoint (host to device)
EP_IN   = 0x81     # Bulk IN  endpoint (device to host)


def find_device():
    """Locate the AM261x USB vendor device on the host."""
    print(f"Looking for USB device VID=0x{USB_VID:04X}, PID=0x{USB_PID:04X}...")
    dev = usb.core.find(idVendor=USB_VID, idProduct=USB_PID, backend=backend)
    if dev is None:
        raise ValueError("Device not found. Make sure the board is running the example and connected via USB.")
    print("Device found.")
    try:
        print(f"  Manufacturer : {dev.manufacturer}")
        print(f"  Product      : {dev.product}")
    except Exception:
        print("  (Could not read device strings - WinUSB driver may not be installed)")
    return dev


def setup_device(dev):
    """Configure the USB device and print endpoint information."""
    try:
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
    except (NotImplementedError, Exception):
        pass

    try:
        dev.set_configuration()
    except Exception as e:
        print(f"  Configuration warning: {e}")

    try:
        cfg  = dev.get_active_configuration()
        intf = cfg[(0, 0)]
        print(f"  Interface : {intf.bInterfaceNumber}")
        print("  Endpoints :")
        for ep in intf:
            print(f"    Address 0x{ep.bEndpointAddress:02X}  MaxPacketSize {ep.wMaxPacketSize}")
    except Exception as e:
        print(f"  Could not read configuration: {e}")

    return dev


def bulk_transfer(dev, data, timeout=10000):
    """
    Send data to the device and receive the echo concurrently.

    The receiver thread starts before the sender so it is already
    listening when data arrives. This keeps the device TX buffer
    drained throughout the transfer and avoids stalls on large payloads.
    """
    received_data = bytearray()
    send_error    = [None]
    recv_error    = [None]
    bytes_sent    = [0]

    def sender_thread():
        try:
            bytes_sent[0] = dev.write(EP_OUT, data, timeout)
        except usb.core.USBError as e:
            send_error[0] = e

    def receiver_thread():
        """Read echoed data until all expected bytes are received or retries are exhausted."""
        expected      = len(data)
        no_data_count = 0

        while len(received_data) < expected and no_data_count < 10:
            try:
                remaining = expected - len(received_data)
                chunk = dev.read(EP_IN, min(remaining, 4096), timeout=2000)
                if len(chunk) > 0:
                    received_data.extend(chunk)
                    no_data_count = 0
                else:
                    no_data_count += 1
            except usb.core.USBError as e:
                if 'timed out' in str(e).lower() or e.errno == 110:
                    no_data_count += 1
                else:
                    recv_error[0] = e
                    break

    receiver = threading.Thread(target=receiver_thread)
    sender   = threading.Thread(target=sender_thread)

    receiver.start()
    sender.start()

    sender.join()
    receiver.join()

    print(f"  Sent     : {bytes_sent[0]} bytes")
    print(f"  Received : {len(received_data)} bytes")

    if send_error[0]:
        print(f"  Send error    : {send_error[0]}")
        return False, 0
    if recv_error[0]:
        print(f"  Receive error : {recv_error[0]}")
        return False, 0

    sent_data = data[:bytes_sent[0]]

    if bytes(received_data) == sent_data:
        return True, bytes_sent[0]
    else:
        if bytes_sent[0] != len(received_data):
            print(f"  Length mismatch : sent {bytes_sent[0]}, received {len(received_data)}")
        else:
            for i in range(len(received_data)):
                if sent_data[i] != received_data[i]:
                    print(f"  Data mismatch at byte {i}: "
                          f"sent 0x{sent_data[i]:02X}, received 0x{received_data[i]:02X}")
                    break
        return False, bytes_sent[0]


def run_test(dev, test_name, data):
    """Execute a single bulk echo transfer test and report the result."""
    print(f"\nTest : {test_name}")
    print(f"  Size : {len(data)} bytes")

    start  = time.time()
    passed, actual_bytes = bulk_transfer(dev, data)
    elapsed = time.time() - start

    if passed:
        throughput = actual_bytes / elapsed / 1024 if elapsed > 0 else 0
        print(f"  Result : PASS  ({elapsed:.3f}s, {throughput:.1f} KB/s)")
    else:
        print(f"  Result : FAIL")

    return passed


def make_test_data(num_packets):
    """
    Build test payload as a sequence of 512-byte packets.
    Each packet contains a 2-byte big-endian sequence number followed by 0xAA fill bytes.
    """
    data = bytearray()
    for i in range(num_packets):
        packet    = bytearray(512)
        packet[0] = (i >> 8) & 0xFF
        packet[1] = i & 0xFF
        for j in range(2, 512):
            packet[j] = 0xAA
        data.extend(packet)
    return bytes(data)


def main():
    print("=" * 60)
    print("USB Vendor Bulk Echo Test")
    print("=" * 60)

    try:
        dev = find_device()
        dev = setup_device(dev)

        print("\n" + "=" * 60)
        print("Running Bulk Transfer Tests")
        print("=" * 60)

        results = []
        results.append(run_test(dev, "128 KB", make_test_data(256)))
        results.append(run_test(dev, "256 KB", make_test_data(512)))
        results.append(run_test(dev, "512 KB", make_test_data(1024)))

        print("\n" + "=" * 60)
        print("Test Summary")
        print("=" * 60)
        passed = sum(results)
        total  = len(results)
        print(f"Passed : {passed}/{total}")
        if passed == total:
            print("All tests PASSED.")
        else:
            print("Some tests FAILED.")

    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)
    except usb.core.USBError as e:
        print(f"USB Error: {e}")
        print("Install the WinUSB driver using the Zadig tool before running this script.")
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
