# USB Vendor Bulk Echo Example {#EXAMPLES_USB_VENDOR_ECHO}

[TOC]

# Introduction

This example is a USB device Vendor class application based on TinyUSB.

The example does the below
- Initializes the TinyUSB USB core driver and Vendor class.
- Exposes a vendor-specific bulk endpoint pair (IN and OUT) to the USB host.
- Any data sent by the USB host on the OUT endpoint is echoed back on the IN endpoint.

# Supported Combinations {#EXAMPLES_USB_VENDOR_ECHO_EXAMPLE_COMBOS}

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_nortos
 ^              | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/usb/device/vendor_bulk_echo

\endcond

# Steps to Run the Example

## Build the example

\if SOC_AM261X
- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
\endif

\cond SOC_AM261X
\note SysConfig will generate an empty `ti_usb_descriptor.c` for this example. The actual USB
descriptors are provided by `usb_descriptors.c` which is part of the example source.
\endcond

## HW Setup

\cond SOC_AM261X

### AM261x-LP
- To test the application, use a Windows PC as a USB host.
- Connect the **J10** (Micro B USB Device connector) on AM261x-LP to the USB host.

  \imageStyle{am261x_lp_j10.png,width:30%}
  \image html am261x_lp_j10.png USB Micro B Device Connector (J10)

\endcond

## Run the example

\if SOC_AM261X
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
\endif

### Install WinUSB Driver (Windows)

A vendor-specific USB device has no built-in OS driver. On first connection, Windows will show
the device as **Unknown device** in Device Manager.

  \imageStyle{usb_vendor_bulk_unknown_device.png,width:40%}
  \image html usb_vendor_bulk_unknown_device.png Unknown device shown in Device Manager

To communicate with the device, install the **WinUSB** driver using the **Zadig** tool:

1. Download Zadig from: https://github.com/pbatard/libwdi/releases/download/v1.4.1/zadig-2.7.exe
2. With the example running on the board and the USB cable connected, open Zadig.
3. Click **Options** → **List All Devices**.
4. Select **Unknown Device #1** (or the equivalent entry) from the dropdown.

  \imageStyle{usb_vendor_bulk_zadig.png,width:50%}
  \image html usb_vendor_bulk_zadig.png Selecting the Unknown Device in Zadig

5. Ensure **WinUSB** is selected as the target driver, then click **Install Driver**.

\note The board must be running the example while using Zadig; otherwise the device may not appear.

### Verify Enumeration

After driver installation:

1. Terminate the debug session in CCS.
2. Power-cycle the board.
3. Restart the debug session.

The device should now appear as **AM261x Vendor** under **Universal Serial Bus devices** in Device Manager.

  \imageStyle{usb_vendor_bulk_device_enum.png,width:35%}
  \image html usb_vendor_bulk_device_enum.png AM261x Vendor device enumerated correctly

# Testing the Example

A Python test script is provided in the SDK to verify the bulk echo functionality:

    tools/usb_test_scripts/usb_vendor_bulk_echo_test.py

The script sends three bulk transfer payloads of increasing size (128 KB, 256 KB, 512 KB)
to the device and verifies that the echoed data matches exactly. It reports the result,
elapsed time, and throughput for each test.

## Prerequisites

### 1. Install PyUSB

Install the PyUSB library using pip:

    pip install pyusb

### 2. Download libusb

PyUSB requires the **libusb** backend DLL on Windows.

- Download the latest libusb Windows release from: https://github.com/libusb/libusb/releases
- Extract the archive and locate `libusb-1.0.dll` under `VS2019\MS64\dll\` (for 64-bit systems).
- Place `libusb-1.0.dll` in the same directory as the test script:

        tools/usb_test_scripts/libusb-1.0.dll

\note If your `libusb-1.0.dll` is in a different location, update the `LIBUSB_DLL_PATH`
variable at the top of the script accordingly.

## Running the Script

With the example running on the board and the USB cable connected, run:

    python usb_vendor_bulk_echo_test.py

If all tests pass, the output should look as follows:

\imageStyle{usb_vendor_bulk_test_output.png,width:30%}
\image html usb_vendor_bulk_test_output.png Expected test output when all bulk transfer tests pass

# See Also

\ref USB_DEVICE_DRIVER
