# USB {#USB_DEVICE_DRIVER}

[TOC]

## Introduction

The USB SW stack in MCU+ SDK is shown below.

\imageStyle{usb_cdc_tinyusb_arch.png,width:30%}
\image html usb_cdc_tinyusb_arch.png "USB SW Stack Block Diagram"

There are three layers as listed below,
- TinyUSB: This is the USB stack itself and acts as the interface for user application to USB
- USB Device Driver: This is the USB HW and SOC specific device drivers
- TinyUSB Porting Layer: This is the layer which binds the open source TinyUSB stack to the SOC specific device driver

We encourage customers to use USB peripheral from tinyUSB APIs. The \ref EXAMPLES_USB_CDC_ECHO is using the CDC class API from TinyUSB. This way, customers can easily adapt to other TinyUSB examples.

For advanced customers or 3P USB stack vendors, refer to the USB device driver references listed below.

## Features Supported

### TinyUSB

TinyUSB is an open-source cross-platform USB Host/Device stack for embedded system, designed to be memory-safe with no dynamic allocation and thread-safe with all interrupt events are deferred then handled in the non-ISR task function.

Below features are supported in the SDK
- USB device
- USB CDC Class

### USB Device Driver

The USB device driver is the USB HW specific software component that provides a programming abstraction to the Cadence USB IP
included in the SOC and supports below features,

- USB device instance creation/deletion
- USB device endpoint handling
- USB device data transfer
- USB device interrupts and DMA

### SoC Porting Layer

The SoC porting layer for USB device driver is the glue layer between the SoC and the USB device driver
and does the below,

- Pinmux needed for USB device
- Clock source selection needed for USB device
- Clock settings needed for USB device
- Configure the USB HW for USB device mode

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Initialize USB device driver including clock setup and pinmux

## Features Not Supported

- TinyUSB Class Drivers Other Than CDC
- TinyUSB Host Core Driver and Class Drivers
- USB 3.0
- USB 2.0 Host Mode
- USB Dual Role Mode

## Implementation Note

- USB device class driver should be operating as higher priority task if host driver is implemented as polling for device response with strict timeout.

- If Cadence driver is used with TinyUSB, makefiles of both example and driver should define TINYUSB_INTEGRATION (like default cdc_echo example). This macro defined in driver and not in example or vice versa is an invalid combination.

## Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/usb/tinyusb</td></tr>
<tr>
    <td>config/
    <td>TinyUSB Stack configuration</td>
</tr>
<tr>
    <td>portable/
    <td>TinyUSB Stack porting</td>
</tr>
<tr>
    <td>tinyusb-stack/
    <td>TinyUSB Stack source files</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/usb/cdn</td></tr>
<tr>
    <td>core_driver/
    <td>Cadence USB core driver source files</td>
</tr>
<tr>
    <td>doc/
    <td>Cadence USB core driver documents</td>
</tr>
<tr>
    <td>include/
    <td>Cadence USB core driver Soc porting layer API header files</td>
</tr>
<tr>
    <td>soc/
    <td>Cadence USB core driver Soc porting layer source files</td>
</tr>
</table>

## Additional References {#TINYUSB_ADDITIONAL_REFERENCES}

<table>
<tr>
    <th>References
</tr>
<tr><td colspan="1" bgcolor=#F0F0F0> SOC Porting Layer </td></tr>
<tr>
    <td>\ref EXAMPLES_USB_CDC_ECHO
</tr>
<tr><td colspan="1" bgcolor=#F0F0F0> TinyUSB </td></tr>
<tr>
    <td>[TinyUSB Github Project](https://github.com/hathach/tinyusb)
</tr>
<tr><td colspan="1" bgcolor=#F0F0F0> USB Device Driver </td></tr>
<tr>
    <td> <a href="../../source/usb/cdn/doc/usb_ss_drd_driver_quick_start_guide.pdf">USB Device Driver Quick Start Guide</a>
</tr>
<tr>
    <td> <a href="../../source/usb/cdn/doc/api_usage_guide.pdf">USB Device Driver APIs</a>
</tr>
<tr>
    <td> <a href="../../source/usb/cdn/doc/core_driver/usb_ss_drd_driver_guide.pdf">USB Device Driver User's Guide</a>
</tr>
<tr>
    <td> <a href="../../source/usb/cdn/doc/porting/porting_guide.pdf">USB Device Driver Porting Guide</a>
</tr>
<tr><td colspan="1" bgcolor=#F0F0F0> SOC Porting Layer </td></tr>
<tr>
    <td>\ref USB_MODULE
</tr>
</table>

