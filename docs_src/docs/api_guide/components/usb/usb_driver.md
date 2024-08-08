# USB {#USB_DEVICE_DRIVER}

[TOC]

## Introduction

The USB SW stack in MCU+ SDK is shown below.

\cond SOC_AM243X || SOC_AM64X
\imageStyle{usb_cdc_tinyusb_arch.png,width:30%}
\image html usb_cdc_tinyusb_arch.png "USB SW Stack Block Diagram"
\endcond

\cond SOC_AM261X
\imageStyle{usb_cdc_tinyusb_arch_am261x.png,width:30%}
\image html usb_cdc_tinyusb_arch_am261x.png "USB SW Stack Block Diagram"
\endcond

There are three layers as listed below,
- TinyUSB: This is the USB stack itself and acts as the interface for user application to USB.
- USB Device Driver: This is the USB HW and SOC specific device drivers
- TinyUSB Porting Layer: This is the layer which binds the open source TinyUSB stack to the SOC specific device driver

We encourage customers to use USB peripheral from TinyUSB APIs. The \ref EXAMPLES_USB_CDC_ECHO is using the CDC class API from TinyUSB. This way, customers can easily adapt to other TinyUSB examples.

For advanced customers or 3P USB stack vendors, refer to the USB device driver references listed below.

## Features Supported

### TinyUSB

TinyUSB is an open-source cross-platform USB Host/Device stack for embedded system, designed to be memory-safe with no dynamic allocation and thread-safe with all interrupt events are deferred then handled in the non-ISR task function.

Below features are supported in the SDK
- USB device
- USB CDC Class
- USB DFU Class 
\cond SOC_AM243X || SOC_AM64X
- USB NCM Class 
- USB RNDIS Class 
\endcond

\note Supported TinyUSB version is [0.14.0](https://github.com/hathach/tinyusb/tree/0.14.0) 

### USB Device Driver
\cond SOC_AM243X || SOC_AM64X
The USB device driver is the USB HW specific software component that provides a programming abstraction to the Cadence USB IP
included in the SOC and supports below features,
\endcond

\cond SOC_AM261X
The USB device driver is the USB HW specific software component that provides a programming abstraction to the Synopsys dwc3 USB IP
included in the SOC and supports below features,
\endcond

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

\cond SOC_AM243X || SOC_AM64X
- TinyUSB Class Drivers Other Than CDC, DFU, NCM & RNDIS 
\endcond

\cond SOC_AM261X
- TinyUSB Class Drivers Other Than CDC & DFU.
\endcond

- TinyUSB Host Core Driver and Class Drivers
- USB 3.0
- USB 2.0 Host Mode
- USB Dual Role Mode

## Implementation Note

- USB device class driver should be operating as higher priority task if host driver is implemented as polling for device response with strict timeout.

\cond SOC_AM243X || SOC_AM64X
- If Cadence driver is used with TinyUSB, makefiles of both example and driver should define TINYUSB_INTEGRATION (like default cdc_echo example). This macro defined in driver and not in example or vice versa is an invalid combination.
\endcond

\cond SOC_AM261X
- If Synopsys driver is used with TinyUSB, makefiles of both example and driver should define TINYUSB_INTEGRATION (like default cdc_echo example). This macro defined in driver and not in example or vice versa is an invalid combination.
\endcond



## USB Logging 

- User needs to define following macros to enable logs. 
	1. **CFG_TUSB_DEBUG** for enabling logs in **TinyUSB** stack .

	2. On AM243x/AM64x with Cadence drier, use **CFG_CUSB_DEBUG** for enabling logs in USB device driver.
       On AM261x with Synopsys driver, use **DEBUG** for enabling logs in USB device driver.

- By default, logging for both the modules will be enabled in **debug** profile and disabled in **release** profile. 

\cond SOC_AM243X || SOC_AM64X
- There is a provision for enabling and disabling logs for certain modules in USB device driver for better performance. Moreover, user can also define required verbosity level. 
- Appropriate verbosity levels 
	1. **DBG_CRIT** - critical         
	2. **DBG_WARN** - warning
    3. **DBG_FYI** - fyi
    4. **DBG_HIVERB** - highly verbose

- refer to \ref CUSBD_MODULE_IDS to check out module Ids. 
- refer to \ref USB_MODULE for API definitions. 
\endcond

\note 
	- Due to timings issue UART logs may not give expected performance in debug profile as the code is not optimized. If user needs high verbose logs is recommended to use UART logs in **release profile or use shared memory logging.**
	- refer \ref KERNEL_DPL_DEBUG_PAGE for more information on **shared memory logging** and how to use it. 

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
\cond SOC_AM243X || SOC_AM64X
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
\endcond

\cond SOC_AM261X
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/usb/synp</td></tr>
<tr>
    <td>dwc3/
    <td>Synopsys USB core driver source files</td>
</tr>
<tr>
    <td>include/
    <td>Synopsys USB core driver Soc porting layer API header files</td>
</tr>
<tr>
    <td>soc/am261x/
    <td>Device wrapper layer files and USB application init source files</td>
</tr>
\endcond
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
<tr>
    <td>\ref EXAMPLES_USB_DFU
</tr>
\cond SOC_AM243X || SOC_AM64X
<tr>
    <td>\ref EXAMPLES_USB_NCM
</tr>
<tr>
    <td>\ref EXAMPLES_USB_RNDIS
</tr>
\endcond
<tr><td colspan="1" bgcolor=#F0F0F0> TinyUSB </td></tr>
<tr>
    <td>[TinyUSB Github Project](https://github.com/hathach/tinyusb)
</tr>
\cond SOC_AM243X || SOC_AM64X
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

- Constants defining module Ids for all modules in USB Device driver. 
<tr><td colspan="1" bgcolor=#F0F0F0> SOC Porting Layer </td></tr>
<tr>
    <td>\ref USB_MODULE
</tr>
\endcond
</table>

