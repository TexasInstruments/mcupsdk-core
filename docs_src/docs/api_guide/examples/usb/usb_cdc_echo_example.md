# USB CDC Echo Example {#EXAMPLES_USB_CDC_ECHO}

[TOC]

# Introduction

This example is a USB device CDC-ACM (Communication Device Class - Abstract Control Model) application based on USB CDC class from TinyUSB.

The example does the below
- Initializes the TinyUSB USB core driver and CDC class.
- Create two virtual COM ports visible to a USB host.
- Any alphabetic input from the USB host will be echo-ed back in lower case on one COM port and in upper case in the other COM port.

# USB Logging Template Example 

- This example also demonstrates how to enable logging for USB using shared memory log feature.
	- The R5FSS0_0_freertos/nortos core will write logs in the shared memory and R5FSS0_1_freertos/nortos core will read and print it on UART0.
- see \ref USB_DEVICE_DRIVER for more information on how to enable USB logs.
- see \ref KERNEL_DPL_DEBUG_PAGE for more information on how to enable shared memory log feature.

\note 
	- Enabling logs in debug build may lead to unwanted behaviour as the application code is build with -O0 flag. 
	- It is recommended to use **release** build when USB logging is enabled. 

# Supported Combinations {#EXAMPLES_USB_CDC_ECHO_EXAMPLE_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 ^				| r5fss0_1_freertos log Reader
 ^              | r5fss0-0_nortos
 ^              | r5fss0-1_nortos   log Reader
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/usb/device/cdc_echo

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 ^				| r5fss0_1_freertos
 ^              | r5fss0-0_nortos
 ^              | r5fss0-1_nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/usb/device/cdc_echo

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME
 Example folder | examples/usb/device/cdc_echo

\endcond

# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## HW Setup

\cond SOC_AM64X

- To test the application, one can use a Windows/Linux PC as a USB host
- Connect the J24 on AM64x/AM243x EVM to the USB host

  \imageStyle{am64x_am243x_evm_j24.png,width:30%}
  \image html am64x_am243x_evm_j24.png USB 2.0 Host/Device Connector

\endcond

\cond SOC_AM243X

### AM243X-EVM
- To test the application, one can use a Windows/Linux PC as a USB host
- Connect the J24 on AM64x/AM243x EVM to the USB host

  \imageStyle{am64x_am243x_evm_j24.png,width:30%}
  \image html am64x_am243x_evm_j24.png USB 2.0 Host/Device Connector

### AM243X-LP
- To test the application, one can use a Windows/Linux PC as a USB host
- Connect the J10 on AM243x LP to the USB host
- USB2.0 is validated from Type C connector

\note The am243x-LP board is powered by the same J10 connector. It requires 
USB-type C port that has power delivery classification. 
- Thunderbolt 
- Battery behind USB logo. 
refer am243x-LP [User Guide](https://www.ti.com/lit/ug/spruj12c/spruj12c.pdf?ts=1677756057987&ref_url=https%253A%252F%252Fwww.google.com%252F)

  \imageStyle{am243x_lp_j10.png,width:30%}
  \image html am243x_lp_j10.png USB Type-C Host/Device Connector

\endcond

\cond SOC_AM261X

### AM261x-LP
- To test the application, one can use a Windows/Linux PC as a USB host.
- Connect the J10 on AM261x-LP to the USB host.

  \imageStyle{am261x_lp_j10.png,width:30%}
  \image html am261x_lp_j10.png USB Type-C Device Connector

\endcond

## Run the example

- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

- When the application is running, two COM ports will be enumerated on the USB host

### For Window 10

- The two enumerated COM ports can be displayed using the Device Manager on Windows 10,

\imageStyle{usb_cdc_echo_example_enum.png,width:50%}
\image html usb_cdc_echo_example_enum.png COM ports (COM3 and COM4) displayed using Device Manager

- On USB host, use any serial port communication program, like Tera-Term, to connect to those two COM ports

\imageStyle{usb_cdc_echo_example_com3.png,width:30%}
\image html usb_cdc_echo_example_com3.png Using Tera Term to connect to port 1

\imageStyle{usb_cdc_echo_example_com4.png,width:30%}
\image html usb_cdc_echo_example_com4.png Using Tera Term to connect to port 2

- Type in any alphabetic characters, the lower case of the input will be displayed on one COM port and the upper case of the input will be displayed on the other COM port

\imageStyle{usb_cdc_echo_example_lower.png,width:30%}
\image html usb_cdc_echo_example_lower.png Lower Case Input Echo-ed

\imageStyle{usb_cdc_echo_example_upper.png,width:30%}
\image html usb_cdc_echo_example_upper.png Upper Case Input Echo-ed

### For Linux

- The two enumerated serial ports can be enumerated as "/dev/ttyACM0" and "/dev/ttyACM1"
- On Linux, one has to change the access right for those two serial ports as shown below.
- Then use any serial port communication program, like putty, to connect to those two serial ports

\imageStyle{usb_cdc_echo_linux_ttyacm0.png,width:30%}
\image html usb_cdc_echo_linux_ttyacm0.png Using putty to connect to port 1

\imageStyle{usb_cdc_echo_linux_ttyacm1.png,width:30%}
\image html usb_cdc_echo_linux_ttyacm1.png Using putty to connect to port 2

- Type in any alphabetic characters, the lower case of the input will be displayed on one COM port and the upper case of the input will be displayed on the other COM port

\imageStyle{usb_cdc_echo_linux_lower.png,width:30%}
\image html usb_cdc_echo_linux_lower.png Lower Case Input Echo-ed

\imageStyle{usb_cdc_echo_linux_upper.png,width:30%}
\image html usb_cdc_echo_linux_upper.png Upper Case Input Echo-ed

# See Also

\ref USB_DEVICE_DRIVER
