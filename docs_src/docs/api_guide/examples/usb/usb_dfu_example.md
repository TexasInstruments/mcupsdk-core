
# USB DFU Example {#EXAMPLES_USB_DFU}

[TOC]

# Introduction

- This example demonstrates the use of **TinyUSB** **DFU DEV Class driver**. Its implements all the necessary callbacks that are 
required by TinyUSB DFU (Device Firmware Upgrade) DEV class driver. 

- When user issues DFU download command then the application receives the data in a static buffer via USB DFU. Once the manifest 
state is completed the application sends the received data back to UART0. 

\cond SOC_AM64X || SOC_AM243X
- When user issues DFU Upload command it sends the **Hello world from AM64x-AM243x DFU! - Partition 0** string back to the HOST PC. 
\endcond

\cond SOC_AM261X
- When user issues DFU Upload command it sends the **Hello world from AM261x DFU! - Partition 0** string back to the HOST PC. 
\endcond

- refer [USB2.0 DFU specs](https://www.usb.org/sites/default/files/DFU_1.1.pdf) to know more about USB 2.0 DFU class. 

\note The max buffer size is 4 KB thus, as far as this example is concerned user can send at max 4 KB of data. 

- To enable USB Logging for this example refer \ref EXAMPLES_USB_CDC_ECHO

# Supported Combinations {#EXAMPLES_USB_DFU_EXAMPLE_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/usb/device/dfu

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/usb/device/dfu

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_LP_BOARD_NAME
 Example folder | examples/usb/device/dfu

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

- When the application is running. Observer DFU device detected on HOST PC. 

- Open cmd in windows and terminal in case of Linux. Run the following command to detect whether a DFU device has been enumerated or not. 

	**Windows** 

        dfu-util -l 

	**Linux** 

        sudo dfu-util -l 

- If the enumeration is successful the following should be displayed on console. 

\imageStyle{usb_dfu_enum.png,width:50%}

\cond SOC_AM261X
\image html usb_dfu_enum.png AM64x-AM243x DFU Device detected. 
\endcond


\cond SOC_AM243X || SOC_AM64X
\image html usb_dfu_enum.png AM64x-AM243x DFU Device detected.
\endcond

- If the DFU device is not detected then most probably the WinUSB driver is not installed. 
- Refer "Install steps for dfu-util tools(windows)" in \ref SDK_DOWNLOAD_PAGE 

#### DFU Download 

- Open COM port in windows and /dev/ttyUSBx in Linux which opens console for UART0. 
- Once the DFU device is enumerated run the following command. 

	**Windows** 

        dfu-util -a 0 -i 0 -D test_data.txt


	**Linux** 

		sudo dfu-util -a 0 -i 0 -D test_data.txt

- The *test_data.txt* file contains a string data that is to be transferred. 
- Once the DFU transfer is completed following will be displayed on the terminal. 


\imageStyle{dfu_cmd.png,width:50%}
\image html dfu_cmd.png DFU Download command successful

- Open UART0 serial port to observe the content of `test_data.txt` file displayed on same serial port. 

\imageStyle{dfu_download.png,width:50%}
\image html dfu_download.png Content of received file displayed on serial port. 

#### DFU Upload 

- Run the following command to read the data from DFU device. 

	**Windows** 

		dfu-util -a 0 -i 0 -U upload_data.txt 

	**Linux** 

		sudo dfu-util -a 0 -i 0 -U upload_data.txt

- Once the DFU Upload transaction is successful , open `upload_data.txt` file to see the contents 

\cond SOC_AM243X || AM64X
\imageStyle{upload_data.png,width:30%}
\image html upload_data.png Data sent by DFU device to DFU host. 
\endcond

\cond SOC_AM261X
\imageStyle{am261x_dfu_upload_data.png,width:30%}
\image html am261x_dfu_upload_data.png Data sent by DFU device to DFU host. 
\endcond
# See Also

\ref USB_DEVICE_DRIVER
