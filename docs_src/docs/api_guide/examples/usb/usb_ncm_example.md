
# USB NCM Example {#EXAMPLES_USB_NCM}

[TOC]

# Introduction {#INTRODUCTION_NCM}

- This examples demonstrates the use of **TinyUSB** **NCM DEV Class driver**. Its implements all the necessary callbacks that are 
required by TinyUSB NCM DEV class driver. 

- The Remote Network Driver Interface Specification (NCM) is a Microsoft proprietary protocol used mostly on top of 
  USB. [NCM_DOC](https://www.usb.org/document-library/network-control-model-devices-specification-v10-and-errata-and-adopters-agreement)

  \image html example_block_diag.png NCM Example Block diagram 

- This example integrates LWIP HTTPD webserver running on r5fss0-0 core with TinyUSB NCM class dirver. The class driver's 
job is to collect the ethernet frames comming over USB and forward it to the Lwip web server and vice versa. 

\note 
In this example we receive one ethernet packet at a time from NCM class driver and forward it to Lwip webeserver. This 
is a simplified design to demonstrate the use of NCM net class driver thus to achive better throughput, further optimizations are required.

- This example also implements a DHCP server which will provide a static ip address to HTTPD web server.Moreover it allocates ip address 
to the NCM Host PC as well. 

- Refer \ref EXAMPLES_ENET_LWIP_CPSW_HTTPSERVER to know more about http web server using lwip( TCP/IP) stack. 

\cond SOC_AM64X
- Once NCM device enumearation is successful, AM64x-EVM will be recognised as a separate network interface. The Network 
interface driver on the Host side will convert pass the ethernet traffic Via USB bus as per NCM protocol specification. 
\endcond

\cond SOC_AM243X
- Once NCM device enumearation is successful, AM243x-EVM will be recognised as a separate network interface. The Network 
interface driver on the Host side will convert and transfer the ethernet traffic Via USB bus as per NCM protocol specification. 
\endcond

- To enable USB Logging for this example refer \ref EXAMPLES_USB_CDC_ECHO

# Supported Combinations {#EXAMPLES_USB_NCM_EXAMPLE_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/usb/device/ncm

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/usb/device/ncm

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

## Run the example

- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

- When application is running observe that ncm network device is detected by HOST PC ( Windows/linux)
- If the enumeration is successful the following should be displayed on console. 

**Windows** 

- Open command prompt and execute **ipconfig**. User should see a new network interface detected. 

  \imageStyle{rndis_win_enum_log.PNG,width:60%}
  \image html rndis_win_enum_log.PNG NCM Device Enumeration log

\note
- NCM Windows driver might be readily available. Follow [link](https://www.driverscape.com/download/cdc-ncm) to download 
and install the NCM host class driver for windows. 

**Linux** 

- Open terminal and execute **dmesg** command. User should see a new network interface over USB detected. 

  \imageStyle{ncm_enum_log.png,width:60%}
  \image html ncm_enum_log.png NCM Device Enumeration log. 

#### NCM Download 

 - Open any web browser application and post following http request. 
	
	http://192.168.7.1 

\note 
- Note that this IP address to host PC and device running lwip webserver will be provided by the dhcp server 
  running along with lwip httpd. Refer \ref INTRODUCTION_NCM


When the http get request will be successful, following HTML page will be display. 

\imageStyle{web_page.png,width:50%}
\image html web_page.png HTTP Get request via USB( NCM ) successful

# See Also

\ref USB_DEVICE_DRIVER
