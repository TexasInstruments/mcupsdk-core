# SBL CAN {#EXAMPLES_DRIVERS_SBL_CAN}

[TOC]

# Introduction

This bootloader does SOC initializations and attempts to boot a multicore appimage received over CAN via custom-made protocol (see below). The image file is sent using a python script (See \ref CAN_BOOTLOADER_PYTHON_SCRIPT). Once image is received, the SBL then parses it, splits it into RPRCs for each core applicable. Each core is then initialized, RPRC image is loaded, entry points are set and the core is released from reset. For more on bootflow/bootloaders, please refer \ref BOOTFLOW_GUIDE

\imageStyle{sbl_can_uniflash_process.PNG,width:60%}
\image html sbl_can_uniflash_process.PNG MCAN SBL CAN UNIFLASH Process

This bootloader runs in two steps:
- Flashing the SBL CAN at offset 0x0 (Setup the EVM in UART Boot Mode)
- Running the python script (See \ref CAN_BOOTLOADER_PYTHON_SCRIPT) for sending the application image file. (Setup the EVM in QSPI Boot Mode)

\imageStyle{am263x_sbl_can_flow.png,width:15%}
\image html am263x_sbl_can_flow.png MCAN SBL CAN Flow Overview

# Protocol
A simple custom made protocol is created for communication between the Host Machine and the Board. Messages between a CAN bootloader host and the target use a simple command and acknowledge
(ACK) protocol. The host sends a command and within a timeout period the target responds with an ACK.

The CAN Uniflash provides a short list of commands that are used during the flashing operation.

<table>
<tr>
    <th>Commands
    <th>Description
</tr>
<tr>
    <td>PING</td>
    <td> This command is used to receive an PONG command from the bootloader indicating that communication has been established.</td>
</tr>
<tr>
    <td>PONG</td>
    <td> Returned as response to the initial PING command from the Bootloader </td>
</tr>
<tr>
    <td>DATA</td>
    <td> To send data packets </td>
</tr>
<tr>
    <td>RUN</td>
    <td>To run the application</td>
</tr>
<tr>
    <td>ACK</td>
    <td>Response from the Bootloader on recieveing the CAN Packets</td>
</tr>
<tr>
    <td>LSTMSG</td>
    <td>To indicate the last data packet of the application</td>
</tr>
<tr>
    <td>LSTMSGCF</td>
    <td>Response from the Bootloader on recieveing the last data packet of the application</td>
</tr>
</table>
Please refer the image below:

\imageStyle{sbl_can_custom_protocol.png,width:50%}
\image html sbl_can_custom_protocol.png SBL CAN Communication Protocol between Host Machine and the Board

# Settings Used in this Application
In this application, the CAN settings are:
- Extended Identifier used
- Make Sure to set correct timing parameters. Current parameters is as per SDK-CAN driver.
    - BitrateFD : f_clock_mhz=80, nom_brp=1, nom_tseg1=67, nom_tseg2=12, nom_sjw=12, data_brp=1, data_tseg1=13, data_tseg2=2, data_sjw=1
- CAN-FD is supported
- Refer to \subpage DRIVERS_MCAN_PAGE, for MCAN dependencies.
# Supported Combinations {#EXAMPLES_DRIVERS_SBL_CAN_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_can

**NOTE: Tested on Windows, using PCAN-USB Peripheral.**
\endcond

# Steps to Run the Example

 Since this is a bootloader, the example will be run every time you boot an application using this example. It is run from a QSPI boot media unlike other examples which are usually loaded with CCS. Nevertheless, you can build this example like you do for the others using makefile or build it via CCS by importing as a project.

 - **Example is tested using PCAN-USB module**

 - **Hardware Conectivity**, First make sure the connections to the PCAN-USB module to PC are proper. Connect the PCAN-USB module to PC from USB and Serial Port to be connected as mentioned in the image below.

\imageStyle{am263x_mcan_sbl_hw_connect.PNG,width:60%}
\image html am263x_mcan_sbl_hw_connect.PNG MCAN Hardware Connectivity with PCAN USB.

- **Software Setup**, To use SBL CAN using PCAN-USB Peripheral,First Download and Install the PCAN-Driver from
https://www.peak-system.com/Drivers.523.0.html?&L=1
    - Peak Systems provides PCAN-Basic API for Connecting to CAN and CAN FD Buses. PCAN-Basic       consists of the actual device driver and an interface DLL, which provides the API functions.
    It can be downloaded from here, https://www.peak-system.com/PCAN-Basic.239.0.html?&L=1
    - After downloading the PCAN-Basic, please extract it. PCAN-Basic directory contains `PCANBasic.py` python file, under
    \code
    .\pcan-basic\PCAN-Basic API\Include\PCANBasic.py
    \endcode
    - Create a folder named `pcan` at following location in SDK directory:
    \code
    ${SDK_INSTALL_PATH}/tools/boot/pcan
    \endcode
    - We've to place this `PCANBasic.py` python file under this folder in SDK:
    \code
    ${SDK_INSTALL_PATH}/tools/boot/pcan/PCANBasic.py
    \endcode
    - `PCANBasic.dll` library needs to be installed in order
    to run applications that use this API. PCAN Device Driver application also installs `PCANBasic.dll` library. Please refer https://www.peak-system.com/produktcd/Develop/PC%20interfaces/Windows/PCAN-Basic%20API/ReadMe.txt for more information on this.
    - Interface DLL file needs to be present in the following windows directory:
    \code
        For x64 Windows systems:
      \x86\PCANBasic.dll --> Windows\SysWOW64
      \x64\PCANBasic.dll --> Windows\System32
    \endcode
    - One can use the Microsoft sigcheck tool (can be dowloaded from https://learn.microsoft.com/en-us/sysinternals/downloads/sigcheck) to find out if your PCANBasic.dll file is 32 or 64 bit version.
    \code
        sigcheck.exe C:\Windows\SysWOW64\PCANBasic.dll
    \endcode

- **When using CCS projects to build**, import the CCS project for the required combination and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).

- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

- After hardware and software setup follow below steps to run the example:
    - First the SBL CAN bootloader will be flashed.
    - Create a flash configuration file, using the default flash configuration file present, at below as reference

        \code ${SDK_INSTALL_PATH}/tools/boot/sbl_prebuilt/{board}/default_sbl_can.cfg \endcode

    - In this config file, modify the paths to the flashing application and CAN bootloader, in case you are not using the pre-built applications

        \code
        --flash-writer={path to flash application .tiimage}
        --file={path to CAN bootloader .tiimage} --operation=flash --flash-offset=0x0
        \endcode

    - Run below python command on the Windows command prompt (`cmd.exe`) or Linux bash shell to flash the files.

        \code
        cd ${SDK_INSTALL_PATH}/tools/boot
        python uart_uniflash.py -p {name of your UART com port} --cfg={path to your edited config file}
        \endcode

    - Refer to the page \ref BASIC_STEPS_TO_FLASH_FILES for more information

    - After flashing the bootloader, the application image file is sent using a python script (see \ref CAN_BOOTLOADER_PYTHON_SCRIPT).


# See Also

\ref DRIVERS_BOOTLOADER_PAGE

# Sample Output

After sending the application image over CAN, logs can be seen on the UART terminal