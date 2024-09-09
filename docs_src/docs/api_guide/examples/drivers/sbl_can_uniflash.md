# SBL CAN UNIFLASH {#EXAMPLES_DRIVERS_SBL_CAN_UNIFLASH}

[TOC]

# Introduction

This is a flash-writer application which can boot the application as well. This works in conjunction with the can_uniflash.py python script. It uses bootloader APIs to do basic SOC initialization required to be able to flash binaries to the QSPI flash. Like other SBLs, this is also booted by the ROM bootloader.

Once the example starts running it attempts to receive files via CAN interface and process them in a loop. Once it receives a file (this is sent by the can_uniflash.py script), it finds out what to do with the received file from the file header. This can be primarily used for Flashing the received file at the given offset.

\imageStyle{sbl_can_uniflash_process.PNG,width:60%}
\image html sbl_can_uniflash_process.PNG MCAN SBL CAN UNIFLASH Process

This bootloader runs in two steps:
- Flashing the SBL CAN UNIFLASH at offset 0x0 (Setup the EVM in UART Boot Mode)
- Running the can_uniflash.py python script for sending the application image file. (Setup the EVM in QSPI Boot Mode)

\imageStyle{sbl_can_uniflash_flow.png,width:15%}
\image html sbl_can_uniflash_flow.png MCAN SBL CAN UNIFLASH Flow Overview

Refer \ref EXAMPLES_DRIVERS_SBL_CAN_UNIFLASH_STEPS

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X

\note RPRC image booting using SBL would be deprecated from SDK 11.00 release onwards. MCELF would be the default boot image format supported by SBL going forward.


# SBL CAN UNIFLASH MULTICORE ELF {#EXAMPLES_DRIVERS_SBL_CAN_UNIFLASH_MCELF}

To flash an **mcelf** file via CAN uniflash, use the project **examples/drivers/boot/sbl_can_uniflash_multicore_elf**

When an mcelf image is received, the SBL parses it, loads each segment to its specified address location. Then the core is released from reset.

The steps to run the example is same irrespective of the image format. Refer \ref EXAMPLES_DRIVERS_SBL_CAN_UNIFLASH_STEPS

\endcond

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
- Refer to \ref DRIVERS_MCAN_PAGE, for MCAN dependencies.

# Supported Combinations {#EXAMPLES_DRIVERS_SBL_CAN_UNIFLASH_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_can_uniflash

\endcond
\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_can_uniflash

\endcond
**NOTE: Tested on Windows, using PCAN-USB Peripheral.**

# Steps to Run the Example {#EXAMPLES_DRIVERS_SBL_CAN_UNIFLASH_STEPS}

 Since this is a bootloader, the example will be run every time you boot an application using this example. It is run from a QSPI boot media unlike other examples which are usually loaded with CCS. Nevertheless, you can build this example like you do for the others using makefile or build it via CCS by importing as a project.

 - **Example is tested using PCAN-USB module**

\cond SOC_AM263X || SOC_AM263PX
 - **Hardware Conectivity**, First make sure the connections to the PCAN-USB module to PC are proper. Connect the PCAN-USB module to PC from USB and Serial Port to be connected as mentioned in the image below.

\imageStyle{am263x_mcan_sbl_hw_connect.PNG,width:60%}
\image html am263x_mcan_sbl_hw_connect.PNG MCAN Hardware Connectivity with PCAN USB.
\endcond

\cond SOC_AM273X
 - **Hardware Conectivity**, First make sure the connections to the PCAN-USB module to PC are proper:
    - Connect the PCAN-USB module to the board, by connecting Low to Low from PCAN-USB to MCAN instance A present on board.
    - Connect Ground to Ground from PCAN-USB to MCAN instance A present on board.
    - Connect High to High from PCAN-USB to MCAN instance A present on board.
    - Refer table below:
        - All pins are on the base board.
        - Pin number 1 starts from the triangular arrow mark.
         CAN_A PHY               |  PCAN-USB
         ------------------------|-----------------
         CANH (Pin 1  of J3)     | Pin 7 (CAN-H)
         CAN-GND (Pin 2 of J3)   | Pin 3 (CAN-GND)
         CANL (Pin 3  of J3)     | Pin 2 (CAN-L)
\endcond

 - **Software Setup**, To use SBL CAN using PCAN-USB Peripheral,First Download and Install the PCAN-Driver from https://www.peak-system.com/Drivers.523.0.html?&L=1
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

    \code
        ${SDK_INSTALL_PATH}/tools/boot/sbl_prebuilt/{board}/default_sbl_can_uniflash.cfg
    \endcode

    - In this config file, modify the paths to the flashing application and CAN bootloader, in case you are not using the pre-built applications

        \code
        --flash-writer={path to flash application .tiimage}
        --file={path to CAN Uniflash .tiimage} --operation=flash --flash-offset=0x0
        \endcode

    - Run below python command on the Windows command prompt (`cmd.exe`) or Linux bash shell to flash the files.

        \code
        cd ${SDK_INSTALL_PATH}/tools/boot
        python uart_uniflash.py -p {name of your UART com port} --cfg={path to your edited config file}
        \endcode

    - Refer to the page \ref BASIC_STEPS_TO_FLASH_FILES for more information

    - After flashing the bootloader, Setup the EVM in QSPI Boot Mode.

    - **CAN Uniflash Python Script**

    - For sending the app-image default_sbl_can_uniflash_app.cfg can be editted with the path of the app-image of the desired application.
    - cfg file contains only one command with arguments like --file, --operation and --flash-offset. \n
\if SOC_AM263X
        For AM263x, Default Flash Offset is at 0x80000.
        \code
        --file=C:/ti/mcu_plus_sdk_am263x_08_05_00_13/examples/drivers/ipc/ipc_rpmsg_echo/am263x-lp/system_freertos_nortos/ipc_rpmsg_echo_system.debug.appimage --operation=flash --flash-offset=0x80000
        \endcode
\endif
\if SOC_AM273X
        For AM273x, Default Flash Offset is at 0xA0000.
        \code
        --file=C:/ti/mcu_plus_sdk_am273x_08_05_00_13/examples/drivers/ipc/ipc_rpmsg_echo/am273x-evm/system_freertos_nortos/ipc_rpmsg_echo_system.debug.appimage --operation=flash --flash-offset=0xA0000
        \endcode
\endif
    - The application image file is sent using `can_uniflash.py` python script:

        \code
        cd ${SDK_INSTALL_PATH}/tools/boot
        python can_uniflash.py --cfg={path to your edited sbl_can_uniflash_app config file}
        \endcode




# See Also

\ref DRIVERS_BOOTLOADER_PAGE

# Sample Output

After sending the application image over CAN, logs can be seen on the UART terminal