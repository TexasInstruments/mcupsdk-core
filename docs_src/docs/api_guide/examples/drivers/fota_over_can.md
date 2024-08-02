# FOTA OVER CAN {#EXAMPLES_DRIVERS_FOTA_OVER_CAN}

[TOC]

# Introduction

This application demonstrates firmware updates over the air through can protocol. This works in conjunction with the fota_can_transfer.py python script. Once the application starts running, it attempts to receive a new image via can, in chunks of 4KB. It writes the incoming image to the other flash region. 

# Settings Used in this Application
In this application, the CAN settings are:
- Extended Identifier used
- Make Sure to set correct timing parameters. Current parameters is as per SDK-CAN driver.
    - BitrateFD : f_clock_mhz=80, nom_brp=1, nom_tseg1=67, nom_tseg2=12, nom_sjw=12, data_brp=1, data_tseg1=13, data_tseg2=2, data_sjw=1
- CAN-FD is supported
- Refer to \subpage DRIVERS_MCAN_PAGE, for MCAN dependencies.

# Supported Combinations

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/fss/fota_over_can

 \endcond

 **NOTE: Tested on Windows, using PCAN-USB Peripheral.**

# Steps to Run the Example


 - **Example is tested using PCAN-USB module**

\cond SOC_AM263PX
 - **Hardware Conectivity**, First make sure the connections to the PCAN-USB module to PC are proper. Connect the PCAN-USB module to PC from USB and Serial Port to be connected as mentioned in the image below.

\imageStyle{am263x_mcan_sbl_hw_connect.PNG,width:60%}
\image html am263x_mcan_sbl_hw_connect.PNG MCAN Hardware Connectivity with PCAN USB.
\endcond

- **Software Setup**, To use this application using PCAN-USB Peripheral,First Download and Install the PCAN-Driver from https://www.peak-system.com/Drivers.523.0.html?&L=1
    - Peak Systems provides PCAN-Basic API for Connecting to CAN and CAN FD Buses. PCAN-Basic       consists of the actual device driver and an interface DLL, which provides the API functions.
    It can be downloaded from here, https://www.peak-system.com/PCAN-Basic.239.0.html?&L=1
    - After downloading the PCAN-Basic, please extract it. PCAN-Basic directory contains `PCANBasic.py` python file, under
    \code
    .\pcan-basic\PCAN-Basic API\Include\PCANBasic.py
    \endcode
    - Create a folder named `pcan` at following location in SDK directory:
    \code
    ${SDK_INSTALL_PATH}/tools/fota/pcan
    \endcode
    - We've to place this `PCANBasic.py` python file under this folder in SDK:
    \code
    ${SDK_INSTALL_PATH}/tools/fota/pcan/PCANBasic.py
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

- After hardware and software setup follow below steps to run the example:

- **When using CCS projects to build**, import the CCS project for the required combination and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using make command (see \ref MAKEFILE_BUILD_PAGE)
- Refer to the page \ref BASIC_STEPS_TO_FLASH_FILES for more information
- After flashing the bootloader, Setup the EVM in OSPI Boot Mode.

- **FOTA CAN Transfer Python Script**

**NOTE: This Application only works with mcelf image.**
- For sending the app-image default_fota_can_app.cfg can be editted with the path of the MCELF image of the desired application.
- cfg file contains 2 commands with arguments like --file, --operation and --flash-offset. \n
- first line contains the Non Xip mcelf image path with flash offset 0x81000.
- second line contains the Xip mcelf image path.

\if SOC_AM263PX
        
        \code
        --file=../../examples/drivers/fss/fota_over_can/am263px-cc/r5fss0-0_freertos/ti-arm-clang/fota_over_can.debug.mcelf --operation=flash --flash-offset=0x81000

        --file=../../examples/drivers/fss/fota_over_can/am263px-cc/r5fss0-0_freertos/ti-arm-clang/fota_over_can.debug.mcelf_xip --operation=flash-xip
        \endcode
\endif

- The application image file is sent using `fota_can_transfer.py` python script:

    \code
    cd ${SDK_INSTALL_PATH}/tools/fota
    python fota_can_transfer.py --cfg={path to your edited fota_can_app config file}
    \endcode

# Sample Output

\code
GPIO LED Blink Test Started ...
LED will Blink for 20 seconds ...
BOOTING from Region B!!!! 
File Transfer Started!!!! 
GPIO LED Blink Test Passed!!
File 1 received!!!! 
File Transfer Started!!!! 
File 2 received!!!! 
Updating boot info....
Please reset the board
All Test Passed!! 
\endcode

After Reset the Boot region is updated 

\code
GPIO LED Blink Test Started ...
LED will Blink for 20 seconds ...
BOOTING from Region A!!!! 
GPIO LED Blink Test Passed!!

\endcode
