# Reset Isolation of MCU domain with IPC {#EXAMPLES_DRIVERS_RESET_ISOLATION_IPC}

[TOC]

# Introduction

The example demonstrates MCU reset isolation in the use case when MCU domain is running
a safety application and needs to be isolated from the MAIN domain.

The example can be run in OSPI boot mode. On running the application the MCU M4 and
the MAIN domain R5 logs a heartbeat message to MAIN UART and MCU UART respectively.
The cores also logs the IPC message between the cores.

The MCU and MAIN domain cores will also be communicating with each other using IPC
Rpmsg  using Vrings in SRAM (MAIN domain). The MAIN to MCU access is still restricted.

On pressing the SOC warm reset button (SW4), the reset isolated MCU receives an interrupts,
isolates the MCU to MAIN domain access and propogates the reset to MAIN domain.

On pressing the GPIO SW5 on the SOC, a GPIO interrupt will be received on MAIN
domain R5 core and it will send an IPC RPmessage to MCU M4. On receiving the
message the MCU M4 initiates a warm reset and resets the MAIN domain.

After reset of the MAIN domain the IPC between the cores is restarted again.

# Supported Combinations {#EXAMPLES_DRIVERS_RESET_ISOLATION_IPC_COMBOS}

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/safety/reset_isolation_ipc

\endcond

# Steps

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE


# Sample Output

Shown below is a sample output when the application is run,

\code

################################ R5 Log #######################################

DMSC Firmware Version 8.5.3--v08.05.03 (Chill Capybar
DMSC Firmware revision 0x8
DMSC ABI revision 3.1

[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
[BOOTLOADER_PROFILE] Boot Media Clock : 166.667 MHz
[BOOTLOADER_PROFILE] Boot Image Size  : 120 KB
[BOOTLOADER_PROFILE] Cores present    :
m4f0-0
r5f0-0
[BOOTLOADER PROFILE] SYSFW init                       :      12214us
[BOOTLOADER PROFILE] System_init                      :      39859us
[BOOTLOADER PROFILE] Drivers_open                     :        268us
[BOOTLOADER PROFILE] Board_driversOpen                :      21929us
[BOOTLOADER PROFILE] Sciclient Get Version            :      10028us
[BOOTLOADER PROFILE] CPU Load                         :     226172us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :     310474us

Image loading done, switching to application ...
Starting R5
Press and release SW5 button on EVM to trigger warm reset from SW...
Press and release SW4 button on EVM to trigger a warm reset from HW..

R5F: R5 <--> M4 is communicating --> 1
I am running (R5) !!:- 0
R5F: R5 <--> M4 is communicating --> 2
I am running (R5) !!:- 1
R5F: R5 <--> M4 is communicating --> 3
I am running (R5) !!:- 2

DMSC Firmware Version 8.5.3--v08.05.03 (Chill Capybar
DMSC Firmware revision 0x8
DMSC ABI revision 3.1

[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
[BOOTLOADER_PROFILE] Boot Media Clock : 166.667 MHz
[BOOTLOADER_PROFILE] Boot Image Size  : 66 KB
[BOOTLOADER_PROFILE] Cores present    :
r5f0-0
[BOOTLOADER PROFILE] SYSFW init                       :      33953us
[BOOTLOADER PROFILE] System_init                      :      36433us
[BOOTLOADER PROFILE] Drivers_open                     :        269us
[BOOTLOADER PROFILE] Board_driversOpen                :      21928us
[BOOTLOADER PROFILE] Sciclient Get Version            :      10020us
[BOOTLOADER PROFILE] CPU Load                         :     186656us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :     289264us

Image loading done, switching to application ...
Starting R5
Press and release SW5 button on EVM to trigger warm reset from SW...
Press and release SW4 button on EVM to trigger a warm reset from HW..

R5F: R5 <--> M4 is communicating --> 1
I am running (R5) !!:- 0
R5F: R5 <--> M4 is communicating --> 2
I am running (R5) !!:- 1
R5F: R5 <--> M4 is communicating --> 3
I am running (R5) !!:- 2
R5F: R5 <--> M4 is communicating --> 4
I am running (R5) !!:- 3

##############################################################################

################################ M4 Log #######################################

M4F: R5 <--> M4 is communicating --> 1
I am running (M4) !!:- 0
M4F: R5 <--> M4 is communicating --> 2
I am running (M4) !!:- 1
M4F: R5 <--> M4 is communicating --> 3
I am running (M4) !!:- 2
M4F: R5 <--> M4 is communicating --> 4
I am running (M4) !!:- 3
M4F: R5 <--> M4 is communicating --> 1
I am running (M4) !!:- 4
M4F: R5 <--> M4 is communicating --> 2
I am running (M4) !!:- 5
M4F: R5 <--> M4 is communicating --> 3
I am running (M4) !!:- 6
M4F: R5 <--> M4 is communicating --> 4
I am running (M4) !!:- 7
M4F: R5 <--> M4 is communicating --> 5
I am running (M4) !!:- 8

##############################################################################

\endcode
