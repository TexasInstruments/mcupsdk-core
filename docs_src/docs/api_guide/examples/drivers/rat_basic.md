# RAT Example {#EXAMPLES_RAT}

[TOC]

# Introduction

RAT example provides how to use configure RAT.

# Supported Combinations {#EXAMPLES_RAT_COMBOS}


\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/rat

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# Sample Output


\code

Starting OSPI Bootloader ...
[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
[BOOTLOADER_PROFILE] Boot Media Clock : 133.333 MHz
[BOOTLOADER_PROFILE] Boot Image Size  : 0 KB
[BOOTLOADER_PROFILE] Cores present    :
r5f0-0
[BOOTLOADER PROFILE] System_init                      :        249us
[BOOTLOADER PROFILE] Drivers_open                     :         78us
[BOOTLOADER PROFILE] LoadHsmRtFw                      :        862us
[BOOTLOADER PROFILE] Board_driversOpen                :      30997us
[BOOTLOADER PROFILE] CPU load                         :         78us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :      32267us

Image loading done, switching to application ...

All tests have passed!!

\endcode

# Description

On how to configure RAT, please go through \ref OPTIFLASH_CONFIGURE.

This application configures RAT to translate a region with starting address 0x70200000 and size of 32KB that would translate to 0x70000000 and both of these addresses are in OCRAM. So when writing to address 0x70200000 would actually write to 0x70000000. At 0x70000000 address `buffer` array has been placed using memory configurator. Therefore, All that is written at address 0x70200000, can be read from buffer variable. This function is being displayed through this example.
