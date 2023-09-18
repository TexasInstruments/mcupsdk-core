#  RL2 Example {#EXAMPLES_RL2}

[TOC]

# Introduction

Rl2 example provides how to use RL2 in applications and also shows how RL2 brings in benefits.

# Supported Combinations {#EXAMPLES_RL2_COMBOS}


\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/rl2

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
L2 Cache:       i = 0   Miss = 129      Hits = 0
L2 Cache:       i = 1   Miss = 0        Hits = 129
L2 Cache:       i = 2   Miss = 0        Hits = 129
L2 Cache:       i = 3   Miss = 0        Hits = 129
L2 Cache:       i = 4   Miss = 0        Hits = 129
L2 Cache:       i = 5   Miss = 0        Hits = 129
L2 Cache:       i = 6   Miss = 0        Hits = 129
L2 Cache:       i = 7   Miss = 0        Hits = 129
L2 Cache:       i = 8   Miss = 0        Hits = 129
L2 Cache:       i = 9   Miss = 0        Hits = 129
Profile Point: With RL2
Cycle Count: 65854
No. Of CPU instructions executed Count: 21107
ICache Miss Count: 9
ICache Access Count: 65819

Profile Point: Without RL2
Cycle Count: 210590
No. Of CPU instructions executed Count: 20602
ICache Miss Count: 1
ICache Access Count: 210600

All tests have passed!!

\endcode

# Description

On how to configure RL2, please go through \ref OPTIFLASH_CONFIGURE.

This example shows how RL2 is used and how its different functions are used. With RL2 enabled, the function that was being executed is getting cached in the RL2 cache bank. First time, it caused 129 L2 cache misses, and because each L2 cache line (at a size of 128KB) is 32 bytes, it implies that a total of ~4KB of code is being cached. This is in alignment with the size of the function that is being executed from the flash. Later on, every time there were L2 cache hits,

Another point to highlight is performance improvement. This is clear from the cycle count with RL2 and without RL2.
