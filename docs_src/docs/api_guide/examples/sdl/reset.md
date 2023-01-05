# SDL RESET Example {#EXAMPLES_SDL_RESET}

[TOC]

# Introduction

This example demonstrates-
1. How to check SoC latest reset cause.
2. How to check Warm reset cause for R5F Core.
3. How to assert Warm reset for SoC.
4. How to assert Local reset for individually DSP Core.



\cond SOC_AM273X || AWR294X
Use Cases
---------
* Assert Warm reset and check latest reset cause.
* Assert Local reset for DSP core individually.

\endcond


# Supported Combinations {#EXAMPLES_SDL_RESET_COMBOS}


\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/reset/reset_mcu/

\endcond

\cond SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/reset/reset_mcu/

\endcond


# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

The above two methods can be used only for building example, but for loading and running, need to follow the below method only.

- Flash the .appimage with qspi bootloader, for refrence see \ref GETTING_STARTED_FLASH

# See Also

\ref SDL_RESET_PAGE

# Sample Output

\cond  SOC_AWR294X
Shown below is a sample output when the application is run for R5F,

\code
Starting QSPI Bootloader ...
[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
[BOOTLOADER_PROFILE] Boot Media Clock : 80.000 MHz
[BOOTLOADER_PROFILE] Boot Image Size  : 31 KB
[BOOTLOADER_PROFILE] Cores present    :
r5f0-0
[BOOTLOADER PROFILE] System_init                      :        443us
[BOOTLOADER PROFILE] Drivers_open                     :         16us
[BOOTLOADER PROFILE] Board_driversOpen                :       2711us
[BOOTLOADER PROFILE] CPU load                         :      28385us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :      31559us

Image loading done, switching to application ...
INFO: Bootloader_runSelfCpu:217: All done, reseting self ...


Power on Reset is happened.
Foe R5F core, Reset cause is Warm Reset asserted by Power on Reset.
SW is asserting Warm reset.....
Starting QSPI Bootloader ...
[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
[BOOTLOADER_PROFILE] Boot Media Clock : 80.000 MHz
[BOOTLOADER_PROFILE] Boot Image Size  : 31 KB
[BOOTLOADER_PROFILE] Cores present    :
r5f0-0
[BOOTLOADER PROFILE] System_init                      :        444us
[BOOTLOADER PROFILE] Drivers_open                     :         16us
[BOOTLOADER PROFILE] Board_driversOpen                :       2711us
[BOOTLOADER PROFILE] CPU load                         :      28387us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :      31561us

Image loading done, switching to application ...
INFO: Bootloader_runSelfCpu:217: All done, reseting self ...


Due to SW, Warm Reset is happened.
Foe R5F core, Reset cause is Warm Reset asserted by Software.
DSP local reset is asserted.

\endcode
\endcond

\cond  SOC_AM273X

Shown below is a sample output when the application is run for R5F,

\code
Starting QSPI Bootloader ...
[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
[BOOTLOADER_PROFILE] Boot Media Clock : 80.000 MHz
[BOOTLOADER_PROFILE] Boot Image Size  : 30 KB
[BOOTLOADER_PROFILE] Cores present    :
r5f0-0
[BOOTLOADER PROFILE] System_init                      :        486us
[BOOTLOADER PROFILE] Drivers_open                     :         17us
[BOOTLOADER PROFILE] Board_driversOpen                :       2705us
[BOOTLOADER PROFILE] CPU load                         :       1825us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :       5034us

Image loading done, switching to application ...

Power on Reset is happened.
Foe R5F core, Reset cause is Warm Reset asserted by Power on Reset.
SW is asserting Warm reset.....
Starting QSPI Bootloader ...
[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
[BOOTLOADER_PROFILE] Boot Media Clock : 80.000 MHz
[BOOTLOADER_PROFILE] Boot Image Size  : 30 KB
[BOOTLOADER_PROFILE] Cores present    :
r5f0-0
[BOOTLOADER PROFILE] System_init                      :        486us
[BOOTLOADER PROFILE] Drivers_open                     :         17us
[BOOTLOADER PROFILE] Board_driversOpen                :       2705us
[BOOTLOADER PROFILE] CPU load                         :       1825us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :       5034us

Image loading done, switching to application ...

Due to SW, Warm Reset is happened.
Foe R5F core, Reset cause is Warm Reset asserted by Software.
DSP local reset is asserted.

\endcode
\endcond
