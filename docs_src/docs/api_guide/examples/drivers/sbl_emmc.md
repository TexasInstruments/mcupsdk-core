# SBL EMMC {#EXAMPLES_DRIVERS_SBL_EMMC}

[TOC]

# Introduction

This bootloader does SOC initializations and attempts to boot a multicore appimage present at 0x800000 in eMMC boot partition 1. To flash a multicore appimage at this location, follow the steps mentioned in \ref BASIC_STEPS_TO_FLASH_FILES.

If a multicore appimage is found at the location, the SBL parses it, splits it into RPRCs for each core applicable. Each core is then initialized, RPRC image is loaded, entry points are set and the core is released from reset. For more on bootflow/bootloaders, please refer \ref BOOTFLOW_GUIDE

# Supported Combinations {#EXAMPLES_DRIVERS_SBL_EMMC_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_emmc

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_emmc

\endcond

# Steps to Run the Example

Since this is a bootloader, the example will be run every time you boot an application using this example (In EMMC boot mode). Nevertheless, you can build this example like you do for the others using makefile or build it via CCS by importing as a project.

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Refer to the page \ref BASIC_STEPS_TO_FLASH_FILES to flash this bootloader along with the application to boot.

# See Also

\ref DRIVERS_BOOTLOADER_PAGE

# Sample Output

\cond SOC_AM64X || SOC_AM243X
\code
[BOOTLOADER_PROFILE] Boot Media       : undefined
[BOOTLOADER_PROFILE] Boot Image Size  : 175 KB
[BOOTLOADER_PROFILE] Cores present    :
m4f0-0
r5f1-0
r5f1-1
a530-0
r5f0-0
r5f0-1
[BOOTLOADER PROFILE] SYSFW init                       :      12483us
[BOOTLOADER PROFILE] System_init                      :    5368613us
[BOOTLOADER PROFILE] Drivers_open                     :      39467us
[BOOTLOADER PROFILE] Board_driversOpen                :          0us
[BOOTLOADER PROFILE] Sciclient Get Version            :       9892us
[BOOTLOADER PROFILE] CPU Load                         :    2311966us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :    2373714us

Image loading done, switching to application ...
\endcode
\endcond

