# SBL SD {#EXAMPLES_DRIVERS_SBL_SD}

[TOC]

# Introduction

This bootloader does SOC initializations and attempts to boot a multicore appimage file named "app" present in the first FAT partition found in the connected SD card. The file can be copied to the SD card by connecting it to the host PC using a card reader. Make sure that it is named "app" without any file extension. If the card is new, make sure that it is formatted with FAT32/16.

If a multicore appimage file is found at the location, the SBL reads the file into a buffer, parses it, splits it into RPRCs for each core applicable. Each core is then initialized, RPRC image is loaded, entry points are set and the core is released from reset. For more on bootflow/bootloaders, please refer \ref BOOTFLOW_GUIDE

# Supported Combinations {#EXAMPLES_DRIVERS_SBL_SD_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_sd

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_sd

\endcond

# Steps to Run the Example

Since this is a bootloader, the example will be run every time you boot an application using this example. It is run from an SD card unlike other examples which are usually loaded with CCS. Nevertheless, you can build this example like you do for the others using makefile or build it via CCS by importing as a project.

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Refer to the page \ref BASIC_STEPS_TO_FLASH_FILES to flash this bootloader along with the application to boot.

# See Also

\ref DRIVERS_BOOTLOADER_PAGE

# Sample Output

\code
[BOOTLOADER PROFILE] SYSFW Load                       :      17632us
[BOOTLOADER PROFILE] System_init                      :      12246us
[BOOTLOADER PROFILE] Drivers_open                     :      82499us
[BOOTLOADER PROFILE] Board_driversOpen                :          0us
[BOOTLOADER PROFILE] File read from SD card           :       6843us
[BOOTLOADER PROFILE] CPU load                         :       9501us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :     129399us

Image loading done, switching to application ...
\endcode