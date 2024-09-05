# SBL OSPI {#EXAMPLES_DRIVERS_SBL_OSPI}

[TOC]

# Introduction

\if SOC_AM65X
This bootloader does SOC initializations and attempts to boot a multicore appimage present at 0x100000 location in the OSPI Flash. To flash a multicore appimage at this location, follow the steps mentioned in \ref BASIC_STEPS_TO_FLASH_FILES.
\else
This bootloader does SOC initializations and attempts to boot a multicore appimage present at 0x81000 location in the OSPI Flash. To flash a multicore appimage at this location, follow the steps mentioned in \ref BASIC_STEPS_TO_FLASH_FILES.
\endif

If a multicore appimage is found at the location, the SBL parses it. Each core is then initialized, application image is loaded, entry points are set and the core is released from reset. For more on bootflow/bootloaders, please refer \ref BOOTFLOW_GUIDE

\cond SOC_AM263PX || SOC_AM261X

\note RPRC image booting using SBL would be deprecated from SDK 11.00 release onwards. MCELF would be the default boot image format supported by SBL going forward.


# SBL OSPI MULTICORE ELF {#EXAMPLES_DRIVERS_SBL_OSPI_MCELF}

To flash an **mcelf** file, use the project **examples/drivers/boot/sbl_ospi_multicore_elf**

When an mcelf image is found, the SBL parses it, loads each segment to its specified address location. Then the core is released from reset.

The steps to run the example is same irrespective of the image format.

# SBL OSPI FASTBOOT {#EXAMPLES_DRIVERS_SBL_OSPI_FASTBOOT_MCELF}

This SBL is optimized for performance. It utilizes caching and disables logs which are present in other SBLs.
Only MCELF application image format is supported with this SBL.

To flash an mcelf file, use the project examples/drivers/boot/sbl_ospi_fastboot.

In addition to the SBL and application, the HSM runtime firmware should also be flashed at an address known to the this SBL.
This flash address can be configured via sysconfig and must match the one present in flash writer tool.

When an mcelf image is found, the SBL parses it, loads each segment to its specified address location. Then the core is released from reset.

The steps to run the example is same irrespective of the image format. Please follow the steps mentioned in \ref FAST_SECURE_BOOT.

# SBL OSPI SWAP {#EXAMPLES_DRIVERS_SBL_OSPI_SWAP}

SBL_OSPI_SWAP provides boot region swaping mechanism based on information provided in the boot sector (4KB) 0x80000.

If the byte at address 0x80080 is 0, 0x80081 is 0, 0x80082 is 0, 0x80083 is 1, then this SBL will boot from 2nd half of flash and if there is any other value in any of the above address then this SBL will boot from 1st half of flash.  

For more on exact mechanism of this switching please refer \ref bootseg_ip_working. If it fails to boot from selected boot region, then it will try to boot from other region and if it fails again then SBL fails to boot application.

This also has anti-rollback support. User Application needs to make sure that minimum application number in the HSM should corresponds to the intended application version.

\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_SBL_OSPI_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_ospi

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_ospi

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_ospi

\endcond

\cond SOC_AM65X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_ospi

\endcond

# Steps to Run the Example

Since this is a bootloader, the example will be run every time you boot an application using this example. It is run from a OSPI boot media  unlike other examples which are usually loaded with CCS. Nevertheless, you can build this example like you do for the others using makefile or build it via CCS by importing as a project.

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
Starting OSPI Bootloader ...
[BOOTLOADER PROFILE] SYSFW Load                       :      17606us
[BOOTLOADER PROFILE] System_init                      :      19466us
[BOOTLOADER PROFILE] Drivers_open                     :        140us
[BOOTLOADER PROFILE] Board_driversOpen                :      21921us
[BOOTLOADER PROFILE] CPU load                         :      18978us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :      78791us
\endcode
\endcond

\cond SOC_AM263PX

\code
    Starting OSPI Bootloader ...
    [BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH
    [BOOTLOADER_PROFILE] Boot Media Clock : 100.000 MHz
    [BOOTLOADER_PROFILE] Boot Image Size  : 0 KB
    [BOOTLOADER_PROFILE] Cores present    :
    r5f0-0
    [BOOTLOADER PROFILE] System_init                      :        199us
    [BOOTLOADER PROFILE] Drivers_open                     :         76us
    [BOOTLOADER PROFILE] LoadHsmRtFw                      :        861us
    [BOOTLOADER PROFILE] Board_driversOpen                :       3039us
    [BOOTLOADER PROFILE] CPU load                         :        102us
    [BOOTLOADER_PROFILE] SBL Total Time Taken             :       4280us
\endcode

\endcond