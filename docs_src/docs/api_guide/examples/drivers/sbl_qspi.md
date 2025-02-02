# SBL QSPI {#EXAMPLES_DRIVERS_SBL_QSPI}

[TOC]

# Introduction

This bootloader does SOC initializations and attempts to boot a multicore appimage present at 0xA0000 location in the QSPI Flash. To flash a multicore appimage at this location, follow the steps mentioned in \ref BASIC_STEPS_TO_FLASH_FILES.

If a multicore appimage is found at the location, the SBL parses it. Each core is then initialized, application image is loaded, entry points are set and the core is released from reset. For more on bootflow/bootloaders, please refer \ref BOOTFLOW_GUIDE

\cond SOC_AM263X

\note RPRC image booting using SBL would be deprecated from SDK 11.00 release onwards. MCELF would be the default boot image format supported by SBL going forward.

# SBL QSPI MULTICORE ELF {#EXAMPLES_DRIVERS_SBL_QSPI_MCELF}

To flash an **mcelf** file, use the project **examples/drivers/boot/sbl_qspi_multicore_elf**

When an mcelf image is found, the SBL parses it, loads each segment to its specified address location. Then the core is released from reset.

The steps to run the example is same irrespective of the image format.

# SBL QSPI FASTBOOT {#EXAMPLES_DRIVERS_SBL_QSPI_FASTBOOT_MCELF}

This SBL is optimized for performance. It utilizes caching and disables logs which are present in other SBLs.
Only MCELF application image format is supported with this SBL.

To flash an mcelf file, use the project examples/drivers/boot/sbl_qspi_fastboot.

In addition to the SBL and application, the HSM runtime firmware should also be flashed at an address known to the this SBL.
This flash address can be configured via sysconfig and must match the one present in flash writer tool.

When an mcelf image is found, the SBL parses it, loads each segment to its specified address location. Then the core is released from reset.

The steps to run the example is same irrespective of the image format. Please follow the steps mentioned in \ref FAST_SECURE_BOOT.

\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_SBL_QSPI_COMBOS}

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_qspi

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_qspi

\endcond

# Steps to Run the Example

Since this is a bootloader, the example will be run every time you boot an application using this example. It is run from a QSPI boot media  unlike other examples which are usually loaded with CCS. Nevertheless, you can build this example like you do for the others using makefile or build it via CCS by importing as a project.

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Refer to the page \ref BASIC_STEPS_TO_FLASH_FILES to flash this bootloader along with the application to boot.

# See Also

\ref DRIVERS_BOOTLOADER_PAGE

# Sample Output

\cond SOC_AM273X || SOC_AWR294X
\code
Starting QSPI Bootloader ...
[BOOTLOADER PROFILE] System_init                      :        666us
[BOOTLOADER PROFILE] Drivers_open                     :         25us
[BOOTLOADER PROFILE] Board_driversOpen                :       2740us
[BOOTLOADER PROFILE] CPU load                         :      60247us
[BOOTLOADER_PROFILE] SBL Total Time Taken             :      63680us

Image loading done, switching to application ...
\endcode
\endcond

\cond SOC_AM263X
\code
Starting QSPI Bootloader ...
[BOOTLOADER PROFILE] System_init : 196us
[BOOTLOADER PROFILE] Drivers_open : 28us
[BOOTLOADER PROFILE] Board_driversOpen : 235us
[BOOTLOADER PROFILE] CPU load : 126043us
[BOOTLOADER_PROFILE] SBL Total Time Taken : 126509us

Image loading done, switching to application ...
\endcode
\endcond
