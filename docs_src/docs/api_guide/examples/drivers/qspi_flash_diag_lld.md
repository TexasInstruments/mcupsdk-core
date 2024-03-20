# QSPI Flash Diag LLD {#EXAMPLES_DRIVERS_QSPI_FLASH_DIAG_LLD}

[TOC]

# Introduction

This is a diagnostic example for NOR-QSPI Flashes using LLD Driver.
This example doesn't use the flash driver and should ideally pass for any NOR-QSPI flash. For this particular example, the options in SysCfg like the number of transfer lines used, or the clock divider value are ignored. This example will always talk to the flash in the lowest settings possible.
The flash device is reset, and is expected to support 1s1s1s mode after reset. Then the QSPI controller is programmed to work in 1s1s1s mode with 3 byte addressing mode.

The test itself is simple, first it tries to read the JEDEC ID of the flash which consists of the flash manufacturer ID and the flash device ID. These are then printed onto the logging console. When
doing flash bring-ups in new boards, this example can be run first for sanity. Users can cross check the printed ID with the one in flash datasheet to confirm that basic communication with flash is working.

The test then tries to erase a flash memory block at an offset of 64 KB and then write some known data to that memory. This data is then read back and verified to confirm that reads and writes are working in 1s1s1s mode.

# Supported Combinations {#EXAMPLES_DRIVERS_QSPI_FLASH_DIAG_LLD_COMBOS}

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/qspi/qspi_flash_diag_lld

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/qspi/qspi_flash_diag_lld

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[QSPI Flash Diagnostic Test] Starting ...
[QSPI Flash Diagnostic Test] Flash Manufacturer ID : 0x1
[QSPI Flash Diagnostic Test] Flash Device ID       : 0x2018
[QSPI Flash Diagnostic Test] Executing Flash Erase on first block...
[QSPI Flash Diagnostic Test] Done !!!
[QSPI Flash Diagnostic Test] Performing Write-Read Test...
[QSPI Flash Diagnostic Test] Write-Read Test Passed!
All tests have passed!!
\endcode

