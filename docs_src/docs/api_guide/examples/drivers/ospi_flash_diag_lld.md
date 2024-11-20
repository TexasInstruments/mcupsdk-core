# OSPI Flash Diagnostic LLD {#EXAMPLES_DRIVERS_OSPI_FLASH_DIAG_LLD}

[TOC]

# Introduction

This is a diagnostic example for NOR-OSPI Flashes. This example doesn't use the flash driver and should ideally pass for any NOR-OSPI flash. For this particular example, the options in SysCfg
like the number of transfer lines used, or the clock divider value are ignored. This example will always talk to the flash in the lowest settings possible. The flash device is reset, and is
expected to support 1s1s1s mode after reset. Then the OSPI controller is programmed to work in 1s1s1s mode with 3 byte addressing mode.

The test itself is simple, first it tries to read the JEDEC ID of the flash which consists of the flash manufacturer ID and the flash device ID. These are then printed onto the logging console. When
doing flash bring-ups in new boards, this example can be run first for sanity. Users can cross check the printed ID with the one in flash datasheet to confirm that basic communication with flash is working.

The test then tries to erase a flash memory block at an offset of 512 KB and then write some known data to that memory. This data is then read back and verified to confirm that reads and writes are working
in 1s1s1s mode.

# Supported Combinations {#EXAMPLES_DRIVERS_OSPI_FLASH_DIAG_LLD_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/ospi/ospi_flash_diag_lld

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ospi/ospi_flash_diag_lld

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ospi/ospi_flash_diag_lld

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ospi/ospi_flash_diag_lld

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_OSPI_PAGE

# Sample Output

\code
[Cortex_R5_0] [OSPI Flash Diagnostic Test] Starting ...
[OSPI Flash Diagnostic Test] Flash Manufacturer ID : 0x9D
[OSPI Flash Diagnostic Test] Flash Device ID       : 0x5A19
[OSPI Flash Diagnostic Test] Executing Flash Erase on first block...
[OSPI Flash Diagnostic Test] Done !!!
[OSPI Flash Diagnostic Test] Performing Write-Read Test...
[OSPI Flash Diagnostic Test] Write-Read Test Passed!
All tests have passed!!
\endcode