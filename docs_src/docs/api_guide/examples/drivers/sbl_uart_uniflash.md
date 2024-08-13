# SBL UART Flash Writer {#EXAMPLES_DRIVERS_SBL_UART_UNIFLASH}

[TOC]

# Introduction

This is a flash-writer application which works in conjunction with the uart_uniflash.py python script mentioned in \ref TOOLS_FLASH. Although it is not strictly not a bootloader, it uses bootloader APIs to do basic SOC initialization required to be able to flash binaries to the OSPI flash. Like other SBLs, this is also booted by the ROM bootloader.

Once the example starts running it attempts to receive files via UART+XMODEM and process them in a loop. Once it receives a file (this is sent by the uart_uniflash.py script), it finds out what to do with the received file from the file header. It can be three things:

- Flash the received file at the given offset
- Verify if the data in the received file is present at the given offset
- Erase the flash at the given offset for the given size

The meta-data required for doing these operations (offset, file size, erase size etc.) will be extracted from the same header.

After the file is processed, an acknowledgment is sent back to the host side python script and the loop continues.

This example is more or less like a flashing server, and will never terminate until EVM is powered down or the core is reset.

# Supported Combinations {#EXAMPLES_DRIVERS_SBL_UART_UNIFLASH_COMBOS}


\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_uart_uniflash

\endcond

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_uart_uniflash

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_uart_uniflash

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_uart_uniflash

\endcond

\cond SOC_AM65X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_uart_uniflash

\endcond

# Steps to Run the Example

Since this is mainly a flash-writer application, this is sent via the UART unlike other examples which are usually loaded with CCS. Nevertheless, you can build this example like you do for the others using makefile or build it via CCS by importing as a project.

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

# See Also

\ref DRIVERS_BOOTLOADER_PAGE

# Sample Output

Since this SBL receives the appimage and other files over UART using XMODEM protocol, it doesn't print anything to the console so as not to corrupt the XMODEM transport.
