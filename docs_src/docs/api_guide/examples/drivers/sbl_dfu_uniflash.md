# SBL DFU Uniflash {#EXAMPLES_DRIVERS_SBL_DFU_UNIFLASH}

[TOC]

# Introduction


This is a flash-writer application which works in conjunction with the **usb_dfu_uniflash.py** python script mentioned in \ref TOOLS_FLASH. Although it is not strictly not a bootloader, it uses bootloader APIs to do basic SOC initialization required to be able to flash binaries to the OSPI flash. Like other SBLs, this is also booted by the ROM bootloader. It is analogous to \ref EXAMPLES_DRIVERS_SBL_UART_UNIFLASH

- DFU which stands for (Device Firmware Upgrade) is a standard device class of USB \ref USB_DEVICE_DRIVER. This application uses DFU to flash the SBLs and application images to ospi/qspi flash.

	- It uses DFU class from TinyUSB stack please refer to \ref USB_DEVICE_DRIVER for more information. 

Once the example starts running it attempts to receive files via USB DFU and process them in a loop. Once it receives a file (this is sent by the usb_dfu_uniflash.py script), it finds out what to do with the received file from the file header. It can be three things:

- Flash the received file at the given offset
- Verify if the data in the received file is present at the given offset
- Erase the flash at the given offset for the given size

The meta-data required for doing these operations (offset, file size, erase size etc.) will be extracted from the same header.

This example is more or less like a **flashing server, and will never terminate until EVM is powered down or the core is reset.**

\note The DFU uniflash differes from \ref EXAMPLES_DRIVERS_SBL_UART_UNIFLASH in response message stage. 
 - SBL UART uniflash send the response header back via UART which contains the status information of the requested flash 
  cmd mentioned in the file header. 
 - As USB DFU is host driven protocol the response stage differs from the UART uniflash. **dfu-util** will take care of necessary error handling. 

- Refer to \htmllink{https://www.usb.org/sites/default/files/DFU_1.1.pdf, DFU_1.1.pdf} to know more about USB DFU class. 

# Supported Combinations {#EXAMPLES_DRIVERS_SBL_DFU_UNIFLASH_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_dfu_uniflash

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_dfu_uniflash

\endcond

# Steps to Run the Example

Since this is mainly a flash-writer application, **this is sent via the USB DFU** unlike other examples which are usually loaded with CCS. Nevertheless, you can build this example like you do for the others using makefile or build it via CCS by importing as a project.

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

# See Also

\ref DRIVERS_BOOTLOADER_PAGE

\ref TOOLS_FLASH_DFU_UNIFLASH. 

\ref TOOLS_FLASH 

\ref GETTING_STARTED_FLASH_DFU

# Sample Output

Since this SBL receives the appimage and other files over USB protocol, it doesn't print anything to the console.
