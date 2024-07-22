# Uniflash Custom Flash Writer {#TOOLS_UNIFLASH_CUSTOM_FLASHER}

[TOC]

# Introduction

This document explains the functionality of a flash-writer application that works in conjunction with the TI Uniflash Tool/CCS, as mentioned in \ref TI_UNIFLASH_TOOL. The TI Uniflash Tool/CCS is packaged with pre-built flash writer intended for use with TI boards. However, this example is primarily aimed at supporting boards with external flash memory that differs from the flash memory that comes with the TI board. This document provides a step-by-step guide on how to build and include the flash writer in the TI Uniflash tool to enable flashing on custom flashes.

# Supported Combinations {#TOOLS_CUSTOM_FLASHER_COMBOS}

\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | tools/flasher/jtag_uniflash

\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | tools/flasher/jtag_uniflash

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | tools/flasher/jtag_uniflash

\endcond

# Steps to Configure the Application
    1. Get Custom Flash Configuration
        - Please refer to the guide \ref CUSTOM_FLASH_SUPPORT_GUIDE and generate the custom flash config JSON
        - Open the sysconfig of the application and fill up the flash configuration.
    2. Selecting buffer Size and Flash Write Type
        - To ensure that the application doesn't overlap with the user applications, it should be loaded onto SBL reserved space. To achieve this, it's important to select the correct buffer size and flash write method for flashing.
        \imageStyle{custom_flash_memory.png,width:70%}
        \image html custom_flash_memory.png "Flashwriter Memory Layout"
        - The buffer size required depends on the flash write method used. Flashes typically have large block sizes and small sector sizes. Depending on the size, block write or sector write can be used. It's recommended to configure the buffer size with a multiple of block or sector size, instead of allocating the buffer with block or sector size. This optimization will help to make the flashing operation more efficient.
        - Define this flag to use sector flashing. It is recommended to confirm sector flashing is functional before defining this flag, as some flashes have a hybrid sector and sector flashing will not work directly.
        \code
        #undef FLASH_WRITE_SECTOR
        \endcode
        - For block or sector type, the flash buffer size should be multiple of block/sector size and aligned with block or sector size
        \code
        /* Please make sure to keep the buffersize >= block/sector size and aligned with block/sector size */
        #define BOOTLOADER_UNIFLASH_MAX_FILE_SIZE (0x10000)
        \endcode
        - To calculate the total memory required for the buffer, add the internal data buffer size (which should be a multiple of sector/block size) and the uniflash header size. The uniflash header contains important information about the data size, flashing operation, flash offset, and other details. It's important to allocate the necessary memory in the linker file to ensure everything works properly.
        \code
        /* This has to match the size of RAM1 section in linker.cmd */
        uint8_t gFileBuf[BOOTLOADER_UNIFLASH_MAX_FILE_SIZE+BOOTLOADER_UNIFLASH_HEADER_SIZE] __attribute__((aligned(128), section(".bss.filebuf")));
        \endcode
        - Ensure that the application memory does not exceed SBL reserved space by checking the map file of the application.


# Steps to Run the Example

As this is primarily a flash-writer application, it is used with TI Uniflash Tool/CCS instead of being loaded with CCS like other examples. However, you can build this example using a makefile or import it as a project in CCS and build it that way.

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

# See Also

For Serial Uniflash Support, please follow the guide below to build a UART Uniflash application. Once built, enter the path of the output .tiimage in the Uniflash tool. This step is not necessary in CCS.
\ref EXAMPLES_DRIVERS_SBL_UART_UNIFLASH

# Troubleshooting

As this is a flashwriter application, it does not print anything to the console. You can check the application's functionality by running it in CCS and manually filling in the header and flash data. This will allow you to confirm whether your program was able to flash and reach its exit() properly. By doing so, you can ensure that the flash writer application is working correctly.
