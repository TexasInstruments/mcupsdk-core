# SBL OSPI Multi-Partition {#EXAMPLES_DRIVERS_SBL_OSPI_MULTI_PARTITION}

[TOC]

# Introduction

This is a bootloader example, which shows an example of booting different CPUs flashed
at different offsets within the EVM flash.
This allows users to update a single CPU binary without updating the other CPU binaries.

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_ospi_multi_partition

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_ospi_multi_partition

\endcond

# Steps to Run the Example

## Build the example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## Run the example

- This example is the SBL which needs to be flashed on the EVM flash, along with sample application images for each CPU.
- There is a default flash config file as shown below which flashes this SBL and the IPC Notify echo applications

        ${SDK_INSTALL_PATH}/examples/drivers/boot/sbl_ospi_multi_partition/@VAR_BOARD_NAME_LOWER/{cpu}_{os}/default_sbl_ospi_multi_partition.cfg

- Make sure IPC notify echo application is built before running the flash script. (see \ref EXAMPLES_DRIVERS_IPC_NOTIFY_ECHO)

- To flash to the EVM, refer to \ref GETTING_STARTED_FLASH . Only when giving the flash config file, point to the `default_sbl_ospi_multi_partition.cfg` shown above.

- Example, assuming SDK is installed at `C:/ti/mcu_plus_sdk` and this example and IPC application is built using makefiles, in Windows,

        cd C:/ti/mcu_plus_sdk/tools/boot
        C:/ti/mcu_plus_sdk/tools/boot>python uart_uniflash.py -p COM13 --cfg=C:/ti/mcu_plus_sdk/examples/drivers/boot/sbl_ospi_multi_partition/@VAR_BOARD_NAME_LOWER/r5fss0-0_nortos/default_sbl_ospi_multi_partition.cfg

# See Also

\ref DRIVERS_BOOTLOADER_PAGE

# Sample Output

After flashing and booting the EVM, you will see below output on the UART console

\cond SOC_AM243X
    Starting OSPI Multi-Partition Bootloader ...

    DMSC Firmware Version 21.1.1--v2021.01a (Terrific Lla
    DMSC Firmware revision 0x15
    DMSC ABI revision 3.1

    INFO: Bootloader_runCpu:151: CPU r5f1-0  is initialized to 800000000 Hz !!!
    INFO: Bootloader_runCpu:151: CPU r5f1-1 is initialized to 800000000 Hz !!!
    INFO: Bootloader_runCpu:151: CPU m4f0-0 is initialized to 400000000 Hz !!!
    INFO: Bootloader_loadSelfCpu2:222: CPU r5f0-0 is initialized to 800000000 Hz !!!
    þ[IPC NOTIFY ECHO] Message exchange started by main core !!! to 800000000 Hz !!!
    [m4f0-0]     0.286023s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
    [r5f0-1]     0.036024s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
    [r5f0-1]     2.217979s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
    [r5f0-1]     2.217994s : All tests have passed!!
    [r5f1-0]     0.512029s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
    [r5f1-0]     2.693704s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
    [r5f1-0]     2.693719s : All tests have passed!!
    [r5f1-1]     0.402025s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
    [r5f1-1]     2.583249s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
    [r5f1-1]     2.583264s : All tests have passed!!
    [IPC NOTIFY ECHO] All echoed messages received by main core from 4 remote cores !!!
    [IPC NOTIFY ECHO] Messages sent to each core = 1000000
    [IPC NOTIFY ECHO] Number of remote cores = 4
    [m4f0-0]     4.280710s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
    [m4f0-0]     4.280730s : All tests have passed!!
\endcond

\cond SOC_AM64X

    Starting OSPI Multi-Partition Bootloader ...

    DMSC Firmware Version 21.5.0--v2021.05 (Terrific Llam
    DMSC Firmware revision 0x15
    DMSC ABI revision 3.1

    INFO: Bootloader_runCpu:147: CPU r5f1-0  is initialized to 800000000 Hz !!!
    INFO: Bootloader_runCpu:147: CPU r5f1-1 is initialized to 800000000 Hz !!!
    INFO: Bootloader_runCpu:147: CPU m4f0-0 is initialized to 400000000 Hz !!!
    INFO: Bootloader_runCpu:147: CPU a530-0 is initialized to 800000000 Hz !!!
    INFO: Bootloader_loadSelfCpu:199: CPU r5f0-0 is initialized to 800000000 Hz !!!
    INFO: Bootloader_loadSelfCpu:199: CPU r5f0-1 is initialized
    ø[IPC NOTIFY ECHO] Message exchange started by main core !!!
    [m4f0-0]     0.283025s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
    [r5f0-1]     0.001023s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
    [r5f0-1]     2.152508s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
    [r5f1-0]     0.435019s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
    [r5f1-0]     2.585876s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
    [r5f1-1]     0.360022s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
    [r5f1-1]     2.511288s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
    [a530-0]     0.209031s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
    [m4f0-0]     4.045393s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
    [IPC NOTIFY ECHO] All echoed messages received by main core from 5 remote cores !!!
    [IPC NOTIFY ECHO] Messages sent to each core = 1000000
    [IPC NOTIFY ECHO] Number of remote cores = 5
    All tests have passed!!
    [a530-0]     4.569557s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!

\endcond
