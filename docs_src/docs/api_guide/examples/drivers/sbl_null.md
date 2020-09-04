# SBL NULL {#EXAMPLES_DRIVERS_SBL_NULL}

[TOC]

# Introduction

This is a bootloader which does SOC initializations and put all the cores in WFI. This example is what is referred to as the SOC initialization binary in \ref EVM_FLASH_SOC_INIT. The SBL NULL does not load any images on the cores. For more information on how this example is being used, refer \ref EVM_FLASH_SOC_INIT

\cond SOC_AM64X || SOC_AM243X
SBL NULL honors the various core variants of the device. The core variants might have lesser number of cores, SBL NULL will not initialize any disabled core. This is achieved by reading JTAG USER ID register of the SOC to identify the features.
\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_SBL_NULL_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_null

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_null

\endcond

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_null

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/boot/sbl_null

\endcond

# Steps to Run the Example

Since this is a bootloader and is used as a SOC initialization binary, the example will be run every time you boot an application using this example. It is generally run from a boot media (OSPI Flash, SD Card) unlike other examples which are usually loaded with CCS. Nevertheless, you can build this example like you do for the others using makefile or build it via CCS by importing as a project.

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Flash this bootloader by following steps mentioned in \ref EVM_FLASH_SOC_INIT

# See Also

\ref DRIVERS_BOOTLOADER_PAGE

# Sample Output
\cond SOC_AM64X
\code
Starting NULL Bootloader ...

DMSC Firmware Version 21.5.0--v2021.05 (Terrific Llam
DMSC Firmware revision 0x15
DMSC ABI revision 3.1

INFO: Bootloader_runCpu:147: CPU r5f1-0  is initialized to 800000000 Hz !!!
INFO: Bootloader_runCpu:147: CPU r5f1-1 is initialized to 800000000 Hz !!!
INFO: Bootloader_runCpu:147: CPU m4f0-0 is initialized to 400000000 Hz !!!
INFO: Bootloader_runCpu:147: CPU a530-0 is initialized to 800000000 Hz !!!
INFO: Bootloader_runCpu:147: CPU a530-1 is initialized to 800000000 Hz !!!
INFO: Bootloader_loadSelfCpu:199: CPU r5f0-0 is initialized to 800000000 Hz !!!
INFO: Bootloader_loadSelfCpu:199: CPU r5f0-1 is initialized to 800000000 Hz !!!
INFO: Bootloader_runSelfCpu:216: All done, reseting self ...
\endcode
\endcond

\cond SOC_AM243X
\code
Starting NULL Bootloader ...

DMSC Firmware Version 21.1.1--v2021.01a (Terrific Lla
DMSC Firmware revision 0x15
DMSC ABI revision 3.1

INFO: Bootloader_runCpu:151: CPU r5f1-0  is initialized to 800000000 Hz !!!
INFO: Bootloader_runCpu:151: CPU r5f1-1 is initialized to 800000000 Hz !!!
INFO: Bootloader_runCpu:151: CPU m4f0-0 is initialized to 400000000 Hz !!!
INFO: Bootloader_loadSelfCpu:214: CPU r5f0-0 is initialized to 800000000 Hz !!!
INFO: Bootloader_loadSelfCpu:214: CPU r5f0-1 is initialized to 800000000 Hz !!!
INFO: Bootloader_runSelfCpu:235: All done, reseting self ...
\endcode
\endcond

\cond SOC_AM263X
\code
Starting NULL Bootloader ...
INFO: Bootloader_runCpu:150: CPU r5f1-1 is initialized to 400000000 Hz !!!
INFO: Bootloader_runCpu:150: CPU r5f1-0 is initialized to 400000000 Hz !!!
INFO: Bootloader_runCpu:150: CPU r5f0-1 is initialized to 400000000 Hz !!!
NULL Bootloader Execution Complete...
INFO: Bootloader_loadSelfCpu:202: CPU r5f0-0 is initialized to 400000000 Hz !!!
INFO: Bootloader_runSelfCpu:219: All done, reseting self ...
\endcode
\endcond
