# Sciclient CCS Init {#EXAMPLES_DRIVERS_SCICLIENT_CCS_INIT}

[TOC]

# Introduction

This is a special soc initialization example which loads the System Firmware on Cortex M3, sends the sciclient boardcfg to the SYSFW running on M3 and initializes the other cores users to connect and debug. This example is used with CCS based initialization for HS-FS devices. You should run this soc initialization binary in the R5F0-0 core only after powering on the board in DEV BOOT MODE.

The SYSFW will be part of the application as a hex array and will be loaded to M3 using `Sciclient_loadFirmware` API. The boardcfg is a SOC specific configuration data regarding the various system attributes controlled by the SYSFW. These include resources, power and clock, security etc. If the SYSFW loads and accepts the boardcfg sent, the test result is passed otherwise failed.

Sciclient CCS Init honors the various core variants of the device. The core variants might have lesser number of cores, Sciclient CCS Init will not initialize any disabled core. This is achieved by reading JTAG USER ID register of the SOC to identify the features.

# Supported Combinations {#EXAMPLES_DRIVERS_SCICLIENT_CCS_INIT_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/sciclient/sciclient_ccs_init

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/sciclient/sciclient_ccs_init

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_SCICLIENT_PAGE

# Sample Output

\code

\endcode