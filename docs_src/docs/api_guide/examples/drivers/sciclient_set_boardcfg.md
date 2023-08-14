# Sciclient Set BoardCfg {#EXAMPLES_DRIVERS_SCICLIENT_SET_BOARDCFG}

[TOC]

# Introduction

This is a special example which sends the sciclient boardcfg to the SYSFW running on Cortex M3. This example is mainly used in conjunction with the NO BOOT MODE, refer to \ref EVM_SOC_INIT_NOBOOT_MODE for more information on this.

The boardcfg is a SOC specific configuration data regarding the various system attributes controlled by the SYSFW. These include resources, power and clock, security etc.

If the SYSFW accepts the boardcfg sent, the test result is passed otherwise failed.

\note Please note that this example is valid only in cases where Cortex-M3 is already loaded with SYSFW. If SYSFW is not loaded, the example would fail. Typically this example is
used for SOC initialization via CCS in GP devices.

# Supported Combinations {#EXAMPLES_DRIVERS_SCICLIENT_SET_BOARDCFG_COMBOS}

\cond SOC_AM65X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
  ^             | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/sciclient/sciclient_set_boardcfg

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
DMSC Firmware Version 22.1.1--v2022.01a (Terrific Lla
DMSC Firmware revision 0x15
DMSC ABI revision 3.1

[SCICLIENT] ABI check PASSED
[SCICLIENT] Board Configuration with Debug enabled ...
[SCICLIENT] Common Board Configuration PASSED
[SCICLIENT] PM Board Configuration PASSED
[SCICLIENT] RM Board Configuration PASSED
[SCICLIENT] Security Board Configuration PASSED

DMSC Firmware Version 22.1.1--v2022.01a (Terrific Lla
DMSC Firmware revision 0x15
DMSC ABI revision 3.1

All tests have passed!!
\endcode
