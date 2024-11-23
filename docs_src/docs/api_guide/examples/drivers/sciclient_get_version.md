# Sciclient Get Version {#EXAMPLES_DRIVERS_SCICLIENT_GET_VERSION}

[TOC]

# Introduction

\cond !SOC_AM62X
This example demonstrates how to check the version of SYSFW running on the Cortex M3 at run time. Since this is frequently done by bootloaders as a sanity check, there is an API included in the Sciclient driver for checking the SYSFW version.
\endcond

\cond SOC_AM62X
This example demonstrates how to check the version of SYSFW running on the Cortex R5 (DM R5) at run time. Since this is frequently done by bootloaders as a sanity check, there is an API included in the Sciclient driver for checking the SYSFW version.
\endcond

However, in this example this API is not used, instead the basic Sciclient_service API is used to send a "GET VERSION" request to the SYSFW and get a response back and print it out.

We also fetch the clock frequency of the current CPU using Sciclient and print that out as well, as additional sanity check.

# Supported Combinations {#EXAMPLES_DRIVERS_SCICLIENT_GET_VERSION_COMBOS}

\cond SOC_AM64X
\attention A53 NORTOS and A53 FREERTOS SMP support is experimental and is NOT supported by TI. \n
\endcond

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-1 nortos
 ^              | r5fss1-0 nortos
 ^              | r5fss1-1 nortos
 ^              | m4fss0-0 nortos
 ^              | a53ss0-0 nortos
 ^              | a53ss0-0 freertos-smp
 ^              | a53ss0-0 freertos
 ^              | a53ss0-1 freertos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/sciclient/sciclient_get_version

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-1 nortos
 ^              | r5fss1-0 nortos
 ^              | r5fss1-1 nortos
 ^              | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/sciclient/sciclient_get_version

\endcond

\cond SOC_AM62X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/sciclient/sciclient_get_version

\endcond

\cond SOC_AM65X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
  ^             | r5fss0-1 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/sciclient/sciclient_get_version

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

Shown below is a sample output when the application is run,

\cond !SOC_AM62X && !SOC_AM65X
\code
DMSC Firmware Version 21.1.1--v2021.01a (Terrific Lla
Firmware revision 0x15
ABI revision 3.1
[SCICLIENT] CPU clock frequency = 800000000 Hz
All tests have passed!!
\endcode
\endcond

\cond SOC_AM62X
\code
DMSC Firmware Version 21.5.1--w2022.07-am62x (Terrifi
Firmware revision 0x15
ABI revision 3.1
[SCICLIENT] CPU clock frequency = 400000000 Hz
All tests have passed!!
\endcode
\endcond

\cond SOC_AM65X
\code
DMSC Firmware Version 22.1.1--v2022.01a (Terrific Lla
Firmware revision 0x15
ABI revision 3.1
[SCICLIENT] CPU clock frequency = 400000000 Hz
All tests have passed!!
\endcode
\endcond
\cond SOC_AM64X
**a53 core0:** on UART0(/dev/ttyUSB0)
\code
DMSC Firmware Version 10.0.8--v10.00.08 (Fiery Fox)
Firmware revision 0xa
ABI revision 4.0
[SCICLIENT] CPU clock frequency = 800000000 Hz 
All tests have passed on a53_core0!!
\endcode
**a53 core1:** on UART1(/dev/ttyUSB2)
\code
DMSC Firmware Version 10.0.8--v10.00.08 (Fiery Fox)
Firmware revision 0xa
ABI revision 4.0
[SCICLIENT] CPU clock frequency = 800000000 Hz 
All tests have passed on a53_core1!!
\endcode
\endcond