# SDL MCRC Auto CPU {#EXAMPLES_SDL_MCRC_AUTO_CPU}

[TOC]

# Introduction

This example demonstrates the usage of the MCRC module. The example shows how to setup and use the MCRC controller in the auto mode of operation.
Shows the generation of matching CRC as well as non-matching signature due to insertion of error in the block of information on which the CRC
is being performed or in the signature provided to the PSA Signature Register (or both, depending on the mode).

Use Cases
---------

 Use Case | Description
 ---------|------------
 UC-1     | Auto CPU-mode signature compute for Channel 1 and comparison against known value.
 UC-2     | Auto CPU-mode signature compute for Channel 2 and comparison against known value.

# Supported Combinations {#EXAMPLES_SDL_MCRC_AUTO_CPU_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss1-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/mcrc/mcrc_auto_cpu/

\endcond

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos, c66ss0 nortos
 Toolchain      | ti-arm-clang, ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/mcrc/mcrc_auto_cpu/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_MCRC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\cond (SOC_AM263X || SOC_AM263PX)
\code

 MCRC Test Application

MCRC AUTO CPU mode on Channel 1: Transfer Test Started...

Calculating Reference MCRC signature Value.
 MCRC signature value : 0x1c133adab4dd50fU
MCRC Full Mode Computation Time: 1085us

Sector signature matches - Passed
Calculated MCRC signature value : 0x01c133adab4dd50fU

EDMA Data transfer completed !!
MCRC Auto Mode Computation Time: 10us

MCRC AUTO CPU mode on Channel 2: Transfer Test Started...

Using Pre-Defined Reference MCRC signature Value.

Pre-defined MCRC signature value : 0x1c133adab4dd50fU

Sector signature matches - Passed
Calculated MCRC signature value : 0x01c133adab4dd50fU

EDMA Data transfer completed !!
MCRC Auto Mode Computation Time: 10us

MCRC AUTO CPU mode on Channel 3: Transfer Test Started...

Using Pre-Defined Reference MCRC signature Value.

Pre-defined MCRC signature value : 0x1c133adab4dd50fU

Sector signature matches - Passed
Calculated MCRC signature value : 0x01c133adab4dd50fU

EDMA Data transfer completed !!
MCRC Auto Mode Computation Time: 10us

MCRC AUTO CPU mode on Channel 4: Transfer Test Started...

Using Pre-Defined Reference MCRC signature Value.

Pre-defined MCRC signature value : 0x1c133adab4dd50fU

Sector signature matches - Passed
Calculated MCRC signature value : 0x01c133adab4dd50fU

EDMA Data transfer completed !!
MCRC Auto Mode Computation Time: 10us

Test Name: MCRCAutoCPUfunctest_main  PASSED

 All tests have passed.
\endcode
\endcond

\cond (SOC_AWR294X)||(SOC_AM273X)
\code
[Cortex_R5_0]
 MCRC Test Application

MCRC AUTO CPU mode on Channel 1: Transfer Test Started...

Calculating Reference MCRC signature Value.
 MCRC signature value : 0x1c133adab4dd50fU
MCRC Full Mode Computation Time: 4619us

Sector signature matches - Passed
Calculated MCRC signature value : 0x01c133adab4dd50fU

EDMA Data transfer completed !!
MCRC Auto Mode Computation Time: 8us

MCRC AUTO CPU mode on Channel 2: Transfer Test Started...

Using Pre-Defined Reference MCRC signature Value.

Pre-defined MCRC signature value : 0x1c133adab4dd50fU

Sector signature matches - Passed
Calculated MCRC signature value : 0x01c133adab4dd50fU

EDMA Data transfer completed !!
MCRC Auto Mode Computation Time: 8us

Test Name: MCRCAutoCPU_main  PASSED

 All tests have passed.
\endcode
\endcond
