# DSP LIB Add32 {#EXAMPLES_DRIVERS_DSPLIB_ADD32}

[TOC]

# Introduction

This example demonstrates the the usage of dsplib functions from MCU SDK.
This is a ADD32 example taken from DSP LIB package itself to demonstrate building with the MCU SDK build system.

# Supported Combinations {#EXAMPLES_DRIVERS_DSPLIB_ADD32_COMBOS}

\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | c66ss0 nortos
 Toolchain      | ti-c6000
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/dsplib/add32

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[DSPLIB] ADD32 Sample Application Started...
DSP_add32	Iter#: 1	Result Successful (r_i)	NX = 8	natC: 112	optC: 39
DSP_add32	Iter#: 2	Result Successful (r_i)	NX = 16	natC: 150	optC: 42
DSP_add32	Iter#: 3	Result Successful (r_i)	NX = 32	natC: 292	optC: 62
DSP_add32	Iter#: 4	Result Successful (r_i)	NX = 64	natC: 562	optC: 102
DSP_add32	Iter#: 5	Result Successful (r_i)	NX = 128	natC: 1102	optC: 182
DSP_add32	Iter#: 6	Result Successful (r_i)	NX = 256	natC: 2168	optC: 342
Memory:  8498912 bytes
Cycles:  5/4*Nx + 22
[DSPLIB] ADD32 Sample Application Completed!!
All tests have passed!!
\endcode

