# Clock Measurement {#EXAMPLES_CLOCK_MEASUREMENT}

[TOC]

# Introduction

This example application is used to measure the clock frequency of the core. The application reads the clock frequency and outputs to the console.
Probing the OBS_CLKOUT pin through oscilloscope reports the divided clock frequency allowing the direct verification.The clock frequency is divided by the fixed divide ratio of 5(out = clk_freq/5).

# Supported Combinations

\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | c66ss0_nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/clock_measurement
\endcond

# Steps to Run the Example

\note This is a `system` or multi-core project, so refer to system project build instructions for CCS project or makefiles when building the example.

- **When using CCS projects to build**, import the system CCS project
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE). This will build all the dependant CPU projects as well
- **When using makefiles to build**, build the system makefile using
  make command (see \ref MAKEFILE_BUILD_PAGE). This will build all the dependant CPU makefiles as well.
- Launch a CCS debug session and run the executables, see \ref CCS_LAUNCH_PAGE

# Sample Output

Shown below is a sample output when the application is run,

R5F output:
\code
MSS Frequency is 400000000 Hz, 400 MHz
\endcode

C66x output:
\code
DSP Frequency is 550000000 Hz, 550 MHz
\endcode
