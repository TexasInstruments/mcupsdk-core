# CSIRX Internal Capture {#EXAMPLES_DRIVERS_CSIRX_INTERNAL_CAPTURE}

[TOC]

# Introduction

In this example, we capture frames using the CSIRX interface in internal debug or test mode.
In this mode, there is no external sensor attached. The CSIRX HW has a internal test mode which is used
to construct and send a CSIRX frame to the CSIRX HW. The CSI HW captures the frame using its HW logic + DMA
and outputs to the frame to memory buffer that is specified.

Here the CSIRX HW is setup using SysConfig. Two DMA contexts are created.
- Context 0, to switch DMA buffers when a complete frame is captured in the DMA buffer, also referred to as frame mode switching.
- Context 1, to switch DMA buffers when a 2 lines are captured in the DMA buffer, , also referred to as line mode switching.

Line mode switching is used when the frame is large and cannot be buffered in the limited internal memory of the SOC.

We show both the buffer switching examples in two different function calls in the example below.

# Supported Combinations

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | c66ss0 nortos
 Toolchain      | ti-arm-clang, ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/csirx/csirx_internal_loopback/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_CSIRX_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
Frame switching mode capture ... starting
Frame switching mode capture ... done !!!
Line switching mode capture ... starting
Line switching mode capture ... done !!!
All tests have passed!!
\endcode

