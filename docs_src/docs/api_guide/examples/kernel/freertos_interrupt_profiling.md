# FreeRTOS Interrupt Profiling Example {#EXAMPLES_KERNEL_FREERTOS_INTERRUPT_PROFILING}

[TOC]

# Introduction

This example demonstrates profiling of interrupts. Two tasks are created, three interrupts of different priority levels are configured. The interrupts are triggered to cause nesting & the behaviour of interrupts is verified against a reference. If the behaviour of interrupts is as expected, the total ISR load and CPU load is displayed and the example is marked as passed.

\cond SOC_AM64X || SOC_AM243X

\note Make sure "#define INTR_PROF" is present in "/source/drivers/hw_include/am64x_am243x/soc_config.h", to enable capturing of interrupt trace. Else the displayed LOAD values will be 0.

\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294X

\note Make sure "#define INTR_PROF" is present in "/source/drivers/hw_include/@VAR_SOC_NAME_LOWER/soc_config.h", to enable capturing of interrupt trace. Else the displayed LOAD values will be 0.

\endcond

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/kernel/freertos/interrupt_profiling

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/kernel/freertos/interrupt_profiling

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/kernel/freertos/interrupt_profiling

\endcond

\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/kernel/freertos/interrupt_profiling

\endcond

\cond SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/kernel/freertos/interrupt_profiling

\endcond

# Steps to Run the Example

- **When the INTR_PROF macro is modified**, the library needs to be re-built. This can be done
  only using makefiles. (see \ref MAKEFILE_BUILD_PAGE).
- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref KERNEL_FREERTOS_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[FreeRTOS] profiling ... start !!!
[FreeRTOS] profiling ... done !!!
[FreeRTOS] profiling ... completed !!!

LOAD: ISR  =  0. 0 %
LOAD: CPU =  0.60 %
LOAD: ping = 0.27 %
LOAD: pong = 0.10 %

All tests have passed!!
\endcode
