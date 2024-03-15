# Dhrystone benchmarking demo {#EXAMPLES_KERNEL_FREERTOS_DHRYSTONE_BENCHMARK}

[TOC]

# Introduction

This example shows an application for benchmarking the FreeRTOS SMP kernel and single core FreeRTOS kernel using dhrystone benchmarking.

The application runs the the dhrystone benchmarking demo using the given number of threads and iterations and logs the results as the number of dhrystones run per second.

The application runs with thread number equal to 1, 2, 5 and 10, with iteration number 30000000.

# Supported Combinations

\cond SOC_AM64X
\attention A53 NORTOS, A53 FREERTOS and A53 FREERTOS SMP support is experimental and is NOT supported by TI. \n
\endcond

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | a53ss0-0 freertos
 ^              | a53ss0-0 freertos-smp
 Toolchain      | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/kernel/freertos/dhrystone_benchmark/

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE and \ref SMP_FREERTOS_GUIDE

# See Also

\ref KERNEL_DPL_PAGE

# Sample Output

Shown below is a sample output when the benchmark application is run,

\code
[DHRYSTONE BENCHMARKING] Iterations                       : 30000000
[DHRYSTONE BENCHMARKING] Threads                          : 1
[DHRYSTONE BENCHMARKING] Dhrystones per second            : 9629845.0

[DHRYSTONE BENCHMARKING] Iterations                       : 30000000
[DHRYSTONE BENCHMARKING] Threads                          : 2
[DHRYSTONE BENCHMARKING] Dhrystones per second            : 9629879.0

[DHRYSTONE BENCHMARKING] Iterations                       : 30000000
[DHRYSTONE BENCHMARKING] Threads                          : 5
[DHRYSTONE BENCHMARKING] Dhrystones per second            : 9629927.0

[DHRYSTONE BENCHMARKING] Iterations                       : 30000000
[DHRYSTONE BENCHMARKING] Threads                          : 10
[DHRYSTONE BENCHMARKING] Dhrystones per second            : 9629947.0

All tests have passed!!
\endcode
