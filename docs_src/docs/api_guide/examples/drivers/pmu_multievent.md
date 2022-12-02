# PMU Multievent {#EXAMPLES_DRIVERS_PMU_MULTIEVENT}

[TOC]

# Introduction

This example demonstrates the usage of the R5 PMU Driver. The PMU supports three 
event counters in additional to the cycle counter. So in total we could profile a 
block of code / or a function for four metrics. The cycle counter is a dedicated 
counter, but the other three are configurable for various PMU events. In this 
example, we configure the counters to count these events:

- ICache Miss
- DCache Miss
- DCache Access

# Supported Combinations {#EXAMPLES_DRIVERS_PMU_MULTIEVENT_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-1 nortos
 ^              | r5fss1-0 nortos
 ^              | r5fss1-1 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/pmu/pmu_multievent

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-1 nortos
 ^              | r5fss1-0 nortos
 ^              | r5fss1-1 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/pmu/pmu_multievent

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-1 nortos
 ^              | r5fss1-0 nortos
 ^              | r5fss1-1 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/pmu/pmu_multievent

\endcond

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-1 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/pmu/pmu_multievent

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Please select the UART port used for console as mentioned in \ref CCS_UART_TERMINAL

# See Also

\ref DRIVERS_UART_PAGE

# Sample Output

Shown below is a sample output when the application is run,
Please note that application prints in both CCS and UART console.

UART Console:
\code
[PMU Multievent] Starting...
CRC Value: 37090
Profile Point: Fxn2
Cycle Count: 831268
ICache Miss Count: 35
DCache Access Count: 63002
DCache Miss Count: 831280

Profile Point: Fxn1
Cycle Count: 3702
ICache Miss Count: 3
DCache Access Count: 1044
DCache Miss Count: 3717

Profile Point: Fxn2
Cycle Count: 831268
ICache Miss Count: 35
DCache Access Count: 63002
DCache Miss Count: 831280

Profile Point: Fxn3
Cycle Count: 9671
ICache Miss Count: 4
DCache Access Count: 2324
DCache Miss Count: 9654

[PMU Multievent] Done...
\endcode
