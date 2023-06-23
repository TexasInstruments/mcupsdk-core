# WFI - Standby mode demo {#EXAMPLES_KERNEL_NORTOS_WFI_DEMO}

[TOC]

# Introduction

This example showcases the use of WFI instruction to put one core in standby mode while the main core measures the current value before and after the standby period with the help of on-board I2c based INA current monitors. The example specifically focuses on a dual-core setup with cores R5 Cluster 0 and R5 Cluster 1 running in lockstep mode underlying no-RTOS environment on the supported SOC.

The example does the below
- Initialize the system and configure the necessary interrupts and IPC mechanisms
- Configure both cores to operate in lockstep mode, ensuring synchronized execution.
- Initialize and set-up the current monitors
- Main core measures the current value and send an IPC notify message to the secondary core to enter into wfi mode.
- Main core measures the current value again after the standby period and compare the difference with the expected value.

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/kernel/nortos/wfi_standby_demo/

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/kernel/nortos/wfi_standby_demo/

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/kernel/nortos/wfi_standby_demo/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref KERNEL_DPL_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[MAIN_Cortex_R5_0_0] [I2C] INA found at device address 0x40
[WFI] Values before wfi !!!
[WFI] power= 1435 mW , current=841300 uA
[WFI] Values after wfi !!!
[WFI] power= 1320 mW , current=773800 uA
[WFI] Actual Power Drop = 115 mW
All tests have passed!!
\endcode
