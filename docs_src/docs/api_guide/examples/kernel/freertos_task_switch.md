# FreeRTOS Task Switch Example {#EXAMPLES_KERNEL_FREERTOS_TASK_SWITCH}

[TOC]

# Introduction

This example shows usage of direct FreeRTOS APIs, i.e not via the DPL APIs.
It shows usage of task APIs, task notification APIs, semaphore and delay APIs.
It also shows how to signal to FreeRTOS task from ISRs.

The example does the below
- Creates two semaphores
- Creates two tasks, ping and pong
- Ping and pong tasks signal each other using semaphores and task notifications
- A HW ISR is also created and ping task is signaled from the ISR
- Task delay API usage is shown

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | m4fss0-0 freertos
 ^              | a53ss0-0 freertos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/kernel/freertos/task_switch

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | m4fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/kernel/freertos/task_switch

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/kernel/freertos/task_switch

\endcond

\cond SOC_AM65X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/kernel/freertos/task_switch

\endcond

# Steps to Run the Example

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
[FreeRTOS] ping task ... start !!!

execution time for task switches = 1526081 us
number of task switches = 2000000
time per task switch (semaphore give/take) = 763 ns

execution time for task switches = 1117010 us
number of task switches = 2000000
time per task switch (direct-to-task notification give/take) = 558 ns

execution time for task - ISR - task - task switches = 2140666 us
number of ISRs = 2000000
time per task - ISR - task switch (semaphore give/take) = 1070 ns

[FreeRTOS] ping task ... done !!!

All tests have passed!!
\endcode
