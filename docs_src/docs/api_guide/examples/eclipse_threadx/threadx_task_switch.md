# ThreadX Task Switch Example {#EXAMPLES_ECLIPSE_THREADX_THREADX_TASK_SWITCH}

[TOC]

# Introduction

This example shows usage of direct ThreadX APIs, i.e not via the DPL APIs.
It shows usage of task APIs, semaphore and delay APIs. It also shows how to signal to FreeRTOS task from ISRs.

The example does the below
- Creates two semaphores
- Creates two tasks, ping and pong
- Ping and pong tasks signal each other using semaphores and task notifications
- A HW ISR is also created and ping task is signaled from the ISR
- Task delay API usage is shown

# Supported Combinations

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/eclipse_threadx/threadx/task_switch

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref ECLIPSE_THREADX_THREADX

# Sample Output

Shown below is a sample output when the application is run,

\code
[THREADX TASK SWITCH] ping task ... start !!!

execution time for task switches = 1526081 us
number of task switches = 2000000
time per task switch (semaphore give/take) = 763 ns

execution time for task switches = 1117010 us
number of task switches = 2000000
time per task switch (direct-to-task notification give/take) = 558 ns

execution time for task - ISR - task - task switches = 2140666 us
number of ISRs = 2000000
time per task - ISR - task switch (semaphore give/take) = 1070 ns

[THREADX TASK SWITCH] ping task ... done !!!

All tests have passed!!
\endcode
