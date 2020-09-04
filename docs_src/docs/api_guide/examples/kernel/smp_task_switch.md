# FreeRTOS SMP Task Switch Example {#EXAMPLES_KERNEL_FREERTOS_SMP_TASK_SWITCH}

[TOC]

# Introduction

This example shows usage of direct FreeRTOS APIs, i.e not via the DPL APIs.
It shows usage of task APIs, task notification APIs, semaphore and delay APIs.
It also shows how to signal to FreeRTOS task from ISRs.

The example does the below
- Creates two semaphores
- Creates two tasks, ping and pong
- Run tasks with core affinity
- Ping and pong tasks signal each other using semaphores and task notifications
- A HW ISR is also created and ping task is signaled from the ISR
- Task delay API usage is shown

# Supported Combinations

\cond SOC_AM64X
\attention A53 FREERTOS SMP support is experimental and is NOT supported by TI. \n
\endcond

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | a53ss0-0 freertos-smp
 Toolchain      | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/kernel/freertos/smp_task_switch

\endcond

# Steps to Run the Example
\note Create a sync group for cores when running an SMP example in CCS

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

execution time for task switches = 32937660 us
number of task switches = 2000000
time per task switch (semaphore give/take) = 16468 ns

execution time for task switches = 17862161 us
number of task switches = 2000000
time per task switch (direct-to-task notification give/take) = 8931 ns

execution time for task - ISR - task - task switches = 19287679 us
number of ISRs = 2000000
time per task - ISR - task switch (semaphore give/take) = 9643 ns

[FreeRTOS] ping task ... done !!!

All tests have passed!!
\endcode
