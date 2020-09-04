# Driver Porting Layer (DPL) demo {#EXAMPLES_KERNEL_DPL_DEMO}

[TOC]

# Introduction

This example shows a simple application, which setups the CPU and the underlying RTOS or no-RTOS
environment on the supported SOC.

The example does the below
- Setup a timer with system tick of 1ms, setup MPU and cache (if available)
- Create a RTOS task (in RTOS example)
- Trigger a ISR and signal a semaphore from ISR to main task
- Show usage of clock delay
- Show usage of cache APIs
- Show usage of heap APIs

# Supported Combinations

\cond SOC_AM64X
\attention A53 NORTOS, A53 FREERTOS and A53 FREERTOS SMP support is experimental and is NOT supported by TI. \n
\endcond

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 ^              | m4fss0-0 nortos
 ^              | m4fss0-0 freertos
 ^              | a53ss0-0 nortos
 ^              | a53ss0-0 freertos
 ^              | a53ss0-0 freertos-smp
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/kernel/dpl/dpl_demo/

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 ^              | m4fss0-0 nortos
 ^              | m4fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/kernel/dpl/dpl_demo/

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/kernel/dpl/dpl_demo/

\endcond

\cond SOC_AM62X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | m4fss0-0 nortos
 ^              | m4fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/kernel/dpl/dpl_demo/

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
[DPL] Hwi post ...
[DPL] Hwi post ... DONE !!!
[DPL] Sleep for 100 msecs ...
[DPL] Sleep ... DONE (Measured time = 100000 usecs, CPU cycles = 80000055 ) !!!
[DPL] Running cache operations ...
[DPL] Running cache operations ... DONE !!!
[DPL] Heap free size = 1984 bytes
[DPL] Allocated 1023 bytes @ 0x80010440, heap free size = 896 bytes
[DPL] Free'ed 1023 bytes @ 0x80010440, heap free size = 1984 bytes
All tests have passed!
\endcode
