# FreeRTOS POSIX {#EXAMPLES_KERNEL_FREERTOS_POSIX}

[TOC]

# Introduction

This example shows usage of POSIX APIs with FreeRTOS running underneath the POSIX layer.
Usage of POSIX thread and POSIX message queue is shown in this example.

The example does the below
- Creates worker message queues and worker threads
- Creates a dispatcher thread
- The dispatcher thread sends messages to each worker message queue
- After few iterations, a exit message is sent and dispatcher and worker threads then exit
- The main thread waits until all the created threads have finished.

# Supported Combinations

\cond SOC_AM64X
\attention A53 FREERTOS and A53 FREERTOS SMP support is experimental and is NOT supported by TI. \n
\endcond

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | m4fss0-0 freertos
 ^              | a53ss0-0 freertos
 ^              | a53ss0-0 freertos-smp
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/kernel/freertos/posix_demo

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | m4fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/kernel/freertos/posix_demo

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/kernel/freertos/posix_demo

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
Worker mqueue #[1879656984] - start
Worker mqueue #[1879657136] - start
Worker mqueue #[1879657288] - start
Worker mqueue #[1879657440] - start
Dispatcher thread - start
Dispatcher iteration #[0] -- Sending msg to worker mqueue #[1879656984].
Dispatcher iteration #[1] -- Sending msg to worker mqueue #[1879657136].
Dispatcher iteration #[2] -- Sending msg to worker mqueue #[1879657288].
Dispatcher iteration #[3] -- Sending msg to worker mqueue #[1879657440].
Dispatcher iteration #[4] -- Sending msg to worker mqueue #[1879656984].
Dispatcher iteration #[5] -- Sending msg to worker mqueue #[1879657136].
Dispatcher iteration #[6] -- Sending msg to worker mqueue #[1879657288].
Dispatcher iteration #[7] -- Sending msg to worker mqueue #[1879657440].
Dispatcher iteration #[8] -- Sending msg to worker mqueue #[1879656984].
Worker thread #[1879657592] -- Received eWORKER_CTRL_MSG_CONTINUE
Worker thread #[1879657592] -- Received eWORKER_CTRL_MSG_CONTINUE
Worker thread #[1879657592] -- Received eWORKER_CTRL_MSG_CONTINUE
Worker thread #[1879670840] -- Received eWORKER_CTRL_MSG_CONTINUE
Worker thread #[1879670840] -- Received eWORKER_CTRL_MSG_CONTINUE
Dispatcher iteration #[9] -- Sending msg to worker mqueue #[1879657136].
Worker thread #[1879662008] -- Received eWORKER_CTRL_MSG_CONTINUE
Worker thread #[1879662008] -- Received eWORKER_CTRL_MSG_CONTINUE
Worker thread #[1879662008] -- Received eWORKER_CTRL_MSG_CONTINUE
Dispatcher [0] -- Sending eWORKER_CTRL_MSG_EXIT to worker thread #[1879656984].
Dispatcher [1] -- Sending eWORKER_CTRL_MSG_EXIT to worker thread #[1879657136].
Dispatcher [2] -- Sending eWORKER_CTRL_MSG_EXIT to worker thread #[1879657288].
Worker thread #[1879657592] -- Finished. Exit now.
Worker thread #[1879666424] -- Received eWORKER_CTRL_MSG_CONTINUE
Worker thread #[1879666424] -- Received eWORKER_CTRL_MSG_CONTINUE
Worker thread #[1879666424] -- Finished. Exit now.
Dispatcher [3] -- Sending eWORKER_CTRL_MSG_EXIT to worker thread #[1879657440].
Worker thread #[1879662008] -- Finished. Exit now.
Worker thread #[1879670840] -- Finished. Exit now.
All tests have passed!!
\endcode
