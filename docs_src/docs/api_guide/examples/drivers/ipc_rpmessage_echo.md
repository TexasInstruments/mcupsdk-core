# IPC RP Message Echo {#EXAMPLES_DRIVERS_IPC_RPMESSAGE_ECHO}

[TOC]

# Introduction

This example shows usage of RP Message APIs to each multi-byte messages between different CPUs

In this example,
- One "main" CPU, sends multi-byte messages to other "remote" CPUs using the RP Message APIs
- The "remote" CPUs, then echo back the same message to the main CPUs
- Once all messages are echoed the main CPU exits. The remote CPUs remain waiting to receive new messages.

# Supported Combinations

\cond SOC_AM64X
\attention A53 NORTOS support is experimental and is NOT supported by TI. \n
\endcond

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-1 nortos
 ^              | r5fss1-0 nortos
 ^              | r5fss1-1 nortos
 ^              | m4fss0-0 nortos
 ^              | a53ss0-0 nortos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/ipc/ipc_rpmsg_echo

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-1 nortos
 ^              | r5fss1-0 nortos
 ^              | r5fss1-1 nortos
 ^              | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ipc/ipc_rpmsg_echo

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-1 nortos
 ^              | r5fss1-0 nortos
 ^              | r5fss1-1 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ipc/ipc_rpmsg_echo

\endcond

# Steps to Run the Example

\note This is a `system` or multi-core project, so refer to system project build instructions for CCS project or makefiles when building the example.

- **When using CCS projects to build**, import the system CCS project
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE). This will build all the dependant CPU projects as well
- **When using makefiles to build**, build the system makefile using
  make command (see \ref MAKEFILE_BUILD_PAGE). This will build all the dependant CPU makefiles as well.
\if SOC_AM263X
- Launch a CCS debug session, follow the steps for running multi core applications in \ref PREREQUISITES and run the executables, see \ref CCS_LAUNCH_PAGE
\else
- Launch a CCS debug session and run the executables, see \ref CCS_LAUNCH_PAGE
\endif
- This is a multi-core example. Hence the executables should be loaded and run for all the above mentioned cores
- The application has a sync mechanism at the start which waits for all cores to start before doing the test. Hence the cores can be loaded and run in any sequence.

# See Also

\ref DRIVERS_IPC_RPMESSAGE_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\if SOC_AM64X
\code
[m4f0-0]     0.019049s : [IPC RPMSG ECHO] Remote Core waiting for messages from main core ... !!!
[r5f0-1]     0.018063s : [IPC RPMSG ECHO] Remote Core waiting for messages from main core ... !!!
[r5f1-0]     0.019063s : [IPC RPMSG ECHO] Remote Core waiting for messages from main core ... !!!
[r5f1-1]     0.019063s : [IPC RPMSG ECHO] Remote Core waiting for messages from main core ... !!!
[a530-0]     0.000043s : [IPC RPMSG ECHO] Remote Core waiting for messages from main core ... !!!
[IPC RPMSG ECHO] Message exchange started by main core !!!
[IPC RPMSG ECHO] All echoed messages received by main core from 5 remote cores !!!
[IPC RPMSG ECHO] Messages sent to each core = 100000
[IPC RPMSG ECHO] Number of remote cores = 5
[IPC RPMSG ECHO] Total execution time = 3479561 usecs
[IPC RPMSG ECHO] One way message latency = 3479 nsec
All tests have passed!!
\endcode
\elseif SOC_AM243X
\code
[m4f0-0]     0.002039s : [IPC RPMSG ECHO] Remote Core waiting for messages from main core ... !!!
[r5f0-1]     0.001056s : [IPC RPMSG ECHO] Remote Core waiting for messages from main core ... !!!
[r5f1-0]     0.002056s : [IPC RPMSG ECHO] Remote Core waiting for messages from main core ... !!!
[r5f1-1]     0.002056s : [IPC RPMSG ECHO] Remote Core waiting for messages from main core ... !!!
[IPC RPMSG ECHO] Message exchange started by main core !!!
[IPC RPMSG ECHO] All echoed messages received by main core from 4 remote cores !!!
[IPC RPMSG ECHO] Messages sent to each core = 100000
[IPC RPMSG ECHO] Number of remote cores = 4
[IPC RPMSG ECHO] Total execution time = 2918245 usecs
[IPC RPMSG ECHO] One way message latency = 3647 nsec
All tests have passed!!
\endcode
\else
\code
[r5f0-1]     1.019992s : [IPC RPMSG ECHO] Remote Core waiting for messages from main core ... !!!
[r5f1-0]     0.512016s : [IPC RPMSG ECHO] Remote Core waiting for messages from main core ... !!!
[r5f1-1]     0.000055s : [IPC RPMSG ECHO] Remote Core waiting for messages from main core ... !!!
[IPC RPMSG ECHO] Message exchange started by main core !!!
[IPC RPMSG ECHO] All echoed messages received by main core from 3 remote cores !!!
[IPC RPMSG ECHO] Messages sent to each core = 100000
[IPC RPMSG ECHO] Number of remote cores = 3
[IPC RPMSG ECHO] Total execution time = 4203377 usecs
[IPC RPMSG ECHO] One way message latency = 7005 nsec
All tests have passed!!
\endcode
\endif
