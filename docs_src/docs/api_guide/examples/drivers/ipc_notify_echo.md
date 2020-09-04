# IPC Notify Echo {#EXAMPLES_DRIVERS_IPC_NOTIFY_ECHO}

[TOC]

# Introduction

This example shows usage of IPC Notify APIs for exchanging 28b messages between multiple CPUs.

In this example,
- One "main" CPU, sends 28b messages to other "remote" CPUs using the IPC notify API
- The "remote" CPUs, then echo back the message to the main CPUs
- Once all messages are echoed all the CPUs exit

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
 Example folder | examples/drivers/ipc/ipc_notify_echo

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
 Example folder | examples/drivers/ipc/ipc_notify_echo

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
 Example folder | examples/drivers/ipc/ipc_notify_echo

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

\ref DRIVERS_IPC_NOTIFY_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\if SOC_AM64X
\code
[IPC NOTIFY ECHO] Message exchange started by main core !!!
[m4f0-0]     0.283025s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
[r5f0-1]     0.001023s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
[r5f0-1]     2.152508s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
[r5f1-0]     0.435019s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
[r5f1-0]     2.585876s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
[r5f1-1]     0.360022s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
[r5f1-1]     2.511288s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
[a530-0]     0.209031s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
[m4f0-0]     4.045393s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
[IPC NOTIFY ECHO] All echoed messages received by main core from 5 remote cores !!!
[IPC NOTIFY ECHO] Messages sent to each core = 1000000
[IPC NOTIFY ECHO] Number of remote cores = 5
All tests have passed!!
[a530-0]     4.569557s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
\endcode
\elseif SOC_AM243X
\code
[IPC NOTIFY ECHO] Message exchange started by main core !!!
[m4f0-0]     0.249022s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
[r5f0-1]     0.473040s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
[r5f0-1]     2.645696s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
[r5f0-1]     2.645712s : All tests have passed!!
[r5f1-0]     0.366043s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
[r5f1-0]     2.538699s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
[r5f1-0]     2.538715s : All tests have passed!!
[r5f1-1]     0.296028s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
[r5f1-1]     2.468686s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
[r5f1-1]     2.468702s : All tests have passed!!
[IPC NOTIFY ECHO] All echoed messages received by main core from 4 remote cores !!!
[IPC NOTIFY ECHO] Messages sent to each core = 1000000
[IPC NOTIFY ECHO] Number of remote cores = 4
[m4f0-0]     4.246567s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
[m4f0-0]     4.246586s : All tests have passed!!
\endcode
\else
\code
[IPC NOTIFY ECHO] Message exchange started by main core !!!
[IPC NOTIFY ECHO] All echoed messages received by main core from 3 remote cores !!!
[IPC NOTIFY ECHO] Messages sent to each core = 1000000
[IPC NOTIFY ECHO] Number of remote cores = 3
All tests have passed!!
[r5f0-1]     2.965606s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
[r5f0-1]     8.203494s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
[r5f1-0]     6.069463s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
[r5f1-0]    11.306840s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
[r5f1-1]     8.245586s : [IPC NOTIFY ECHO] Remote Core waiting for messages from main core ... !!!
[r5f1-1]    13.483076s : [IPC NOTIFY ECHO] Remote core has echoed all messages !!!
\endcode
\endif
