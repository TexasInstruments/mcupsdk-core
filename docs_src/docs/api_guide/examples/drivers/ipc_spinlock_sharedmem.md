# IPC Shared Memory Mutex using Spinlock {#EXAMPLES_DRIVERS_IPC_SPINLOCK_SHAREDMEM}

[TOC]

# Introduction

This example shows shared memory access between multiple cores in an atomic way using spinlock.
Shared memory is allocated across two cores - using gUserSharedMem variable.

In this example, this memory is just a counter for illustration purpose.
Each core does a read-modify write to this variable in a continuous manner.

Spinlocks are used to protect this shared memory access. This ensures
that the access is atomic and the shared variable is incremented properly.
If proper protection is done, then at the end of the iteration the counter
value will be N times loop count where N is number of cores acting on the
variable.

When iteration count reaches SPINLOCK_TEST_LOOPCNT, the example is completed.

# Supported Combinations {#EXAMPLES_DRIVERS_IPC_SPINLOCK_SHAREDMEM_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-1 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/ipc/ipc_spinlock_sharedmem

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-1 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ipc/ipc_spinlock_sharedmem

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-1 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ipc/ipc_spinlock_sharedmem

\endcond
# Steps to Run the Example

\note This is a `system` or multi-core project, so refer to system project build instructions for CCS project or makefiles when building the example.

- **When using CCS projects to build**, import the system CCS project
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE). This will build all the dependant CPU projects as well
- **When using makefiles to build**, build the system makefile using
  make command (see \ref MAKEFILE_BUILD_PAGE). This will build all the dependant CPU makefiles as well.
\if SOC_AM263X || SOC_AM263PX
- Launch a CCS debug session, run the hotmenu item "AM263x Device Configuration -> MailBox_MEM_Init" in the GEL file and run the executables, see \ref CCS_LAUNCH_PAGE
\else
- Launch a CCS debug session and run the executables, see \ref CCS_LAUNCH_PAGE
\endif
- This is a multi-core example. Hence the executables should be loaded and run for all the above mentioned cores
- The application has a sync mechanism at the start which waits for all cores to start before doing the test. Hence the cores can be loaded and run in any sequence.

# See Also

\ref DRIVERS_SPINLOCK_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[IPC Spinlock Sharedmem] Example started ...
Waiting for all cores to start ...
Waiting for all cores to complete ...
[r5f0-1]     0.000016s : [IPC Spinlock Sharedmem] Example started ...
All tests have passed!!
[r5f0-1]     0.000036s : Waiting for all cores to start ...
[r5f0-1]     0.199144s : Waiting for all cores to complete ...
[r5f0-1]     0.199157s : All tests have passed!!
\endcode
