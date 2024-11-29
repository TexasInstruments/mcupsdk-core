# IPC Notify Echo Example With OptiShare {#EXAMPLES_DRIVERS_IPC_NOTIFY_ECHO_OPTISHARE}

[TOC]

# Introduction

This example shows usage of IPC Notify APIs for exchanging 28b messages between multiple CPUs with Optishare enabled. 

In this example,
- One "main" CPU, sends 28b messages to other "remote" CPUs using the IPC notify API
- The "remote" CPUs, then echo back the message to the main CPUs
- Once all messages are echoed all the CPUs exit
- With the application of OptiShare, there is Shared code and data. 

# Supported Combinations

\cond SOC_AM263PX || SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-1 nortos
 ^              | r5fss1-0 nortos
 ^              | r5fss1-1 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ipc/ipc_notify_echo_optishare

\endcond
# Steps to Run the Example

\note This is a `system` or multi-core project, so refer to system project build instructions for CCS project or makefiles when building the example.

## Building with Makefile 

- build the system makefile using
  make command (see \ref MAKEFILE_BUILD_PAGE). This will build all the dependent CPU makefiles as well.
  To build with OptiShare enabled, use command 
\code 
  make sso
\endcode 
Here, it builds `sso` rule which does the normal build but also does the further OptiShare Processing.

## Loading And Running the example using uart_uniflash tool

- Flash the application using uart_uniflash.py. Here Flash the binaries with extension *.optishare.mcelf and *.optishare.mcelf_xip from system_freertos_nortos folder. 
- More on how to flash can be read at \ref TOOLS_FLASH_UART_UNIFLASH
- Change the boot mode to OSPI boot mode and do the board reset to see the output as shown in sample output.

## Loading And Running the example using CCS

Once this example has been built using Makefile, loadAll.js JavaScript can be used to load optishare example.

1. Load the debugging session as described in \ref CCS_LAUNCH_PAGE 
2. Open file {sdk_path}/examples/drivers/ipc/ipc_notify_echo_optishare/am263px-cc/system_freertos_nortos/loadAll.js in any editor and edit the variable `sdkPath` by adding the correct SDK path.  
3. Open the scripting console. Goto "CCS Toolbar > View > Scripting Console"
4. Type the below command in the scripting console and press "enter", to load
  \code 
    loadJSFile "{sdk_path}/examples/drivers/ipc/ipc_notify_echo_optishare/am263px-cc/system_freertos_nortos/loadAll.js"
  \endcode 

Performing the above steps will load all the binaries. All that is required is to resume the execution.

## Building using CCS

1. \ref OPTIFLASH_OPTISHARE_CCS to see how to enable optishare in a system project in the CCS. Perform the same steps in for this project. 

2. Load `/ipc_notify_echo_optishare_am263px-cc_system_freertos_nortos/Debug/ipc_notify_echo_optishare_am263px-cc_system_freertos_nortos.out` first before loading any other binaries as this contains the shared code and data.  
3. When Optishare is enabled, CCS generates ${ProjName}_optishare.out binaries for each core. Load Optishare binaries for each core. These Optishare .out do not contains shared code and data. 
4. Run all the cores.  


# See Also

\ref DRIVERS_IPC_NOTIFY_PAGE

# Sample Output

Shown below is a sample output when the application is run.

\code 
[IPC NOTIFY ECHO] Message exchange started by main core !!!
[IPC NOTIFY ECHO] All echoed messages received by main core from 3 remote cores !!!
[IPC NOTIFY ECHO] Messages sent to each core = 1000000
[IPC NOTIFY ECHO] Number of remote cores = 3
All tests have passed!!
\endcode
