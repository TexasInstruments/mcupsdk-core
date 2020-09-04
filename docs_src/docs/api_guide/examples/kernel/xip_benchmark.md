# XIP Benchmark {#EXAMPLES_KERNEL_DPL_XIP_BENCHMARK}

[TOC]

# Introduction

This example shows a application running in XIP (eXecute In Place) mode. This example can be used to check and evaluate the performance
of the SOC when running in XIP mode.

The example does the below
- SOC is booted to run applications for multiple CPUs
- The bootloader configures the flash at highest possible speed and highest throughput mode.
\cond SOC_AM64X || SOC_AM243X
- On @VAR_SOC_NAME, this means, the flash is in
  - Direct access (DAC) mode for read
  - Dual data rate (DDR) mode
  - OSPI and flash at 133Mhz clock speed
  - OSPI Phy enable
  - OSPI Phy pipeline enabled.
\endcond
- All the CPUs are configured to run FreeRTOS with below functions being executed.
  - FIR function, here, code, input data and coeff data is kept in flash
  - memcpy function, here, code, input data is kept in flash
- The linker command file of the CPUs is setup so that code and rodata is execute and referenced from flash.
- An MPU entry is setup at each CPU, to enable execute from flahs region with cache enabled.
- Each function is run multiple times and the worst case, best case and average case performance is printed on the
  UART terminal in units of cycles.
- There is a compile time flag `ENABLE_CPU_SYNC` to "sync" all the CPUs such that all execute and access the code and data from flash
  at the approximately the same time. This allows to simulate the worst case simultaneous access to the flash device in multi CPU environment.
  **NOTE**, by default, `ENABLE_CPU_SYNC` is `0`, i.e the CPUs are not synced.

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-1 freertos
 ^              | r5fss1-0 freertos
 ^              | r5fss1-1 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/kernel/dpl/xip_benchmark/

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-1 freertos
 ^              | r5fss1-0 freertos
 ^              | r5fss1-1 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/kernel/dpl/xip_benchmark/

\endcond

# Steps to Run the Example

\note This is a `system` or multi-core project, so refer to system project build instructions for CCS project or makefiles when building the example.

- **When using CCS projects to build**, import the system CCS project
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE). This will build all the dependant CPU projects as well
- **When using makefiles to build**, build the system makefile using
  make command (see \ref MAKEFILE_BUILD_PAGE). This will build all the dependant CPU makefiles as well.

- This program cannot be loaded and run from CCS since it expects some sections of code/data to reside in flash

- To flash this program use the sample config file located at below.
\cond SOC_AM64X
        examples/kernel/dpl/xip_benchmark/@VAR_BOARD_NAME_LOWER/system_freertos/sbl_ospi.cfg
\endcond

\cond SOC_AM243X

## AM243X-EVM
        examples/kernel/dpl/xip_benchmark/@VAR_BOARD_NAME_LOWER/system_freertos/sbl_ospi.cfg

## AM243X-LP
        examples/kernel/dpl/xip_benchmark/@VAR_LP_BOARD_NAME_LOWER/system_freertos/sbl_ospi.cfg

\endcond

- This config file has the filenames listed assuming application is built using makefile's and release profile.
  If you are using CCS projects or debug profile, edit this file to point to appropriate files.

- Now flash to EVM using the steps mention here, only use the config file mentioned above, \ref GETTING_STARTED_FLASH

- This is a multi-core example. Above steps will flash applications for all the CPUs.

# See Also

\ref BOOTFLOW_XIP

# Sample Output

Shown below is a screenshot, after R5F is halted and symbols loaded, on AM64x. You can see the code execution is at flash region (`0x60xxxxxx`).

\imageStyle{xip_benchmark.png,width:50%}
\image html xip_benchmark.png "Code execution in XIP"

Shown below is a sample output when the application is run on am64x-evm,

\code
Starting OSPI Bootloader ...

DMSC Firmware Version 21.1.1--v2021.01a (Terrific Lla
DMSC Firmware revision 0x15
DMSC ABI revision 3.1

INFO: Bootloader_loadSelfCpu:203: CPU r5f0-0 is initialized to 800000000 Hz !!!
INFO: Bootloader_loadSelfCpu:203: CPU r5f0-1 is initialized to 800000000 Hz !!!
INFO: Bootloader_runCpu:151: CPU m4f0-0 is initialized to 400000000 Hz !!!
INFO: Bootloader_runCpu:151: CPU r5f1-0  is initialized to 800000000 Hz !!!
INFO: Bootloader_runCpu:151: CPU r5f1-1 is initialized to 800000000 Hz !!!
INFO: Bootloader_runSelfCpu:215: All done, reseting self ...


### XIP benchmark ###
[r5f0-1]     0.000387s :
[r5f0-1]     0.000450s : ### XIP benchmark ###
[r5f0-1]     0.000871s : FIR:
[r5f0-1]     0.000880s :   26721 cycles (code/data fully cached) ,
[r5f0-1]     0.000895s :   59814 cycles (code/data not cached) ,
[r5f0-1]     0.000908s :   30033 cycles (code/data not-cached 1 of 10 iterations)
[r5f0-1]     0.000960s : MEMCPY:
[r5f0-1]     0.000966s :    1564 cycles (code/data fully cached) ,
[r5f0-1]     0.000979s :    5073 cycles (code/data not cached) ,
[r5f0-1]     0.000992s :    1919 cycles (code/data not-cached 1 of 10 iterations)
[r5f0-1]     0.001018s : All tests have passed!!
[r5f0-1]     0.001029s :
[r5f1-0]     0.000386s :
[r5f1-0]     0.000449s : ### XIP benchmark ###
[r5f1-0]     0.000871s : FIR:
[r5f1-0]     0.000878s :   26721 cycles (code/data fully cached) ,
[r5f1-0]     0.000893s :   60349 cycles (code/data not cached) ,
[r5f1-0]     0.000906s :   30086 cycles (code/data not-cached 1 of 10 iterations)
[r5f1-0]     0.000958s : MEMCPY:
[r5f1-0]     0.000966s :    1564 cycles (code/data fully cached) ,
[r5f1-0]     0.000979s :    5002 cycles (code/data not cached) ,
[r5f1-0]     0.000991s :    1911 cycles (code/data not-cached 1 of 10 iterations)
[r5f1-0]     0.001017s : All tests have passed!!
[r5f1-0]     0.001027s :
[r5f1-1]     0.000384s :
[r5f1-1]     0.000446s : ### XIP benchmark ###
[r5f1-1]     0.000869s : FIR:
[r5f1-1]     0.000876s :   26721 cycles (code/data fully cached) ,
[r5f1-1]     0.000891s :   60298 cycles (code/data not cached) ,
[r5f1-1]     0.000904s :   30081 cycles (code/data not-cached 1 of 10 iterations)
[r5f1-1]     0.000956s : MEMCPY:
[r5f1-1]     0.000963s :    1564 cycles (code/data fully cached) ,
[r5f1-1]     0.000976s :    5057 cycles (code/data not cached) ,
[r5f1-1]     0.000988s :    1916 cycles (code/data not-cached 1 of 10 iterations)
[r5f1-1]     0.001014s : All tests have passed!!
[r5f1-1]     0.001024s :
FIR:
  26721 cycles (code/data fully cached) ,
  58646 cycles (code/data not cached) ,
  30119 cycles (code/data not-cached 1 of 10 iterations)
MEMCPY:
   1564 cycles (code/data fully cached) ,
   4179 cycles (code/data not cached) ,
   1830 cycles (code/data not-cached 1 of 10 iterations)
All tests have passed!!
\endcode

