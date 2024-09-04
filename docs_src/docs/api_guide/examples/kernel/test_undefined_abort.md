# Undefined Abort Test {#EXAMPLES_KERNEL_UNDEFINED_ABORT}

[TOC]

# Introduction
This example shows the how custom data abort handler be implemented in the user application.

Provides information about the undef exception
1. `address`: Instruction causing the exception
2. `spsr`: Saved Program Status Register

\note Please refer to R5F TRM for more information

# Supported Combinations

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/kernel/dpl/test_undef/

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

- This config file has the filenames listed assuming application is built using makefile's and release profile.
  If you are using CCS projects or debug profile, edit this file to point to appropriate files.

- Now flash to EVM using the steps mention here, only use the config file mentioned above, \ref GETTING_STARTED_FLASH

- This is a multi-core example. Above steps will flash applications for all the CPUs.


# Sample Output

\note this application do not gives any output. This goes inside the `HwiP_user_undefined_handler_c` function and loops forever there. User needs to pause the applcaition and see the values.
