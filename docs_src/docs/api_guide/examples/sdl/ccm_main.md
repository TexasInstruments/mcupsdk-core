# SDL CCM {#EXAMPLES_SDL_CCM}

[TOC]

# Introduction

This example demonstrates the usage of the SDL R5 CCM module. The example shows how to setup and use the R5 CCM Safety Diagnostic operation.

Use Cases
---------

 Use Case | Description
 ---------|------------
 UC-1     | self test on R5 CCM.
 UC-2     | self test with error force on R5 CCM.


# Supported Combinations {#EXAMPLES_SDL_CCM_COMBOS}

\cond SOC_AM263X || SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ccm/sdl_ccm_example/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_CCM_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\cond (SOC_AM263X) || (SOC_AM273X) || (SOC_AWR294X)
\code
R5F0 CORE:

R5 CPU Application

R5F-CPU example application CCM_Test_init: Init MCU ESM complete

CCM_Test_init: CCM Init complete

R5F-CPU example application CCM_Test_init: CCM Init complete

CCM Functional Test

R5F-CPU example application CCM self test: starting

R5F-CPU example application CCM Self Test complete

R5F-CPU example application CCM self test with error forcing: starting

R5F-CPU example application CCM Self Test with error forcing complete

R5F-CPU example application CCM inject  error: test starting

R5F-CPU example application CCM inject Test complete

CPU Functionality Passed.

All tests have passed.

\endcode
\endcond

\cond (SOC_AM263X)
\code
R5F1 CORE:

R5 CPU Application

R5F-CPU example application CCM_Test_init: Init MCU ESM complete

CCM_Test_init: CCM Init complete

R5F-CPU example application CCM_Test_init: CCM Init complete

CCM Functional Test

R5F-CPU example application CCM self test: starting

R5F-CPU example application CCM Self Test complete

R5F-CPU example application CCM self test with error forcing: starting

R5F-CPU example application CCM Self Test with error forcing complete

R5F-CPU example application CCM inject  error: test starting

R5F-CPU example application CCM inject Test complete

CPU Functionality Passed.

All tests have passed.
\endcode
\endcond