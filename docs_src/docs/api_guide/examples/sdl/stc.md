# SDL STC Example {#EXAMPLES_SDL_STC}

[TOC]

# Introduction

This example demonstrates how to perform STC for a given core.


\cond SOC_AM273X
This example displays:

    * How to check if STC is Done for a Instance(CPU core).
    * How to Run STC for the Instance (CPU core).

\endcond


\cond SOC_AM273X || AWR294X
Use Cases
---------
* STC run for DSP core after that it will run for R5F(Self Core).

\endcond
\cond SOC_AM263X || SOC_AM263PX
Use Cases
---------
* STC run for R5F1 core after that it will run R5F0( SELF CORE).

\endcond

# Supported Combinations {#EXAMPLES_SDL_STC_COMBOS}


\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/stc/stc_mcu/

\endcond

\cond SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/stc/stc_mcu/

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss1-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/stc/stc_mcu/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_STC_PAGE

# Sample Output

\cond SOC_AM273X || SOC_AWR294X
Shown below is a sample output when the application is run for R5F,

\code

DSP STC Test Application started.
DSP STC Test should Pass & Completed.
If DSP STC test is successfull, DSP Core will be reset.
Wait for interrupt registration in DSP Core .
Interrupt registration in Done on DSP Core .
DSP Core is Reset.
DSP STC is done Successfully & Passed.
R5F STC Test Application started.
R5F STC Test should Pass & Completed.
If R5F STC test is successfull, Core will go in to Reset.
R5F Core is Reset.
R5F STC is done Successfully & Passed.
R5F STC Test Application started.
R5F STC Test should Fail & Completed.
If R5F STC test is successfull, Core will go in to Reset.
R5F Core is Reset.
R5F STC Test is Completed & failing.
Waiting in loop in STC_func_test_Main().

\endcode
\endcond

\cond SOC_AM263X
Shown below is a sample output when the application is run for R5F1 followed by R5F0,
code is running on  R5F0 core.
\code

STC Test Application started.
If STC test is successfull, Core1 will go in to Reset.
STC Test Application started.
If STC test is successfull, Core0 will go in to Reset.
Core1 is Reset.
STC is done Successfully & Passed for R5F1.
Core0 is Reset.
STC is done Successfully & Passed for R5F0.
Waiting in loop in STC_Main().

\endcode
\endcond

\cond SOC_AM263PX
\code

STC Test Application started.
If STC test is successfull, Core1 will go in to Reset.
Core1 is Reset.
STC is done Successfully & Passed for R5F1.
Waiting in loop in STC_Main().

\endcode
\endcond