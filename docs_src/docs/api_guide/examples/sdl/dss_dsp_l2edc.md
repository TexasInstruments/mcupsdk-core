# SDL DSS DSP L2 EDC {#EXAMPLES_SDL_DSS_DSP_L2_EDC}

[TOC]

# Introduction

This example demonstrates the usage of the SDL DSS DSP L2 EDC errors. The example shows how to setup and use the EDC SEC/DED Safety Diagnostic operation.
This shows the DSS DSP EDC error injection.

# Supported Combinations {#EXAMPLES_SDL_DSS_DSP_L2_EDC_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | c66ss0 nortos
 Toolchain      | ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_dss_l2_edc/

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_ECC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

EDC Example Application

EDC UC-1 Example

ESM_Test_init: Init DSS ESM complete

EDC Safety Example tests: starting

Initiliazed and Enabled IDMA1 Interrupt

Enable the Error Detect logic...

IDMA1 call back function called. 

Waiting for IDMA1 transfer Interrupt

IDMA1 transfer is done and got interrupt !!

Suspend the Error Detect logic...

Toggle a single bit in the Dummy function

Enable the Error Detect logic...

Call dummy function

ESM Call back function called : instType 0x2, grpChannel 0x1, intSrc 0x4 
 
Take action 

Low Priority Interrupt Executed

Waiting for ESM Interrupt

ESM Interrupt has occurred!!

SEC test has passed!!

EDC Safety Example tests: starting

Initiliazed and Enabled IDMA1 Interrupt

Enable the Error Detect logic...

IDMA1 call back function called. 

Waiting for IDMA1 transfer Interrupt

IDMA1 transfer is done and got interrupt !!

Suspend the Error Detect logic...

Toggle double bit in the Dummy function

Enable the Error Detect logic...

Call dummy function

ESM Call back function called : instType 0x2, grpChannel 0x1, intSrc 0x6 
 
Take action 

Low Priority Interrupt Executed

Waiting for ESM Interrupt

ESM Interrupt has occurred!!

DED test has passed!!

All Use_Cases have passed. 
