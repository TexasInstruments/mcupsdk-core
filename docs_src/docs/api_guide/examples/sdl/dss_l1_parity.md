# SDL PARITY DSS L1 {#EXAMPLES_SDL_DSS_L1_PARITY}

[TOC]

# Introduction

This example demonstrates the usage of the SDL DSS L1 parity. The example shows how to setup and use the Parity Safety Diagnostic operation.
This shows the DSS L1 parity error injection.

# Supported Combinations {#EXAMPLES_SDL_DSS_L1_PARITY_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | c66ss0 nortos
 Toolchain      | ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/parity/sdl_dss_l1_parity/

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

Parity Example Application

Parity UC-1 Example

ESM_Test_init: Init DSS ESM complete

Parity Safety Example tests: starting

Initiliazed and Enabled IDMA1 Interrupt

Enable the Error Detect logic...

Waiting for IDMA1 transfer Interrupt

IDMA1 transfer is done and got interrupt !!

Suspend the Error Detect logic...

Toggle a single bit in the Dummy function

Waiting for IDMA1 transfer Interrupt

IDMA1 transfer is done and got interrupt !!

Enable the Error Detect logic...

ESM Call back function called : instType 0x2, grpChannel 0x1, intSrc 0x3 
 
Take action 

Low Priority Interrupt Executed

Waiting for IDMA1 transfer Interrupt

IDMA1 transfer is done and got interrupt !!

Waiting for ESM Interrupt

ESM Interrupt Received.

All tests have passed!!

All Use_Cases have passed. 
