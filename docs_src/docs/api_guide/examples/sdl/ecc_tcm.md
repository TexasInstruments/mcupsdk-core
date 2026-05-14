# SDL R5F ECC TCM {#EXAMPLES_SDL_R5F_ECC_TCM}

[TOC]

# Introduction

This example demonstrates the usage of the SDL ECC on R5F TCM module. The example shows how to setup and use the ECC Safety Diagnostic operation.
Shows the generation of SEC and DED error on R5F TCM.

\note DM swaps ATCM and BTCM addresses. ATCM starts at 0x41010000 and BTCM starts at 0x0.

Use Cases
---------
 Use Case | Description
 ---------|------------
 UC-1     | Double bit error injection.
 UC-2     | Single bit error injection.


# Supported Combinations {#EXAMPLES_SDL_R5F_ECC_TCM_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc_tcm/


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

\code

ECC TCM Example Application

ECC_Example_init: Init MAIN ESM complete

ECC_Example_init: Init WKUP ESM complete

AGGR ECC Init complete

ECC Safety Example tests: starting

B0TCM0 Bank0 Double bit error self test: starting
B0TCM0 Bank0 Double bit error self test: pErrMem 0x00000000 fixed location once test complete
Waiting for ESM Interrupt

UC-1: Got High priority ESM Interrupt

B0TCM0 Bank0 Single bit error inject test: starting

B0TCM0 Bank0 Single bit error inject test: at pErrMem 0x00000000: fixed location once test complete
Waiting for ESM Interrupt

UC-2: Got Low priority ESM Interrupt

ECC Safety Example tests: success

ECC UC-1 and UC-2 Test

All tests have passed.

\endcode
