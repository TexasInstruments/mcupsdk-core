# SDL DSS DSP EDC {#EXAMPLES_SDL_DSS_DSP_EDC}

[TOC]

# Introduction

This example demonstrates the usage of the SDL DSS DSP EDC errors. The example shows how to setup and use the Parity Safety Diagnostic operation.
This shows the DSS DSP EDC error injection.

# Supported Combinations {#EXAMPLES_SDL_DSS_DSP_EDC_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | c66ss0 nortos
 Toolchain      | ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_dss_dsp_edc/

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

Initiliazed and Enabled ECC DED Interrupt

Initiliazed and Enabled ECC SEC Interrupt

Enabling and Propagating the memory

Waiting for DSS Interrupt

Disabled the MASK and FLG registers

UC-1: DSP EDC Error tested and got all Interrupts 

All Use_Cases have passed. 
