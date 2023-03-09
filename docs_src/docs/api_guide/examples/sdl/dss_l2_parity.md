# SDL PARITY DSS L2 {#EXAMPLES_SDL_DSS_L2_PARITY}

[TOC]

# Introduction

This example demonstrates the usage of the SDL DSS L2 parity. The example shows how to setup and use the Parity Safety Diagnostic operation.
This shows the DSS L2 parity error injection.

# Supported Combinations {#EXAMPLES_SDL_DSS_L2_PARITY_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | c66ss0 nortos
 Toolchain      | ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/parity/sdl_dss_l2_parity/

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

DSS L2 parity Safety Example tests: starting

ESM_Test_init: Init DSS ESM complete

DSS L2 parity is enabled

DSS L2 parity initialization is completed

DSS L2 parity error injected

Waiting for ESM Interrupt 

cleared DSS_CTRL.DSS_DSP_L2RAM_PARITY_CTRL.DSS_DSP_L2RAM_PARITY_CTRL_ENABLE

UC-1: Parity error is injected 1-bit error and got ESM Interrupt 

All Use_Cases have passed.
