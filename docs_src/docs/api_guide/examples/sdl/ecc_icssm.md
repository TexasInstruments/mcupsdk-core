# SDL ECC ICSSM {#EXAMPLES_SDL_ECC_ICSSM}

[TOC]

# Introduction

This example demonstrates the usage of the SDL ICSSM module. The example shows how to setup and use the ECC Safety Diagnostic operation.
Shows the generation of SEC and DED error on ICSSM ECC Aggregator.

Use Cases
---------

 Use Case | Description
 ---------|------------
 UC-1     | Double bit error injection.
 UC-2     | Single bit error injection.

# Supported Combinations {#EXAMPLES_SDL_ECC_ICSSM_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_ecc_icssm/

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

\cond (SOC_AM263X || SOC_AM263PX)
\code
ECC Example Application

ECC UC-1 and UC-2 Test

ECC_Test_init: Initialize of ICSSM ECC Memory is complete

ESM_Test_init: Init MSS ESM complete

ECC_Test_init: ICSSM ECC initialization is completed

ICSSM Double bit error inject: starting

ESM Call back function called : instType 0x2, intType 0x1, grpChannel 0x2, index 0xd, intSrc 0x4d

Take action

High Priority Interrupt Executed

ECC Error Call back function called : eccMemType 6, errorSrc 0x2, ramId 0, bitErrorOffset 0x00000000, bitErrorGroup 0

ICSSM Double bit error inject: pErrMem fixed location = 0x48000000 once test complete: the value of pErrMem is 0x00000007

Waiting for ESM Interrupt

UC-1: Injected 2-bit error and got ESM Interrupt

ICSSM Single bit error inject: starting

ESM Call back function called : instType 0x2, intType 0x2, grpChannel 0x2, index 0xe, intSrc 0x4e

Take action

Low Priority Interrupt Executed

ECC Error Call back function called : eccMemType 6, errorSrc 0x1, ramId 0, bitErrorOffset 0x00000001, bitErrorGroup 0

ICSSM Single bit error inject at pErrMem = 0x48000000 and the value of pErrMem is 0x00000000 :test complete

Waiting for ESM Interrupt

UC-2: Injected 1-bit error and got ESM Interrupt

All tests have passed.

\endcode
\endcond
