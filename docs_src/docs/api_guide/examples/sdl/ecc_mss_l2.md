# SDL ECC MSS L2 {#EXAMPLES_SDL_ECC_MSS_L2}

[TOC]

# Introduction

This example demonstrates the usage of the SDL MSS L2 module. The example shows how to setup and use the ECC Safety Diagnostic operation.
Shows the generation of SEC and DED error on MSS L2 ECC Aggregator.

Use Cases
---------

 Use Case | Description
 ---------|------------
 UC-1     | Single bit error injection.
 UC-2     | Double bit error injection.

# Supported Combinations {#EXAMPLES_SDL_ECC_MSS_L2_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_ecc_mss_l2/



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
\cond (SOC_AM263X)
\code

ECC Example Application

ECC UC-1 and UC-2 Test 

ECC_Test_init: Initialize of MSS L2 ECC Memory is complete 

ESM_Test_init: Init MSS ESM complete 

ECC_Test_init: MSS L2 ECC initialization is completed 

MSS L2 RAMB Single bit error inject: starting 

ESM Call back function called : instType 0x2, intType 0x1, grpChannel 0x0, index 0x13, intSrc 0x13
 
Take action 

High Priority Interrupt Executed

ECC Error Call back function called : eccMemType 0, errorSrc 0x1, ramId 2, bitErrorOffset 0x00000003, bitErrorGroup 0

MSS L2 RAMB Single bit error inject at pErrMem = 0x70100008 and the value of pErrMem is 0x00000000 :test complete

Waiting for ESM Interrupt

UC-1: Injected 1-bit error and got ESM Interrupt 

MSS L2 RAMB Double bit error inject: starting 

ESM Call back function called : instType 0x2, intType 0x1, grpChannel 0x0, index 0x14, intSrc 0x14
 
Take action 

High Priority Interrupt Executed

ECC Error Call back function called : eccMemType 0, errorSrc 0x2, ramId 2, bitErrorOffset 0x00000002, bitErrorGroup 0

MSS L2 RAMB Double bit error inject: pErrMem fixed location = 0x70100008 once test complete: the value of pErrMem is 0x00000000

Waiting for ESM Interrupt 

UC-2: Injected 2-bit error and got ESM Interrupt

All tests have passed.

\endcode
\endcond

\cond (SOC_AM273X) || (SOC_AWR294X)
\code

ECC Example Application

ECC UC-1 and UC-2 Test 

ECC_Test_init: Initialize of MSS L2 ECC Memory is complete 

ECC_Test_init: MSS L2 ECC initialization is completed 

ESM_Test_init: Init MSS L2 RAMB single bit ESM complete 

MSS L2 RAMB Single bit error inject: starting 

ESM Call back function called : instType 0x1, grpChannel 0x1, intSrc 0x12 

Take action 

ECC Error Call back function called : eccMemType 2, errorSrc 0x1, ramId 1, bitErrorOffset 0x00000001, bitErrorGroup 0

MSS L2 RAMB Single bit error inject at pErrMem = 0x10280000 and the value of pErrMem is 0x00000000 :test complete

Waiting for ESM Interrupt

UC-1: Injected 1-bit error and got ESM Interrupt 

ESM_Test_init: Init MSS L2 RAMB single bit ESM complete 

MSS L2 RAMB Double bit error inject: starting 

ESM Call back function called : instType 0x1, grpChannel 0x1, intSrc 0x11 

Take action 

ECC Error Call back function called : eccMemType 2, errorSrc 0x2, ramId 1, bitErrorOffset 0x00000000, bitErrorGroup 0

MSS L2 RAMB Double bit error inject: pErrMem fixed location = 0x10280000 once test complete: the value of pErrMem is 0x00012002

Waiting for ESM Interrupt 

UC-2: Injected 2-bit error and got ESM Interrupt

All tests have passed. 

\endcode
\endcond
