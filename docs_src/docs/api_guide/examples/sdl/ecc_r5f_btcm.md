# SDL R5F ECC BTCM {#EXAMPLES_SDL_R5F_ECC_BTCM}

[TOC]

# Introduction

This example demonstrates the usage of the SDL R5F BTCM module. The example shows how to setup and use the ECC Safety Diagnostic operation.
Shows the generation of SEC and DED error on R5F BTCM ECC Aggregator.

Use Cases
---------
\cond SOC_AM263X
 Use Case | Description
 ---------|------------
 UC-1     | Double bit error injection.
 UC-2     | Single bit error injection.
\endcond

\cond SOC_AM273X || SOC_AWR294X
 Use Case | Description
 ---------|------------
 UC-1     | Single bit error injection.
 UC-2     | Double bit error injection.
 
\endcond

# Supported Combinations {#EXAMPLES_SDL_R5F_ECC_BTCM_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_ecc_r5_btcm/


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

ECC_Test_init: Exception init complete 

ECC_Test_init: Initialize of R5FSS0 CORE0 ECC Memory is complete 

ESM_Test_init: Init MSS ESM complete 

ECC_Test_init: R5FSS0 CORE0 ECC initialization is completed 

R5FSS0 CORE0 BTCM Double bit error inject: starting 

ESM Call back function called : instType 0x2, intType 0x1, grpChannel 0x1, index 0x10, intSrc 0x30 

Take action 

High Priority Interrupt Executed

Read data of DED MSS_CTRL register is 0x0

Read data of DED RAW MSS_CTRL register is 0x4

R5FSS0 CORE0 BTCM Double bit error inject: pErrMem fixed location = 0x00080010 once test complete: the value of pErrMem is 0x00000000

Waiting for ESM Interrupt 

UC-1: Injected 2-bit error and got ESM Interrupt

R5FSS0 CORE0 BTCM Single bit error inject: starting 

ESM Call back function called : instType 0x2, intType 0x2, grpChannel 0x1, index 0xf, intSrc 0x2f 

Take action 

Low Priority Interrupt Executed

Read data of SEC MSS_CTRL register is  0x0

Read data of SEC RAW MSS_CTRL register is 0x0

R5FSS0 CORE0 BTCM Single bit error inject at pErrMem = 0x00080010 and the value of pErrMem is 0x00000000 :test complete

Waiting for ESM Interrupt 

UC-2: Injected 1-bit error and got ESM Interrupt

All tests have passed. 
All tests have passed. 

\endcode
\endcond

\cond (SOC_AM273X) || (SOC_AWR294X)
\code

ECC Example Application

ECC UC-1 and UC-2 Test 

ECC_Test_init: Exception init complete 

ECC_Test_init: Initialize of R5FSS0 CORE0 ECC Memory is complete 

ESM_Test_init: Init MSS ESM complete 

ECC_Test_init: R5FSS0 CORE0 ECC initialization is completed 

R5FSS0 CORE0 BTCM Single bit error inject: starting 

ESM Call back function called : instType 0x1, grpChannel 0x1, intSrc 0x2a 

Take action 

ECC Error Call back function called : eccMemType 49783896, errorSrc 0x400, ramId 270978723, bitErrorOffset 0x1026aa054fea7fca, bitErrorGroup 0

R5FSS0 CORE0 BTCM Single bit error inject at pErrMem = 0x00080010 and the value of pErrMem is 0x00080000 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt

R5FSS0 CORE0 BTCM Double bit error inject: starting 

R5FSS0 CORE0 BTCM Double bit error inject: pErrMem fixed location = 0x00080010 once test complete: the value of pErrMem is 0x00080000

Waiting for ESM Interrupt 

UC-2: Injected 2-bit error and ESM Interrupt not occured

All tests have passed. 

\endcode
\endcond
