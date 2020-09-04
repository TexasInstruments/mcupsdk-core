# SDL ECC MCAN {#EXAMPLES_SDL_ECC_MCAN}

[TOC]

# Introduction

This example demonstrates the usage of the SDL MCAN module. The example shows how to setup and use the ECC Safety Diagnostic operation.
Shows the generation of SEC and DED error on MCAN ECC Aggregator.

Use Cases
---------

 Use Case | Description
 ---------|------------
 UC-1     | Single bit error injection.
 UC-2     | Double bit error injection.

# Supported Combinations {#EXAMPLES_SDL_ECC_MCAN_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_ecc_mcan/

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

ECC_Test_init: Initialize of MCANA ECC Memory is complete 

ESM_Test_init: Init MCANA ESM complete 

ECC_Test_init: MCANA ECC initialization is completed 

MCANA Single bit error inject: starting 

ESM Call back function called : instType 0x2, intType 0x2, grpChannel 0x0, index 0x2, intSrc 0x2

Take action

Low Priority Interrupt Executed

ECC Error Call back function called : eccMemType 7, errorSrc 0x1, ramId 0, bitErrorOffset 0x00000001, bitErrorGroup 0

MCANA Single bit error inject at pErrMem = 0x52600000 and the value of pErrMem is 0x00000000 :test complete

Read data =  0x01010101

Read data =  0x02020202

Read data =  0x03030303

Read data =  0x04040404

Read data =  0x05050505

Read data =  0x06060606

Read data =  0x07070707

Read data =  0x08080808

Read data =  0x09090909

Read data =  0x0A0A0A0A

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt 

MCANA double bit error inject: starting 

ESM Call back function called : instType 0x2, intType 0x1, grpChannel 0x0, index 0x3, intSrc 0x3

Take action

High Priority Interrupt Executed

ECC Error Call back function called : eccMemType 7, errorSrc 0x2, ramId 0, bitErrorOffset 0x00000000, bitErrorGroup 0

MCANA Double bit error inject: pErrMem fixed location = 0x52600000 once test complete: the value of pErrMem is 0x00000007

Read data =  0x01010101

Read data =  0x02020202

Read data =  0x03030303

Read data =  0x04040404

Read data =  0x05050505

Read data =  0x06060606

Read data =  0x07070707

Read data =  0x08080808

Read data =  0x09090909

Read data =  0x0A0A0A0A

Waiting for ESM Interrupt

UC-2: Injected 2-bit error and got ESM Interrupt

All tests have passed. 

\endcode
\endcond
\cond (SOC_AM273X) || (SOC_AWR294X)
\code

ECC Example Application

ECC UC-1 and UC-2 Test 

ECC_Test_init: Initialize of MCANA ECC Memory is complete 

ECC_Test_init: MCANA ECC initialization is completed 

ESM_Test_init: Init MCANA ESM single bit complete

MCANA Single bit error inject: starting 

ESM Call back function called : instType 0x1, grpChannel 0x1, intSrc 0x33 
  
Take action 

ECC Error Call back function called : eccMemType 4, errorSrc 0x1, ramId 0, bitErrorOffset 0x00000001, bitErrorGroup 0

MCANA Single bit error inject at pErrMem = 0x02040000 and the value of pErrMem is 0x00000000 :test complete

Read data =  0x01010101

Read data =  0x02020202

Read data =  0x03030303

Read data =  0x04040404

Read data =  0x05050505

Read data =  0x06060606

Read data =  0x07070707

Read data =  0x08080808

Read data =  0x09090909

Read data =  0x0A0A0A0A

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt 

ESM_Test_init: Init MCANA ESM double bit complete 

MCANA double bit error inject: starting 

ESM Call back function called : instType 0x1, grpChannel 0x1, intSrc 0x32 
  
Take action 

ECC Error Call back function called : eccMemType 4, errorSrc 0x2, ramId 0, bitErrorOffset 0x00000000, bitErrorGroup 0

MCANA Double bit error inject: pErrMem fixed location = 0x02040000 once test complete: the value of pErrMem is 0x00000007

Read data =  0x01010101

Read data =  0x02020202

Read data =  0x03030303

Read data =  0x04040404

Read data =  0x05050505

Read data =  0x06060606

Read data =  0x07070707

Read data =  0x08080808

Read data =  0x09090909

Read data =  0x0A0A0A0A

Waiting for ESM Interrupt

UC-2: Injected 2-bit error and got ESM Interrupt

All tests have passed. 

\endcode
\endcond
