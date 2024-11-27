# SDL R5F ECC ATCM0 {#EXAMPLES_SDL_R5F_ECC_ATCM0}

[TOC]

# Introduction

This example demonstrates the usage of the SDL R5F ATCM0 module. The example shows how to setup and use the ECC Safety Diagnostic operation.
Shows the generation of SEC and DED error on R5F ATCM0 ECC Aggregator.

Use Cases
---------
\cond SOC_AM263X || SOC_AM263PX
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

# Supported Combinations {#EXAMPLES_SDL_R5F_ECC_ATCM0_COMBOS}

\cond (SOC_AM263X || SOC_AM263PX)
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss1-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_ecc_r5_atcm0/
\endcond

\cond (SOC_AM273X) || (SOC_AWR294X)
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_ecc_r5_atcm0/
\endcond

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

ECC_Test_init: Exception init complete

ECC_Test_init: Initialize of R5FSS0 CORE0 ECC Memory is complete

ESM_Test_init: Init MSS ESM complete

ECC_Test_init: R5FSS0 CORE0 ECC initialization is completed

R5FSS0 CORE0 ATCM0 BANK0 Double bit error inject: starting

ESM Call back function called : instType 0x2, intType 0x1, grpChannel 0x1, index 0x10, intSrc 0x30

Take action

High Priority Interrupt Executed

Read data of DED MSS_CTRL register is 0x0

Read data of DED RAW MSS_CTRL register is 0x0

R5FSS0 CORE0 ATCM0 BANK0 Double bit error inject: pErrMem fixed location = 0x00000510 once test complete: the value of pErrMem is 0x70056921

Waiting for ESM Interrupt

UC-1: Injected 2-bit error and got ESM Interrupt

R5FSS0 CORE0 ATCM0 BANK0 Single bit error inject: starting

ESM Call back function called : instType 0x2, intType 0x2, grpChannel 0x1, index 0xf, intSrc 0x2f

Take action

Low Priority Interrupt Executed

Read data of SEC MSS_CTRL register is  0x0

Read data of SEC RAW MSS_CTRL register is 0x0

R5FSS0 CORE0 ATCM0 BANK0 Single bit error inject at pErrMem = 0x00000510 and the value of pErrMem is 0x00000000 :test complete

Waiting for ESM Interrupt

UC-2: Injected 1-bit error and got ESM Interrupt

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

R5FSS0 CORE0 ATCM0 BANK0 Single bit error inject: starting

ESM Call back function called : instType 0x1, grpChannel 0x1, intSrc 0x2c

Take action

ECC Error Call back function called : eccMemType 49783896, errorSrc 0x1000, ramId 271037283, bitErrorOffset 0x10279005deadbabe, bitErrorGroup 0

R5FSS0 CORE0 ATCM0 BANK0 Single bit error inject at pErrMem = 0x00000510 and the value of pErrMem is 0x00000000 :test complete

Waiting for ESM Interrupt

UC-1: Injected 1-bit error and got ESM Interrupt

R5FSS0 CORE0 ATCM0 BANK0 Double bit error inject: starting

R5FSS0 CORE0 ATCM0 BANK0 Double bit error inject: pErrMem fixed location = 0x00000510 once test complete: the value of pErrMem is 0x00000000

Waiting for ESM Interrupt

UC-2: Injected 2-bit error and ESM Interrupt not occured

All tests have passed.

\endcode
\endcond
