# SDL ECC MSS TPTC {#EXAMPLES_SDL_ECC_MSS_TPTC}

[TOC]

# Introduction

This example demonstrates SEC and DED on MSS TPTC FIFO memories.

Use Cases
---------

 Use Case | Description
 ---------|------------
 UC-1     | Single bit error injection.
 UC-2     | Double bit error injection.

# Supported Combinations {#EXAMPLES_SDL_ECC_MSS_TPTC_COMBOS}

\cond (SOC_AM263X || SOC_AM263PX)
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss1-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_ecc_mss_tptc/
\endcond

\cond (SOC_AM273X) || (SOC_AWR294X)
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_ecc_mss_tptc/
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

[EDMA] Interrupt Transfer Test Started...

ESM_Test_init: Init MSS ESM single bit complete

ESM_Test_init: Init MSS ESM double bit complete

ECC_Test_init: MSS ECC AGGR initialization is completed

MSS TPTC_A0 Single bit error inject: test starting

ESM Call back function called : instType 0x2, intType 0x1, grpChannel 0x0, index 0x13, intSrc 0x13

Take action

High Priority Interrupt Executed

ECC Error Call back function called : eccMemType 0, errorSrc 0x1, ramId 6, bitErrorOffset 0x00000001, bitErrorGroup 0

MSS TPTC_A0 Single bit error inject at pErrMem 0x00000000

[EDMA] Interrupt Transfer Test Completed!!

All tests have passed!!

All tests have passed.


\endcode
\endcond

\cond (SOC_AM273X) || (SOC_AWR294X)
\code
ECC Example Application

ECC UC-1 and UC-2 Test

[EDMA] Interrupt Transfer Test Started...

ESM_Test_init: Init MSS ESM single bit complete

ESM_Test_init: Init MSS ESM double bit complete

ECC_Test_init: MSS ECC AGGR initialization is completed

MSS TPTC_A0 Single bit error inject: test starting

ESM Call back function called : instType 0x1, grpChannel 0x1, intSrc 0x12

Take action

ECC Error Call back function called : eccMemType 2, errorSrc 0x1, ramId 5, bitErrorOffset 0x00000001, bitErrorGroup 0

MSS TPTC_A0 Single bit error inject at pErrMem 0x00000000

[EDMA] Interrupt Transfer Test Completed!!

All tests have passed!!

All tests have passed.


\endcode
\endcond
