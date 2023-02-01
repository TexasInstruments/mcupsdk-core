# SDL DSS ECC TPTC {#EXAMPLES_SDL_DSS_TPTC_ECC}

[TOC]

# Introduction

This example demonstrates SEC and DED on DSS TPTC FIFO memories.

Use Cases
---------

 Use Case | Description
 ---------|------------
 UC-1     | Single bit error injection on DSS TPTC.
 UC-2     | Double bit error injection on DSS TPTC.

# Supported Combinations {#EXAMPLES_SDL_DSS_TPTC_ECC_COMBOS}

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | c66ss0 nortos
 Toolchain      | ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_ecc_dss_tptc/

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

\cond (SOC_AM273X) || (SOC_AWR294X)
\code

ECC Example Application

ECC UC-1 and UC-2 Test

[EDMA] Interrupt Transfer Test Started...

ESM_Test_init: Init DSS ESM single bit complete

ESM_Test_init: Init DSS ESM double bit complete

ECC_Test_init: DSS ECC AGGR initialization is completed 

DSS TPTC_A0 Single bit error inject: test starting

ESM Call back function called : instType 0x2, grpChannel 0x1, intSrc 0x5c 

Take action 

ECC Error Call back function called : eccMemType 3, errorSrc 0x1, ramId 9, bitErrorOffset 0x00000001, bitErrorGroup 0

DSS TPTC_A0 Single bit error inject at pErrMem 0x00000000

[EDMA] Interrupt Transfer Test Completed!!

All tests have passed!!

All Use_Cases have passed. 

\endcode
\endcond
