# SDL ECC DMA {#EXAMPLES_SDL_DMA_PARITY}

[TOC]

# Introduction

This example demonstrates the usage of the SDL DMA parity. The example shows how to setup and use the Parity Safety Diagnostic operation.
This shows the DMA parity error injection.

# Supported Combinations {#EXAMPLES_SDL_DMA_PARITY_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/parity/sdl_dma_parity/

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
Parity Example Application

ESM_Test_init: Init MSS ESM complete 

TCM PARITY Example : Started

TCM PARITY : R5FSS0_0

TCM PARITY : ATCM0 Started

ESM Call back function called : instType 0x2, intType 0x1, grpChannel 0x0, index 0xe, intSrc 0xe 

Take action 

TCM Parity Status for ATCM0 = 0x1

R5FSS0_0 ATCM0 Parity : Completed

TCM PARITY : B0TCM0 Started

ESM Call back function called : instType 0x2, intType 0x1, grpChannel 0x0, index 0xe, intSrc 0xe 

Take action 

TCM Parity Status for B0TCM0 = 0x2

R5FSS0_0 B0TCM0 Parity : Completed

TCM PARITY : B1TCM0 Started

ESM Call back function called : instType 0x2, intType 0x1, grpChannel 0x0, index 0xe, intSrc 0xe 

Take action 

TCM Parity Status for B1TCM0 = 0x4

R5FSS0_0 B1TCM0 Parity : Completed

TCM PARITY:R5FSS0_1

MSS TCM PARITY:ATCM1 Started

ESM Call back function called : instType 0x2, intType 0x1, grpChannel 0x0, index 0xf, intSrc 0xf 

Take action 

TCM Parity Status for ATCM1 = 0x1

R5FSS0_1 ATCM1 Parity : Completed

TCM PARITY : B0TCM1 Started

ESM Call back function called : instType 0x2, intType 0x1, grpChannel 0x0, index 0xf, intSrc 0xf 

Take action 

TCM Parity Status for B0TCM1 = 0x2

R5FSS0_1 B0TCM1 Parity : Completed

TCM PARITY : B1TCM1 Started

ESM Call back function called : instType 0x2, intType 0x1, grpChannel 0x0, index 0xf, intSrc 0xf 

Take action 

TCM Parity Status for B1TCM1 = 0x4

R5FSS0_1 B1TCM1 Parity : Completed

TCM PARITY:R5FSS1_0

TCM PARITY : ATCM0 Started

ESM Call back function called : instType 0x2, intType 0x2, grpChannel 0x0, index 0x10, intSrc 0x10 

Take action 

TCM Parity Status for ATCM0 = 0x1

R5FSS1_0 ATCM0 Parity : Completed

TCM PARITY:B0TCM0 Started

ESM Call back function called : instType 0x2, intType 0x2, grpChannel 0x0, index 0x10, intSrc 0x10 

Take action 

TCM Parity Status for B0TCM0 = 0x2

R5FSS1_0 B0TCM0 Parity : Completed

TCM PARITY : B1TCM0 Started

ESM Call back function called : instType 0x2, intType 0x2, grpChannel 0x0, index 0x10, intSrc 0x10 

Take action 

TCM Parity Status for B1TCM0 = 0x4

R5FSS1_0 B1TCM0 Parity : Completed

TCM PARITY:R5FSS1_1

TCM PARITY:ATCM1 Started

ESM Call back function called : instType 0x2, intType 0x2, grpChannel 0x0, index 0x11, intSrc 0x11 

Take action 

TCM Parity Status for ATCM1 = 0x1

R5FSS1_1 ATCM1 Parity : Completed

TCM PARITY : B0TCM1 Started

ESM Call back function called : instType 0x2, intType 0x2, grpChannel 0x0, index 0x11, intSrc 0x11 

Take action 

TCM Parity Status for B0TCM1 = 0x2

R5FSS1_1 B0TCM1 Parity : Completed

TCM PARITY : B1TCM1 Started

ESM Call back function called : instType 0x2, intType 0x2, grpChannel 0x0, index 0x11, intSrc 0x11 

Take action 

TCM Parity Status for B1TCM1 = 0x4

R5FSS1_1 B1TCM1 Parity : Completed

All tests have passed.   

\endcode
\endcond

\cond (SOC_AM273X) || (SOC_AWR294X)
\code
Parity Example Application

TCM PARITY Example : Started

ESM_Test_init: Init MSS ESM complete 

MSS TCM PARITY: ATCM0 Started

ESM Call back function called : instType 0x1, grpChannel 0x2, intSrc 0x3 

Take action 

MSS ATCM0 Parity : Completed

ESM_Test_init: Init MSS ESM complete 

MSS TCM PARITY: ATCM1 Started

ESM Call back function called : instType 0x1, grpChannel 0x2, intSrc 0x6 

Take action 

MSS ATCM1 Parity : Completed 

ESM_Test_init: Init MSS ESM complete 

MSS TCM PARITY: B0TCM0 Started

ESM Call back function called : instType 0x1, grpChannel 0x2, intSrc 0x4 

Take action 

MSS B0TCM0 Parity : Completed 

ESM_Test_init: Init MSS ESM complete 

MSS TCM PARITY: B0TCM1 Started

ESM Call back function called : instType 0x1, grpChannel 0x2, intSrc 0x7 

Take action 

MSS B0TCM1 Parity : Completed 

ESM_Test_init: Init MSS ESM complete 

MSS TCM PARITY: B1TCM0 Started

ESM Call back function called : instType 0x1, grpChannel 0x2, intSrc 0x5 

Take action 

MSS B1TCM0 Parity : Completed 

ESM_Test_init: Init MSS ESM complete 

MSS TCM PARITY: B1TCM1 Started

ESM Call back function called : instType 0x1, grpChannel 0x2, intSrc 0x8 

Take action 

MSS B1TCM1 Parity : Completed 

All tests have passed.
\endcode
\endcond