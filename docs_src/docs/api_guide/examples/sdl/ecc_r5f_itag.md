# SDL R5F ECC ITAG {#EXAMPLES_SDL_R5F_ECC_ITAG}

[TOC]

# Introduction

The example shows how to setup and use the ECC Safety Diagnostic operation on i-tag cache momory of R5F core.
Shows the generation of SEC error on R5F ECC Aggregator for ITAG cache moemories.
Use Cases
---------
\cond SOC_AM263X 
 Use Case | Description
 ---------|------------
 UC-1     | Single bit error injection.
\endcond


# Supported Combinations {#EXAMPLES_SDL_R5F_ECC_ITAG_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_ecc_r5_i-tag/


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

ECC UC-1 Test 

ECC_Test_init: Exception init complete 

ESM_Test_init: Init MSS ESM complete 

ECC_Test_init: R5FSS0 CORE0 ECC initialization is completed 

R5FSS0 CORE0 i_tag Single bit error inject: starting 

R5FSS0 CORE0 i_tag Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_ID = 0.

R5FSS0 CORE0 i_tag Single bit error inject: starting 

R5FSS0 CORE0 i_tag Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_ID = 1.

R5FSS0 CORE0 i_tag Single bit error inject: starting 

R5FSS0 CORE0 i_tag Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 2

R5FSS0 CORE0 i_tag Single bit error inject: starting 

R5FSS0 CORE0 i_tag Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 3

All tests have passed.  

\endcode
\endcond

