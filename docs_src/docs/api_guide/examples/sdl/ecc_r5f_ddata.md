# SDL R5F ECC DDATA {#EXAMPLES_SDL_R5F_ECC_DDATA}

[TOC]

# Introduction

The example shows how to setup and use the ECC Safety Diagnostic operation on d-data cache momory of R5F core.
Shows the generation of SEC error on R5F ECC Aggregator for DDATA cache moemories.
Use Cases
---------
\cond (SOC_AM263X || SOC_AM263PX)
 Use Case | Description
 ---------|------------
 UC-1     | Single bit error injection.
\endcond


# Supported Combinations {#EXAMPLES_SDL_R5F_ECC_DDATA_COMBOS}

\cond (SOC_AM263X || SOC_AM263PX)
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss1-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_ecc_r5_d-data/
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

\cond (SOC_AM263X)
\code

ECC Example Application

ECC UC-1 Test 

ECC_Test_init: Exception init complete 

ESM_Test_init: Init MSS ESM complete 

ECC_Test_init: R5FSS0 CORE0 ECC initialization is completed 

R5FSS0 CORE0 D_DATA Single bit error inject: starting 

R5FSS0 CORE0 D_DATA Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 13

R5FSS0 CORE0 D_DATA Single bit error inject: starting 

R5FSS0 CORE0 D_DATA Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 14

R5FSS0 CORE0 D_DATA Single bit error inject: starting 

R5FSS0 CORE0 D_DATA Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 15

R5FSS0 CORE0 D_DATA Single bit error inject: starting 

R5FSS0 CORE0 D_DATA Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 16

R5FSS0 CORE0 D_DATA Single bit error inject: starting 

R5FSS0 CORE0 D_DATA Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 17

R5FSS0 CORE0 D_DATA Single bit error inject: starting 

R5FSS0 CORE0 D_DATA Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 18

R5FSS0 CORE0 D_DATA Single bit error inject: starting 

R5FSS0 CORE0 D_DATA Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 19

R5FSS0 CORE0 D_DATA Single bit error inject: starting 

R5FSS0 CORE0 D_DATA Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 20

All tests have passed. 

\endcode
\endcond

\cond (SOC_AM263PX)
\code

ECC Example Application

ECC UC-1 Test

ECC_Test_init: Exception init complete 

ESM_Test_init: Init MSS ESM complete 

ECC_Test_init: R5FSS0 CORE0 ECC initialization is completed 

R5FSS0 CORE0 D_DATA Single bit error inject: starting 

R5FSS0 CORE0 D_DATA Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 13

R5FSS0 CORE0 D_DATA Single bit error inject: starting 

R5FSS0 CORE0 D_DATA Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 14

R5FSS0 CORE0 D_DATA Single bit error inject: starting 

R5FSS0 CORE0 D_DATA Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 15

R5FSS0 CORE0 D_DATA Single bit error inject: starting 

R5FSS0 CORE0 D_DATA Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 16

R5FSS0 CORE0 D_DATA Single bit error inject: starting 

R5FSS0 CORE0 D_DATA Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 17

R5FSS0 CORE0 D_DATA Single bit error inject: starting 

R5FSS0 CORE0 D_DATA Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 18

R5FSS0 CORE0 D_DATA Single bit error inject: starting 

R5FSS0 CORE0 D_DATA Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 19

R5FSS0 CORE0 D_DATA Single bit error inject: starting 

R5FSS0 CORE0 D_DATA Single bit error inject at pErrMem = 0x00000000 and the value of pErrMem is 0xE59FF018 :test complete

Waiting for ESM Interrupt 

UC-1: Injected 1-bit error and got ESM Interrupt for ram_Id = 20

All tests have passed.  

\endcode
\endcond
