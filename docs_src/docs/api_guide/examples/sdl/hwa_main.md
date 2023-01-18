# SDL HWA {#EXAMPLES_SDL_HWA_MAIN}

[TOC]

# Introduction

This example demonstrates the usage of the SDL HWA module.

The example shows how to setup and use the Parity and FSM Lockstep Diagnostic operation. Shows the generation of parity and FSM Lockstep errors on HWA data memory

Use Cases
---------

 Use Case | Description
 ---------|------------
  UC-1     | Parity insertion on HWA DMA0 DMEM0.
 UC-2     | Parity insertion on HWA DMA0 DMEM1.
 UC-3     | Parity insertion on HWA DMA0 DMEM2.
 UC-4     | Parity insertion on HWA DMA0 DMEM3.
 UC-5     | Parity insertion on HWA DMA0 DMEM4.
 UC-6     | Parity insertion on HWA DMA0 DMEM5.
 UC-7     | Parity insertion on HWA DMA0 DMEM6.
 UC-8     | Parity insertion on HWA DMA0 DMEM7.
 UC-9     | Parity insertion on HWA DMA0 WindowRam.
 UC-10    | Parity insertion on HWA DMA1 DMEM0.
 UC-11    | Parity insertion on HWA DMA1 DMEM1.
 UC-12    | Parity insertion on HWA DMA1 DMEM2.
 UC-13    | Parity insertion on HWA DMA1 DMEM3.
 UC-14    | Parity insertion on HWA DMA1 DMEM4.
 UC-15    | Parity insertion on HWA DMA1 DMEM5.
 UC-16    | Parity insertion on HWA DMA1 DMEM6.
 UC-17    | Parity insertion on HWA DMA1 DMEM7.
 UC-18    | Parity insertion on HWA DMA1 WindowRam.
 UC-19    | FSM locstep insertion on HWA.

# Supported Combinations {#EXAMPLES_SDL_HWA_MAIN_COMBOS}

\cond SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | c66  nortos
 Toolchain      | ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/hwa/hwa_main/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_HWA_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[C66xx_DSP]
HWA Application

HWA TEST START : starting

ESM Initialization for all the SDL HWA Nodes is Done

Applications Name: HWA_ParityDMA0DMEM0Test  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: HWA_ParityDMA0DMEM1Test  PASSED  and Time taken for the Test is 16  micro secs

Applications Name: HWA_ParityDMA0DMEM2Test  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: HWA_ParityDMA0DMEM3Test  PASSED  and Time taken for the Test is 16  micro secs

Applications Name: HWA_ParityDMA0DMEM4Test  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: HWA_ParityDMA0DMEM5Test  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: HWA_ParityDMA0DMEM6Test  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: HWA_ParityDMA0DMEM7Test  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: HWA_ParityDMA0WindowRamTest  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: HWA_ParityDMA1DMEM0Test  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: HWA_ParityDMA1DMEM1Test  PASSED  and Time taken for the Test is 16  micro secs

Applications Name: HWA_ParityDMA1DMEM2Test  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: HWA_ParityDMA1DMEM3Test  PASSED  and Time taken for the Test is 16  micro secs

Applications Name: HWA_ParityDMA1DMEM4Test  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: HWA_ParityDMA1DMEM5Test  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: HWA_ParityDMA1DMEM6Test  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: HWA_ParityDMA1DMEM7Test  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: HWA_ParityDMA1WindowRamTest  PASSED  and Time taken for the Test is 17  micro secs

Applications Name: HWA_FsmLockStepTest  PASSED  and Time taken for the Test is 9  micro secs

All tests have passed
\endcode

