# SDL HWA {#EXAMPLES_SDL_HWA_MAIN}

[TOC]

# Introduction

This example demonstrates the usage of the SDL HWA module.

1.The example shows how to setup and use the ECC BUS Safety Diagnostic operation. Shows the generation of SEC DED and RED error on HWA DMA0 and HWA DMA1 bus

Note : SEC - Single Error Correction, DED - Double Error Correction, RED - Redundancy Error Correction

2.The example shows how to setup and use the Parity and FSM Lockstep Diagnostic operation. Shows the generation of parity and FSM Lockstep errors on HWA data memory

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
 UC-20    | SEC Error insertion on HWA DMA0.
 UC-21    | DED Error insertion on HWA DMA0.
 UC-22    | RED Error insertion on HWA DMA0.
 UC-23    | SEC Error insertion on HWA DMA1.
 UC-24    | DED Error insertion on HWA DMA1.
 UC-25    | RED Error insertion on HWA DMA1.

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
 HWA Application

 HWA TEST START : starting

 Applications Name: HWA_ParityDMA0DMEM0Test  PASSED

 Applications Name: HWA_ParityDMA0DMEM1Test  PASSED

 Applications Name: HWA_ParityDMA0DMEM2Test  PASSED

 Applications Name: HWA_ParityDMA0DMEM3Test  PASSED

 Applications Name: HWA_ParityDMA0DMEM4Test  PASSED

 Applications Name: HWA_ParityDMA0DMEM5Test  PASSED

 Applications Name: HWA_ParityDMA0DMEM6Test  PASSED

 Applications Name: HWA_ParityDMA0DMEM7Test  PASSED

 Applications Name: HWA_ParityDMA0WindowRamTest  PASSED

 Applications Name: HWA_ParityDMA1DMEM0Test  PASSED

 Applications Name: HWA_ParityDMA1DMEM1Test  PASSED

 Applications Name: HWA_ParityDMA1DMEM2Test  PASSED

 Applications Name: HWA_ParityDMA1DMEM3Test  PASSED

 Applications Name: HWA_ParityDMA1DMEM4Test  PASSED

 Applications Name: HWA_ParityDMA1DMEM5Test  PASSED

 Applications Name: HWA_ParityDMA1DMEM6Test  PASSED

 Applications Name: HWA_ParityDMA0DMEM7Test  PASSED

 Applications Name: HWA_ParityDMA1WindowRamTest  PASSED

 Applications Name: HWA_FsmLockStepTest  PASSED

 Applications Name: HWA_DMA0SECTest  PASSED

 Applications Name: HWA_DMA0DEDTest  PASSED

 Applications Name: HWA_DMA0REDTest  PASSED

 Applications Name: HWA_DMA1SECTest  PASSED

 Applications Name: HWA_DMA1DEDTest  PASSED

 Applications Name: HWA_DMA1REDTest  PASSED

 All tests have passed
\endcode

