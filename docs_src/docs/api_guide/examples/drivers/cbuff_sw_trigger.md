# CBUFF Software Trigger {#EXAMPLES_DRIVERS_CBUFF_SW_TRIGGER}

[TOC]

# Introduction

This example performs CBUFF software trigger.
- The user memory is initialized with incremented pattern.
  Cache write back is done to ensure that initialized data from cache is
  written to memory.
- Configures EDMA channel PaRAM set with source address as user buffer
  and destination as CBUFF FIFO.
- DCA1000 has to be connected to EVM to capture the data sent over LVDS
  interface by the test.
- After the test is completed, a binary file would be captured by
  DCA1000 EVM which can be verified manually. If the CBUFF module triggers the
  frame done interrupt test is successful.


# Supported Combinations {#EXAMPLES_DRIVERS_CBUFF_SOFTWARE_TRIGGER_COMBOS}

\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | c66ss0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/cbuff/cbuff_sw_trigger

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[CBUFF] SW Trigger Test Started...
waiting for frameDone interrupt : 0
Received frameDone interrupt : 1
Data is transmitted over LVDS successfully.
[CBUFF] SW Trigger Test Completed!!
All tests have passed!!
\endcode

