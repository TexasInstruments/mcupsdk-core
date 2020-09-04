# EDMA Chain Transfer {#EXAMPLES_DRIVERS_EDMA_CHAIN_TRANSFER}

[TOC]

# Introduction

This example performs EDMA transfer test using chain mode.
- In chain transfer mode completion of one transfer automatically sets the event for another channel.
- The PaRAM set is initialized with two channels and provides a synchronization
  event (or trigger) to the chained DMA channel, as soon as the transfer (final or intermediate)
  completes on the main DMA/QDMA channel.This example uses AB Synchronized transfer mode.
- The source memory is initialized with sample data and destination memory
  is initialized with zeroes for validation. Cache write back is done to
  ensure that initialized data from cache is written to memory.
- When a chained completion code is detected, the value of which is dictated by the
  transfer completion code of the PaRAM set associated with the channel,
  Intermediate and final transfer interrupts are enabled and the transfer
  completion interrupt status is polled to be set before giving next trigger.
- After the transfer is completed, cache is invalidated to update cache with
  the transferred data. Data validation is performed by comparing
  source and destination memory. If the equality test is successful, the test
  was successful.


# Supported Combinations {#EXAMPLES_DRIVERS_EDMA_CHAIN_TRANSFER_COMBOS}

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | c66ss0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/edma/edma_chain_transfer

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/edma/edma_chain_transfer/

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
[EDMA] Chain Transfer Test Started...
[EDMA] Chain Transfer Test Completed!!
All tests have passed!!
\endcode

