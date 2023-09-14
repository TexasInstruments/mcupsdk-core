# EDMA Link Transfer {#EXAMPLES_DRIVERS_EDMA_LINK_TRANSFER}

[TOC]

# Introduction
This example performs EDMA transfer test using link mode.
- The link mode allows the entire PaRAM set to be reloaded from a location within the PaRAM memory map.
- The PaRAM set is initialized with two channels and sets the Link field of
  the PaRAM set associated with first channel to point  to the PaRAM set
  associated with second channel.The transfer done in AB Synchronized and
  so,CCNT triggers are needed to complete the transfer.
- The source memory is initialized with sample data and destination memory
  is initialized with zeroes for validation. Cache write back is done to
  ensure that initialized data from cache is written to memory.
- Upon completion of a transfer, the current transfer parameters are reloaded with the parameter set pointed that the 16-bit link address
  field of the current parameter.
- Intermediate and final transfer interrupts are enabled and the transfer
  completion interrupt status is polled to be set before giving next trigger.
- After the transfer is completed, cache is invalidated to update cache with
  the transferred data. Data validation is performed by comparing
  source and destination memory. If the equality test is successful, the test
  was successful.

# Supported Combinations {#EXAMPLES_DRIVERS_EDMA_LINK_TRANSFER_COMBOS}

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | c66ss0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/edma/edma_link_transfer

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/edma/edma_link_transfer/

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
[EDMA] Link Transfer Test Started...
[EDMA] Link Transfer Test Completed!!
All tests have passed!!
\endcode

