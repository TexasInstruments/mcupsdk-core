# Swap To Region B {#EXAMPLES_DRIVERS_SWAP_TO_B}

[TOC]

# Introduction

    This example: 
    1. writes data to flash at 18MB offset. 
    2. remaps address from 16MB and above to 0MB and above.
    3. reads back the data from 2MB offset.
    4. checks if data that is read back is correct or not.


# Supported Combinations {#EXAMPLES_DRIVERS_SWAP_TO_B_COMBOS}


\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/fss/swap_to_b

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# Sample Output


\code

All tests have passed!!

\endcode
