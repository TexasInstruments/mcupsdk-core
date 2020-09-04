# DDR ECC Test MAIN ESM {#EXAMPLES_DRIVERS_DDR_ECC_TEST_MAIN_ESM}

[TOC]

\note This example demostrates handling the ESM error in the Main domain R5.
The error can also be routed to MCU ESM and error handled in the safety domain (M4 core)

# Introduction

This example simulates a 1b and 2b ECC error for DDR and waits for interrupt via the MAIN ESM instance. If the interrupt is not received the test fails.

# Supported Combinations {#EXAMPLES_DRIVERS_DDR_ECC_TEST_MAIN_ESM_COMBOS}

\cond SOC_AM243X

## AM243X-EVM
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/ddr/ddr_ecc_test_main_esm/

\endcond

\cond SOC_AM64X

## AM64X
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/ddr/ddr_ecc_test_main_esm/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_DDR_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
Waiting on Single bit Error Correction Interrupt...
1b ECC error detected and corrected
Waiting on Dual bit error detection Interrupt...
2b ECC error detected
All tests have passed!!
\endcode
