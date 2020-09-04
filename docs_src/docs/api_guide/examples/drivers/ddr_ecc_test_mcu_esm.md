# DDR ECC Test MCU ESM {#EXAMPLES_DRIVERS_DDR_ECC_TEST_MCU_ESM}

[TOC]

\note When handling the ESM error events through MCU ESM (in M4 core), the interrupt
corresponding to MAIN ESM error event is enabled in R5 (though not handled) so that
R5 does not goes into an exception.

# Introduction

This example generates a 1b and 2b ECC error for DDR from R5. The M4 enables the
ESM instances (MAIN ESM0 and MCU ESM0).
On generating an ECC error from R5, the M4 receives the interrupt from MCU ESM
(through the MAIN ESM error signal output routed to the MCU ESM).
On receiving the interrupt M4 signals R5 (via IPC) to take corrective action.

# Supported Combinations {#EXAMPLES_DRIVERS_DDR_ECC_TEST_MCU_ESM_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/ddr/ddr_ecc_test_mcu_esm/

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
