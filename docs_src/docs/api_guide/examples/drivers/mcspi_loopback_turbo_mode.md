# MCSPI Loopback Turbo Mode {#EXAMPLES_DRIVERS_MCSPI_LOOPBACK_TURBO_MODE}

[TOC]

# Introduction

This example demonstrates the McSPI RX and TX operation configured
in Turbo mode.

Turbo mode improves the throughput of the MCSPI interface when a single channel
is enabled by allowing transfers until the shift register and the MCSPI_RX_0/1/2/3 register are full. Turbo mode is time saving when a transfer exceeds two words. When several channels are enabled, the TURBO bit has no effect.

This example sends a known data in the TX mode of length APP_MCSPI_MSGSIZE
and then receives the same in RX mode. Internal pad level loopback mode
is enabled to receive data.To enable internal pad level loopback mode, D0 pin is configured to both TX Enable as well as RX input pin in the SYSCFG.

When transfer is completed, TX and RX buffer data are compared.
If data is matched, test result is passed otherwise failed.

# Supported Combinations {#EXAMPLES_DRIVERS_MCSPI_LOOPBACK_TURBO_MODE_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 ^              | a53ss0-0 freertos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcspi/mcspi_loopback_turbo_mode

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 ^              | m4fss0_0_nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcspi/mcspi_loopback_turbo_mode

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcspi/mcspi_loopback_turbo_mode

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_MCSPI_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[MCSPI] Loopback example started ...
----------------------------------------------------------
McSPI Clock 50000000 Hz
----------------------------------------------------------
Data Width 	Data Length 	Transfer Time (micro sec)
8		128		39.00
----------------------------------------------------------

All tests have passed!!
\endcode

