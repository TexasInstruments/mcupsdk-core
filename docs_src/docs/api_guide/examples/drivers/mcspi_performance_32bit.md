# MCSPI Performance 32 Bit{#EXAMPLES_DRIVERS_MCSPI_PERFORMANCE_32BIT}

[TOC]

# Introduction

This Controller application demonstrates the
data transfer in controller mode with performance measurment.

- McSPI is configured in Tx Only mode with FIFO enabled for Tx.
- Word Length tested is 32 bits.
- SPI CLK Frequency used is 12 MHZ.
- Number of Words is 5.
- Data is transmitted on D0 pin.
- Data transmission is in polled mode.

# Supported Combinations {#EXAMPLES_DRIVERS_MCSPI_PERFORMANCE_32BIT_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 ^              | m4fss0-0 nortos
 ^              | a53ss0-0 freertos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcspi/mcspi_performance_32bit

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 ^              | m4fss0-0 nortos(As am243x-lp has no MCU SPI, M4 core support excluded for am243x-lp)
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcspi/mcspi_performance_32bit

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcspi/mcspi_performance_32bit

\endcond

\cond SOC_AM62X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | m4fss0-0 freertos
 ^              | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/mcspi/mcspi_performance_32bit

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

\cond !SOC_AM62X
r5fss0-0_nortos app log:
\code
[MCSPI] Performance Example Started...

----------------------------------------------------------
McSPI Clock 12000000 Hz
----------------------------------------------------------
Data Width      Data Length     Transfer Time (micro sec)
32              5               17.40
----------------------------------------------------------

All tests have passed!!
\endcode

r5fss0-0_freertos app log:
\code
[MCSPI] Performance Example Started...

----------------------------------------------------------
McSPI Clock 12000000 Hz
----------------------------------------------------------
Data Width      Data Length     Transfer Time (micro sec)
32              5               17.60
----------------------------------------------------------

All tests have passed!!
\endcode
\endcond

\cond SOC_AM64X
a53ss0-0_freertos app log
\code
[MCSPI] Performance Example Started...

----------------------------------------------------------
McSPI Clock 12500000 Hz
----------------------------------------------------------
Data Width      Data Length     Transfer Time (micro sec)
32              5               14.00
----------------------------------------------------------
\endcode
\endcond

\cond SOC_AM62X
m4fss0-0_freertos app log:
\code
[BLAZAR_Cortex_M4F_0] [MCSPI] Performance Example Started...

----------------------------------------------------------
McSPI Clock 12000000 Hz
----------------------------------------------------------
Data Width      Data Length     Transfer Time (micro sec)
32              5               17.80
----------------------------------------------------------

All tests have passed!!
\endcode
\endcond
m4fss0-0_nortos app log:
\code
[BLAZAR_Cortex_M4F_0] [MCSPI] Performance Example Started...

----------------------------------------------------------
McSPI Clock 12000000 Hz
----------------------------------------------------------
Data Width      Data Length     Transfer Time (micro sec)
32              5               17.80
----------------------------------------------------------

All tests have passed!!
\endcode
