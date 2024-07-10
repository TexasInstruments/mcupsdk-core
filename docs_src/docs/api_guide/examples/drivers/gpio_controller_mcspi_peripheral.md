# GPIO Controller MCSPI peripheral {#EXAMPLES_DRIVERS_GPIO_CONTROLLER_MCSPI_PERIPHERAL}

[TOC]

# Introduction

This example demonstrates the usage of GPIO module to emulate the SPI
controller on R5FSS0_0.
MCSPI module is configured as a peripheral on R5FSS0_1 to simulate a
real controller-peripheral environment.

4 different GPIO pins are configured to emulate SPI controller by bitbanging the data.
GPIO43 as SPIPICO, GPIO44 as SPIPOCI, GPIO45 as SPICLK & GPIO46 as Chip Select(CS)
It demonstrates 8-bit and 32-bit full-duplex transfers, with configurable word and
message sizes.

The transfer is done in the worst case scenario at a mean frequency of 6.25MHz,
the controller SPICLK frequency can be fixed by using appropriate delays.

When transfer is completed, TX and RX buffer data are compared.
If data is matched, test result is passed otherwise failed.

# External Connections
- Connect GPIO43 -> SPI1_D1 (HSEC 49 -> HSEC 77)
- Connect GPIO44 -> SPI1_D0 (HSEC 51 -> HSEC 75)
- Connect GPIO45 -> SPI1_CLK (HSEC 53 -> HSEC 79)
- Connect GPIO46 -> SPI1_CS0 (HSEC 55 -> HSEC 81)
- The above pins can be connected with an oscilloscope to view the transfer waveforms

# Supported Combinations

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^			    | r5fss0-1 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/gpio/gpio_spi_master

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_MCSPI_PAGE
\ref DRIVERS_GPIO_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[Cortex_R5_0] GPIO controller operation started...
[Cortex_R5_1] MCSPI peripheral operation started...
[Cortex_R5_0]
[Cortex_R5_1] PERIPHERAL | 	 0xaaaaaaaa 	 | 	 0xaaaaaaaa
[Cortex_R5_0] CONTROLLER | 	 0xaaaaaaaa 	 | 	 0xaaaaaaaa
[Cortex_R5_1]
[Cortex_R5_0]
[Cortex_R5_1] PERIPHERAL | 	 0xaaaaaaab 	 | 	 0xaaaaaaab
[Cortex_R5_0] CONTROLLER | 	 0xaaaaaaab 	 | 	 0xaaaaaaab
[Cortex_R5_1]
[Cortex_R5_0]
[Cortex_R5_1] PERIPHERAL | 	 0xaaaaaaac 	 | 	 0xaaaaaaac
[Cortex_R5_0] CONTROLLER | 	 0xaaaaaaac 	 | 	 0xaaaaaaac
[Cortex_R5_1]
[Cortex_R5_0]
[Cortex_R5_1] PERIPHERAL | 	 0xaaaaaaad 	 | 	 0xaaaaaaad
[Cortex_R5_0] CONTROLLER | 	 0xaaaaaaad 	 | 	 0xaaaaaaad
[Cortex_R5_1] All tests have passed!!
[Cortex_R5_0] All tests have passed!!
\endcode


\imageStyle{gpio_controller_mcspi_peripheral_output.png, width:70%}
    \image html gpio_controller_mcspi_peripheral_output.png "GPIO Controller MCSPI peripheral demonstration on Logic Analyzer 8"

