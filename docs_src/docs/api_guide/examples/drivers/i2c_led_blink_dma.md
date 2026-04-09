# I2C LED Blink DMA {#EXAMPLES_DRIVERS_I2C_LED_BLINK_DMA}

[TOC]

# Introduction

This example demonstrates I2C communication with a TPIC2810 LED controller using DMA for efficient data transfer. The example showcases a sequential LED blinking pattern by controlling LEDs through I2C write operations with DMA, while also reading back LED states for verification.

The application uses the I2C DMA driver to efficiently transfer data to and from the TPIC2810 LED controller. It demonstrates how to control multiple LEDs using the TPIC2810's shift register and output loading operations. The LEDs light up in a sequential pattern, with each new LED being added to the previously lit LEDs.

Application exits after 5 iterations of LED patterns.

# Supported Combinations {#EXAMPLES_DRIVERS_I2C_LED_BLINK_DMA_COMBOS}

\cond SOC_AM263X 

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/i2c/i2c_led_blink_dma

\endcond

\cond SOC_AM263X 

## AM263X-LP
- Watch out for LED D7, D8, D9, D10, D11, D12, D13, D14 on the LP to blink which is controlled by I2C1.
\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_I2C_PAGE

# Sample Output

Shown below is a sample output when the application is run:

\code

Cortex_R5_0: [I2C] LED Blink DMA Test Started ...
Cortex_R5_0: LED will Blink for 10 loop in DMA Mode ...
Cortex_R5_0: All tests have passed!!

\endcode