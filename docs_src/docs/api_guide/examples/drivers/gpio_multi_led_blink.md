# GPIO MULTI LED Blink {#EXAMPLES_DRIVERS_GPIO_MULTI_LED_BLINK}

[TOC]

# Introduction

This example configures multiple GPIO pins connected to LEDs on the EVM in output mode.
The application toggles the each LED on/off for 10 seconds and exits.

# Supported Combinations {#EXAMPLES_DRIVERS_GPIO_MULTI_LED_BLINK_COMBOS}

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/gpio/gpio_multi_led_blink/

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/gpio/gpio_multi_led_blink/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
\cond SOC_AM243X
- Watch out for LED LD1 on the LP to blink which is controlled by GPIO0_22.
- Watch out for LED LD5 on the LP to blink which is controlled by GPIO0_26.
- Watch out for LED LD4 on the LP to blink which is controlled by GPIO0_27.
- Watch out for LED LD3(RED) on the LP to blink which is controlled by GPIO1_38.
- Watch out for LED LD3(GREEN) on the LP to blink which is controlled by GPIO1_39.
- Watch out for LED LD2 on the LP to blink which is controlled by GPIO1_55.
\endcond

\cond SOC_AM263X
## AM263X-CC
- Watch out for LED LD13 on the CC to blink which is controlled by GPIO0_1.
- Watch out for LED LD12 on the CC to blink which is controlled by GPIO0_22.
\endcond

\cond SOC_AM263PX
## AM263PX-CC
- Watch out for LEDs LD6 and LD7 on the CC to blink.
\endcond

# See Also

\ref DRIVERS_GPIO_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
GPIO Multi LED Blink Test Started ...
Each LED will Blink for 10 seconds ...
GPIO Multi LED Blink Test Passed!!
All tests have passed!!
\endcode
