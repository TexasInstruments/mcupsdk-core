# RTI LED Blink {#EXAMPLES_DRIVERS_RTI_LED_BLINK}

[TOC]

# Introduction

This example configures a GPIO pin connected to an LED on the EVM in output mode.
The application toggles the LED ON/OFF for 10 seconds using RTI Timer and exits.

# Supported Combinations {#EXAMPLES_DRIVERS_RTI_LED_BLINK_COMBOS}

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/rti/rti_led_blink

\endcond

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/rti/rti_led_blink

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
\cond SOC_AM263X || SOC_AM263PX
## AM263X-CC
- Watch out for LED LD13 on the CC to blink which is controlled by GPIO0_1.

## AM263X-LP
- Watch out for LED DS2 on the LP to blink which is controlled by GPIO0_26.
\endcond
\cond SOC_AM273X
##AM273X-EVM
- Watch out for LED TPR_MSS_GPIO2 on the EVM to blink which is controlled by MSS_GPIO26.

\endcond
\cond SOC_AWR294X
##AWR294X-EVM
- Watch out for LED MSS_GPIO2 on the EVM to blink which is controlled by GPIO26.

\endcond

# Sample Output

Shown below is a sample output when the application is run,

\code
[RTI LED Blink Test] Starting ...
[RTI LED Blink Test] Timer Started...
[RTI LED Blink Test] Timer Stopped...
[RTI LED Blink Test] Blinking the LED...
All tests have passed!!
\endcode

