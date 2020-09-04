# GPIO LED Blink {#EXAMPLES_DRIVERS_GPIO_LED_BLINK}

[TOC]

# Introduction

This example configures a GPIO pin connected to an LED on the EVM in output mode.
The application toggles the LED on/off for 10 seconds and exits.
\cond SOC_AM62X
We have not any LED connected direct from MCU connection for testing gpio pins on the board, So we are using MCU_HEADER pins for gpio pins for glowing LED.
Make the following connections for this example to work on the AM62X-SK-EVM.
 - Connect LED(high) to Pin 18 of J9.
 - Connect LED(ground) to Pin 27 of J9.
\endcond
# Supported Combinations {#EXAMPLES_DRIVERS_GPIO_LED_BLINK_COMBOS}

\cond SOC_AM64X
\attention A53 NORTOS, A53 FREERTOS and A53 FREERTOS SMP support is experimental and is NOT supported by TI. \n
\endcond

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | m4fss0-0 nortos
 ^              | a53ss0-0 nortos
 ^              | a53ss0-0 freertos
 ^              | a53ss0-0 freertos-smp
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/gpio/gpio_led_blink/

\endcond

\cond SOC_AM62X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | m4fss0-0 nortos
 ^              | m4fss0-0 freertos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/gpio/gpio_led_blink/

\endcond

\cond SOC_AM243X

## AM243X-EVM
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/gpio/gpio_led_blink/

## AM243X-LP
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/gpio/gpio_led_blink/

\endcond

\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | c66ss0 nortos
 Toolchain      | ti-arm-clang, ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/gpio/gpio_led_blink/

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/gpio/gpio_led_blink/

\endcond

\cond SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/gpio/gpio_led_blink/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
\cond SOC_AM64X
- Watch out for LED LD26 on the EVM to blink which is controlled by MCU_GPIO0_5.
\endcond

\cond SOC_AM263X
## AM263X-CC
- Watch out for LED LD13 on the CC to blink which is controlled by GPIO0_1.

## AM263X-LP
- Watch out for LED DS2 on the LP to blink which is controlled by GPIO0_26.
\endcond

\cond SOC_AM243X

## AM243X-EVM
- Watch out for LED LD26 on the EVM to blink which is controlled by MCU_GPIO0_5.

## AM243X-LP
- Watch out for LED LD1 on the LP to blink which is controlled by GPIO0_22.

\endcond


\cond SOC_AM273X

##AM273X-EVM
- Watch out for LED TPR_MSS_GPIO2 on the EVM to blink which is controlled by MSS_GPIO26.

\endcond
\cond SOC_AWR294X

##AWR294X-EVM
- Watch out for LED MSS_GPIO2 on the EVM to blink which is controlled by GPIO26.

\endcond
# See Also

\ref DRIVERS_GPIO_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
GPIO LED Blink Test Started ...
LED will Blink for 10 seconds ...
GPIO LED Blink Test Passed!!
All tests have passed!!
\endcode
