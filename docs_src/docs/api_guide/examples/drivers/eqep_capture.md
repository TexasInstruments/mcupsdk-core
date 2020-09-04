# EQEP Capture {#EXAMPLES_DRIVERS_EQEP_CAPTURE}

[TOC]

# Introduction

This example demonstrates eQEP capture test.
Example configures the eQEP and captures the quadrature input signal
at index event. Example also configures the eQEP to calculate frequency
using unit timeout event. Based on the position count values, it calculates the
frequency of the input signal. EQEP signal is generated using GPIO pin toggling.
GPIO pins need to be looped back to the EQEP pins that are available in the
board. Below is the connection details.
\cond SOC_AM64X
- This example needs IO breakout board for testing on AM64X-EVM.
- All pin numbers are on the IO break out board.
 GPIO                    | EQEP
 ------------------------|-----------------
 Gpio0_26(Pin 3  of J7)  | EQEP0_A(Pin 17 of J8)
 Gpio0_27(Pin 5  of J7)  | EQEP0_B(Pin 19 of J7)
 Gpio0_43(Pin 13 of J7)  | EQEP0_S(Pin 19 of J8)
 Gpio0_44(Pin 15 of J7)  | EQEP0_I(Pin 17 of J7)

\endcond

\cond SOC_AM243X

## AM243X-EVM
- This example needs IO breakout board for testing on AM243X-EVM.
- All pin numbers are on the IO break out board.
 GPIO                    | EQEP
 ------------------------|-----------------
 Gpio0_26(Pin 3  of J7)  | EQEP0_A(Pin 17 of J8)
 Gpio0_27(Pin 5  of J7)  | EQEP0_B(Pin 19 of J7)
 Gpio0_43(Pin 13 of J7)  | EQEP0_S(Pin 19 of J8)
 Gpio0_44(Pin 15 of J7)  | EQEP0_I(Pin 17 of J7)

## AM243X-LP
All pin numbers are on the AM243x-LP board.
 GPIO                    | EQEP
 ------------------------|-----------------
 Gpio1_56(Pin 4 of J13)  | EQEP1_A(Pin 1 of J12)
 Gpio1_57(Pin 5 of J13)  | EQEP1_B(Pin 2 of J12)
 Gpio1_58(Pin 6 of J13)  | EQEP1_I(Pin 3 of J12)

\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_EQEP_CAPTURE_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/eqep/eqep_capture/

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/eqep/eqep_capture/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Please connect GPIO to EQEP pins as mentioned above in Introduction section

# See Also

\ref DRIVERS_EQEP_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
EQEP Capture application started...
Sending quadrature wave for 50 cycles in clockwise direction.With index event in between, Captures 4 edges per cycle
Quadrature input capture test clockwise direction passed
Sending quadrature wave for 50 cycles in anticlockwise direction.With index event in between, Captures 4 edges per cycle
Quadrature input capture test anti clockwise direction passed
Starting Frequency calculation test
Expected Frequency is 500 Hz
Average frequency is 500 Hz
Frequency calculation test passed
All tests have passed.
\endcode
