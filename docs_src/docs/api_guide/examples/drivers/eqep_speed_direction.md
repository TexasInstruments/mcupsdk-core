# EQEP Speed Direction {#EXAMPLES_DRIVERS_EQEP_SPEED_DIRECTION}

[TOC]

# Introduction

Speed and Direction Measurement Using eQEP

This example can be used to sense the speed and direction of motor using eQEP in quadrature encoder mode. ePWM1A is configured to simulate motor encoder signals with frequency of 5 kHz on both A and B pins with 90 degree phase shift (so as to run this example without motor).
EQEP unit timeout is set which will generate an interrupt every UNIT_PERIOD microseconds and speed calculation occurs continuously based on the direction of motor.
The PWM signal is passed internally via input XBAR to PWMXBAR to EQEPA/B

Signal path:
GPIO45/46 -> INPUTXBAR0/1 -> PWMXBAR0/1 -> EQEPA/B

The configuration for this example is as follows
 - PWM frequency is specified as 5000Hz
 - UNIT_PERIOD is specified as 10000 us
 - Simulated quadrature signal frequency is 20000Hz (4 * 5000)
 - Encoder holes assumed as 1000
 - Thus Simulated motor speed is 300rpm (5000 * (60 / 1000))

 - freq : Simulated quadrature signal frequency measured by counting the external input pulses for UNIT_PERIOD (unit timer set to 10 ms).
 - speed : Measure motor speed in rpm
 - dir : Indicates clockwise (1) or anticlockwise (-1)

The example does the below

Configures EPWM to generate a signal and EQEP to measure speed and direction of this generated signal (a loopback connection is done internally).
The application runs for the specified time and the speed and direction calculation is done using the EQEP ISR.
After the specified time, the application checks if the measured speed was within range of the generated speed.

# Internal connections

- ePWM1A to eQEP0A (EPWM_A simulates eQEP Phase A signal)
- ePWM1B to eQEP0B (EPWM_B simulates eQEP Phase B signal)

# Supported Combinations {#EXAMPLES_DRIVERS_EQEP_SPEED_DIRECTION_COMBOS}

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/eqep/eqep_speed_direction/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- View the UART console logs for results

# See Also

\ref DRIVERS_EQEP_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
EQEP Speed Direction Test Started ...
Please wait few seconds ...
Expected speed = 300 RPM, Measured speed = 300.00 RPM
Rotation direction = Clockwise, forward
EQEP Speed Direction Test Passed!!
All tests have passed!!
\endcode
