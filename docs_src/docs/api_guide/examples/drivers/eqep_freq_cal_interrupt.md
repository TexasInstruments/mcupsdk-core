# EQEP Speed Direction {#EXAMPLES_DRIVERS_EQEP_FREQ_CAL_INTERRUPT}

[TOC]

# Introduction

Frequency Measurement Using eQEP

This example is used to measure the frequency of an input signal using the eQEP module. ePWM1A is configured to generate this input signal with a frequency of 5KHz. eQEP unit timeout is set which will generate an interrupt evey UNIT_PERIOD microsecond for frequency measurement.
The PWM signal is passed internally via input XBAR to PWMXBAR to EQEPA/B

Signal path:
GPIO45/46 -> INPUTXBAR0/1 -> PWMXBAR0/1 -> EQEPA/B

The configuration for this example is as follows
 - PWM frequency is specified as 5000Hz
 - UNIT_PERIOD is specified as 10000 us
 - Min frequency is (1/(2*10ms)) i.e 50Hz
 - Highest frequency can be (2^32)/(2*10ms)
 - Resolution of frequency measurement is 50hz

 - freq : Simulated signal frequency measured by counting the external input pulses for UNIT_PERIOD (unit timer set to 10 ms).

The example does the below

Configures EPWM to generate a signal and EQEP to measure frequency of this generated signal (a loopback connection is done internally).
The application runs for the specified time and the frequency calculation is done using the EQEP ISR.
After the specified time, the application checks if the measured frequency was within range of the generated frequency.

# Internal connections

- ePWM1A to eQEP0A (EPWM_A simulates eQEP Phase A signal)
- ePWM1B to eQEP0B (EPWM_B simulates eQEP Phase B signal)

# Supported Combinations {#EXAMPLES_DRIVERS_EQEP_FREQ_CAL_INTERRUPT_COMBOS}

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/eqep/eqep_freq_cal_interrupt/

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
EQEP Frequency Measurement Test Started ...
Please wait few seconds ...
Expected frequency = 5000 Hz, Measured frequency = 5000 Hz 
EQEP Frequency Measurement Test Passed!!
All tests have passed!!
\endcode
