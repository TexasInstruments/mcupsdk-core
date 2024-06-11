# ECAP EPWM Loopback {#EXAMPLES_DRIVERS_ECAP_EPWM_LOOPBACK}

[TOC]

# Introduction

This example demonstrates ePWM to eCAP loopback test.
The ecap module is configured in the capture mode and the ecap device pin is
configured as input pin. A square wave needs to be fed to the ecap pin
externally. Based on the internal counter the count values for each of the
edge is latched in register. 4th edge will generate the interrupt. Based on
the latched counter values, calculates the input signal frequency and the
duty cycle based on the epwm input to the ecap module.
ePWM is configured to generate a square wave with 25% duty cycle.
Connect the ePWM output to eCAP input externally on the board.
Below is the setup details.
\cond SOC_AM64X
- This example needs IO breakout board for testing on AM64X-EVM.
- Short Pin 2 and 3 of J11 on IO break out board.
- Connect the EPWM output to ECAP input on the board by connecting
  Pin 7 on J6 Header in IO IO break out board to Pin 1 on J12 Header on base board.
\endcond

\cond SOC_AM243X

## AM243X-EVM
- This example needs IO breakout board for testing on AM243X-EVM.
- Short Pin 2 and 3 of J11 on IO break out board.
- Connect the EPWM output to ECAP input on the board by connecting
  Pin 7 on J6 Header in IO IO break out board to Pin 1 on J12 Header on base board.

## AM243X-LP
- Connect the EPWM output to ECAP input on the AM243X-LP board by connecting
  Pin 1 on J4(BP 40) Header to Pin 1 on J21 Header on the board.

\endcond

\cond SOC_AM273X

## AM273X-EVM
- Connect the EPWM output to ECAP input on the AM273X board.
- Remove the R144 resistor which is connected to MSS_SPIA_CLK on BallNo. G19
- On the board connect external wire from resistor R81 to MSS_SPIA_CLK connected on BallNo G19(where R144 resistor is removed).

\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_ECAP_EPWM_LOOPBACK_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 ^              | a53ss0-0 freertos
 ^              | a53ss0-0 nortos
 Toolchain      | ti-arm-clang
 ^              | arm.gnu.aarch64-none
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/ecap/ecap_epwm_loopback/

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ecap/ecap_epwm_loopback/

\endcond

\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 ^              | c66ss0 nortos
 Toolchain      | ti-arm-clang, ti-c6000
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/ecap/ecap_epwm_loopback/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Please connect the ePWM output to eCAP input externally as mentioned above in Introduction section

# See Also

\ref DRIVERS_ECAP_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
EPWM to ECAP loopback application started...
Please refer EXAMPLES_DRIVERS_ECAP_EPWM_LOOPBACK example user guide for the test setup details.
Count1 = 93751, Count2 = 31247, Count3 = 93751, Count4 = 31247
Hight time is 250 us, Low time is 750 us
Expected DutyCycle 25%, Actual DutyCycle 25%
Expected Output Frequency 1KHz, Actual Output Frequency 1KHz
All tests have passed.
\endcode
