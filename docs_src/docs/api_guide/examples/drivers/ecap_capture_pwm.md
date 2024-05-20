# ECAP Capture Pwm {#EXAMPLES_DRIVERS_ECAP_CAPTURE_PWM}

[TOC]

# Introduction
## ECAP Capture Mode
ECAP runs at 200 MHz and timestamps the edges (rising and/or falling) into 4 Capture registers. This example reads a PWM wave and determines its frequency and duty cycle. Configuring the input via GPI and InputXbar into the ECAP for Falling, Rising, Falling, Rising

```mermaid

INPUT
                _________           _________           _________
                |       |           |       |           |
        ________|       |___________|       |___________|
                        ^           ^       ^           ^
EDGE EVENT             1.F         2.R     3.F         4.R
DATA            <--NA--><-OFF time-><--ON--><-OFF time->
```
ECAP will be configured to reset counter on each edge recieved, and capture for one shot.
So the timestamp registers hold the data as follows
    2. OFF time
    3. ON time
    4. OFF time

This example uses the ECAP in capture mode to capture PWM.

The example does the below
- Configures ECAP in Capture mode and captures the epwm output.
- It captures the time between the rising and falling edge of epwm output.

```mermaid
  ___________________          _____________             __________________
  |                 |          |           |             |                |
  |      EPWM       |---GPIO---| I/P XBAR  |---I/P MUX---|     ECAP       |---INT XBAR
  |_________________|          |___________|             |________________|

```

# External Connections

## AM263X-CC or AM263PX-CC
No external connection is required.

## AM263X-LP or AM263PX-LP
No external connection is required.

# Supported Combinations {#EXAMPLES_DRIVERS_ECAP_CAPTURE_PWM_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/ecap/ecap_capture_pwm/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_ECAP_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
ECAP Capture Pwm Test Started ...
Period : 10010 ns
Duty : 0.500000
frequency : 99.900101 Hz
Period : 10030 ns
Duty : 0.500000
frequency : 99.700897 Hz
Period : 10050 ns
Duty : 0.500000
frequency : 99.502487 Hz
Period : 10070 ns
Duty : 0.500000
frequency : 99.304863 Hz
Period : 10090 ns
Duty : 0.500000
frequency : 99.108025 Hz
Period : 10110 ns
Duty : 0.500000
frequency : 98.911972 Hz
Period : 10130 ns
Duty : 0.500000
frequency : 98.716682 Hz
Period : 10150 ns
Duty : 0.500000
frequency : 98.522171 Hz
Period : 10170 ns
Duty : 0.500000
frequency : 98.328415 Hz
Period : 10190 ns
Duty : 0.500000
frequency : 98.135429 Hz
Interrupt No.: 10 and Pass Count: 10
ECAP Capture Pwm Passed!!
All tests have passed!!
\endcode
