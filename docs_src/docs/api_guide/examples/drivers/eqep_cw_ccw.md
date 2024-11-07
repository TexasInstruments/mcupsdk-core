# EQEP CW/CCW Example {#EXAMPLES_DRIVERS_EQEP_CW_CCW}

[TOC]

# Introduction

This example demonstrates the use of eQEP (Enhanced Quadrature Encoder Pulse) module to measure position, direction, frequency and speed from CW/CCW input pulses. The example emulates encoder signals using GPIO outputs and timer-based interrupts.

\imageStyle{am26x_eqep_cw_ccw_input.png, width:60%}
    \image html am26x_eqep_cw_ccw_input.png "EQEP CW/CCW Input pulses"

## Example Description

The example configures:
- Two GPIO outputs to generate CW and CCW pulses that emulate encoder signals
- A third GPIO output to generate index pulses
- eQEP module in QMA Mode 2 (Active Pulse High)
- Timer-based interrupts to generate the pulses
- Unit timeout period of 10ms for speed measurement

The example demonstrates:
- Position counting in both CW and CCW directions
- Direction detection
- Frequency measurement
- Speed calculation in RPM
- QMA error detection

## Configuration Details

- Encoder resolution: 1000 counts/revolution
- Pulse pattern: 50 CW pulses followed by 50 CCW pulses
- Pulse frequency: 2.5kHz (150 RPM)
- Index pulse generated every 1000 pulses
- Unit timeout period: 10ms

# External Connections

The following GPIO pins can be monitored with an oscilloscope to view the generated encoder signals:
\cond SOC_AM263X || SOC_AM263PX
## AM263x-CC, AM263Px-CC
- GPIO43/Hsec 49: eQEP Phase A signal
- GPIO44/Hsec 51: eQEP Phase B signal  
- GPIO48/Hsec 52: eQEP Index Signal

## AM263x-LP, AM263Px-LP
- GPIO43/J2.11: eQEP Phase A signal
- GPIO44/J6.59: eQEP Phase B signal
- GPIO48/J4.40: eQEP Index Signal
\endcond
\cond SOC_AM261X
## AM261x-SOM
- GPIO49: eQEP Phase A signal
- GPIO50: eQEP Phase B signal
- GPIO48: eQEP Index Signal

## AM261x-LP
- GPIO49/J4.38: eQEP Phase A signal
- GPIO50/J4.37: eQEP Phase B signal
- GPIO48/J4.36: eQEP Index Signal
\endcond
# Watch Variables

The following variables can be monitored to observe the eQEP measurements:

- currentEncoderPos: Current absolute encoder position (increments with CW, decrements with CCW)
- direction: Current direction (1 = CW, -1 = CCW)
- freq: Pulse frequency in Hz
- speed: Motor speed in RPM

# Supported Combinations
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/eqep/eqep_cw_ccw

\endcond
# Steps to Run the Example

1. Import and build the project for your target configuration
2. Connect an oscilloscope to monitor the GPIO outputs if desired
3. Load and run the program
4. Monitor the watch variables to observe:
   - Position counting
   - Direction changes
   - Speed measurement
   - QMA error detection

# Sample Output

The example will display position, direction, frequency and speed measurements via UART:

\code
EQEP Position Speed Test Started ...
All tests have passed!!
\endcode

\imageStyle{am26x_eqep_cw_output.png,width:50%}
 \image html am26x_eqep_cw_output.png "EQEP CW Output"

\imageStyle{am26x_eqep_ccw_output.png,width:50%}
 \image html am26x_eqep_ccw_output.png "EQEP CCW Output"


# See Also

- \ref DRIVERS_EQEP_PAGE
- \ref DRIVERS_GPIO_PAGE
