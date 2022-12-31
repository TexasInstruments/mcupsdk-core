# DAC Ramp Wave {#EXAMPLES_DRIVERS_DAC_RAMP_WAVE}

[TOC]

# Introduction

This example uses DAC to generate a ramp wave on the DAC output.

The example does the below
- Configures Timer to generate interrupt at a rate of 20KHz.
- Increments a variable alpha from 0 to 360 based on the output ramp wave frequency required.
- Scales alpha to the range of DAC output in the DAC Shadow register within the ISR.
- This repeated programming of the registers generates a ramp wave.

# External Connections
- DAC output pin can be connected to an oscilloscope to view the ramp wave output.

## AM263X-CC
Note: To use the reference voltage generated on ControlCard, make sure that the System VREF source select switch SW8 on ControlCard is set to position 1-2

When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Capture waveform on HSEC Pin 9

## AM263X-LP
Note: To use the reference voltage generated on LaunchPad, make sure that the DAC_VREF source select switch S1 on LaunchPad is set to position 1-2

With AM263X-LP
- Capture the waveform on boosterpack header J1/J3 Pin 30


# Supported Combinations {#EXAMPLES_DRIVERS_DAC_RAMP_WAVE_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/dac/dac_ramp_wave/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Use an Oscilloscope to view the ramp wave on DAC output pin.

# See Also

\ref DRIVERS_DAC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
DAC Ramp Wave Test Started ...
DAC Ramp Wave Test Passed!!
All tests have passed!!
\endcode

\imageStyle{am263_dac_ramp_wave.png,width:50%}
\image html am263_dac_ramp_wave.png "Sample Output"