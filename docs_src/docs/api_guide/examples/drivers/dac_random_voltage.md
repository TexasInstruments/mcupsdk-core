# DAC Random Voltage {#EXAMPLES_DRIVERS_DAC_RANDOM_VOLTAGE}

[TOC]

# Introduction

This example uses DAC to generate a Random Voltage on the DAC output.

The example does the below
- Configures Timer to generate interrupt at a rate of 20KHz.
- Assigns a Random value to a output variable and wirtes it to the Shadow Regester within the ISR.
- This repeated programming of the registers generates a Random Votlage output.

# External Connections
- DAC output pin can be connected to an oscilloscope to view the Random Voltage output.

## AM263X-CC or AM263PX-CC
Note: To use the reference voltage generated on ControlCard, make sure that the System VREF source select switch SW8 on ControlCard is set to position 1-2

When using AM263x-CC or AM263Px-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Capture waveform on HSEC Pin 9

## AM263X-LP or AM263PX-LP
Note: To use the reference voltage generated on LaunchPad, make sure that the DAC_VREF source select switch S1 on LaunchPad is set to position 1-2

With AM263X-LP or AM263Px-LP
- Capture the waveform on boosterpack header J1/J3 Pin 30

# Supported Combinations {#EXAMPLES_DRIVERS_DAC_RANDOM_VOLTAGE_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/dac/dac_random_voltage/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Use an Oscilloscope to view the random voltage on DAC output pin.

# See Also

\ref DRIVERS_DAC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
DAC Random Voltage Test Started ...
DAC Random Voltage Test Passed!!
All tests have passed!!
\endcode

\imageStyle{am263_dac_random_voltage.png,width:50%}
\image html am263_dac_random_voltage.png "Sample Output"
