# DAC Sine DMA {#EXAMPLES_DRIVERS_DAC_SINE_DMA}

[TOC]

# Introduction

This example uses DMA to generate a sine wave on the DAC output.

\imageStyle{am263_dac_sine_dma.png,width:50%}
\image html am263_dac_sine_dma.png "Module Block diagram"

The example does the below
- Configures Timer to generate DMA trigger at a rate of 20KHz.
- Calculates sine values and stores it in a buffer (sine table).
- Uses DMA to transfer sine values stored in the sine table to DAC.
- This repeated programming of the DAC registers generates a sine wave.
- The generated output can be viewed throug an oscilloscope on the DAC output pin.

# External Connections
- DAC output pin can be connected to an oscilloscope to view the sine wave output.

## AM263X-CC or AM263PX-CC
Note: To use the reference voltage generated on ControlCard, make sure that the System VREF source select switch SW8 on ControlCard is set to position 1-2

When using AM263x-CC or AM263Px-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Capture waveform on HSEC Pin 9


## AM263X-LP or AM263PX-LP
With AM263X-LP or AM263Px-LP
Note: To use the reference voltage generated on LaunchPad, make sure that the DAC_VREF source select switch S1 on LaunchPad is set to position 1-2

- Capture the waveform on boosterpack header J1/J3 Pin 30

# Supported Combinations {#EXAMPLES_DRIVERS_DAC_SINE_DMA_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/dac/dac_sine_dma/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Use an Oscilloscope to view the sine wave on DAC output pin.

# See Also

\ref DRIVERS_DAC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
DAC Sine DMA Test Started ...
DAC Sine DMA Test Passed!!
All tests have passed!!
\endcode

\imageStyle{am263_dac_sine_dma_output.png,width:50%}
\image html am263_dac_sine_dma_output.png "Sample Output"
