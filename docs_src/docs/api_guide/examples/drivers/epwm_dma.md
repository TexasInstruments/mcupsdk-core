# EPWM DMA {#EXAMPLES_DRIVERS_EPWM_DMA}

[TOC]

# Introduction

This example configures an ePWM instance and DMA as follows:
  - ePWMx is set up to generate PWM waveforms
  - DMA channel 0 is set up to update the CMPAHR, CMPA, CMPBHR and CMPB every
    period with the next value in the configuration array. This allows the
     user to create a DMA enabled fifo for all the CMPx and CMPxHR registers
     to generate unconventional PWM waveforms.
  - DMA channel 1 is set up to update the TBPHSHR, TBPHS, TBPRDHR and TBPRD
    every period with the next value in the configuration array.
  - Other registers such as AQCTL can be controlled through the DMA as well
    by following the same procedure.

 \imageStyle{am263_epwm_dma.png,width:40%}
 \image html am263_epwm_dma.png "Block Diagram for EPWM DMA example"

 # External Connections

- For AM263x-CC: EPWM0_A/B pin can be connected to an oscilloscope to view the waveform.
- For AM263x-LP: EPWM1_A/B pin can be connected to an oscilloscope to view the waveform.

## AM263X-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Capture waveform on HSEC Pin 49 for epwm0_A
- Capture waveform on HSEC Pin 51 for epwm0_B

## AM263X-LP
When using AM263x-LP
- Capture waveform on boosterpack header J2/J4 Pin 37 for epwm1_A
- Capture waveform on boosterpack header J2/J4 Pin 38 for epwm1_B

# Supported Combinations {#EXAMPLES_DRIVERS_EPWM_DMA_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_dma

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_EPWM_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
EPWM DMA Test Started ...
EPWM DMA Test Passed!!
All tests have passed!!
\endcode

\imageStyle{am263_epwm_dma_output.PNG,width:80%}
 \image html am263_epwm_dma_output.PNG "EPWM DMA waveform"