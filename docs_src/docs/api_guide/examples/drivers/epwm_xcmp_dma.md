# EPWM XCMP EDMA {#EXAMPLES_DRIVERS_EPWM_XCMP_DMA}

[TOC]

# Introduction

EPWM XCMP Multiple Edges

Example Description :
    The Example demonstrates xcmp shadow set registers of EPWMs to be loaded with EDMA, post period and duty calculations. Xcmp shadow values are calculated and populated in an array in TCM memory, whilst EDMA channels are configured for the transfer of these set of values to the Shadow sets of the EPWMs, xlinked with xload register.

EDMA Configurations :
1. Each EPWM xcmp shadow set has 1 EDMA channel configured, in AB synchronized mode.
2. All the EDMA channels except for the last are chained to their previous channel transfers so that, 1 manual trigger can trigger all the params
3. Last EDMA channel has interrupt enabled for the intermediate and final transfers

Shadow sets 2,3 are set to repeat 5 times each.


\imageStyle{am263_epwm_xcmp_dma_block_diagram.PNG,width:80%}
 \image html am263_epwm_xcmp_dma_block_diagram.PNG "EPWM XCMP multiple edges waveform"



# External Connections

connect the following EPWM outputs to an oscilloscope to view the waveform.

## AM263x-CC E1/E2 or AM263Px-CC E1
    - EPWM0_A and EPWM0_B pin ie, HSEC 49, HSEC 51.
    - EPWM1_A and EPWM1_B pin ie, HSEC 53, HSEC 55.
    - EPWM2_A and EPWM2_B pin ie, HSEC 50, HSEC 52.
    - EPWM3_A and EPWM3_B pin ie, HSEC 54, HSEC 56.
    - EPWM4_A and EPWM4_B pin ie, HSEC 57, HSEC 59.

## AM263x-LP
    - EPWM0_A and EPWM0_B pin ie, HSEC 49, HSEC 51.
    - EPWM1_A and EPWM1_B pin ie, HSEC 53, HSEC 55.
    - EPWM2_A and EPWM2_B pin ie, HSEC 50, HSEC 52.
    - EPWM3_A and EPWM3_B pin ie, HSEC 54, HSEC 56.
    - EPWM4_A and EPWM4_B pin ie, HSEC 57, HSEC 59.

# Supported Combinations {#EXAMPLES_DRIVERS_EPWM_XCMP_DMA_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/epwm/epwm_xcmp_dma_

\endcond



# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_EPWM_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
EPWM XCMP EDMA Test Started ...
EPWM XCMP EDMA Test Passed!!
All tests have passed!!
\endcode

\imageStyle{am263_epwm_xcmp_dma_output.PNG,width:80%}
 \image html am263_epwm_xcmp_dma_output.PNG "EPWM XCMP multiple edges waveform"

