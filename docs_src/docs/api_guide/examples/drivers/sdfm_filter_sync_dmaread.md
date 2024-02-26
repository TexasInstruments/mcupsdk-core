# SDFM Filter sync DMA read {#EXAMPLES_DRIVERS_SDFM_FILTER_SYNC_DMAREAD}

[TOC]

# Introduction

A SDFM example that reads filter data from CPU
In this example, SDFM filter data is read by DMA by SDFM FIFO DATA READTY INT trigger.
SDFM configuration is shown below:
  - SDFM used in this example - SDFM0
  - Input control mode selected - MODE0
  - Data filter settings
    - Filter 1 is enabled
    - Sinc3 filter selected
    - OSR = 256
    - Synchronized by using MFE (Main Filter enable bit)
    - Filter output represented in 16 bit format
    - In order to convert 26 bit Data filter into 16 bit format user needs to right shift by 10 bits for Sinc3 filter with OSR = 256
    - FIFO is enabled for 16 levels. 
  - Interrupt module settings for SDFM filter
    - Filter 1 will generate interrupt when a new filter FIFO Threshold is crossed.


\imageStyle{am263_sdfm_dmaread_block_diagram.png,width:75%}
\image html am263_sdfm_dmaread_block_diagram.png "Example Block diagram"

## External Connections
  -  Connect Sigma-Delta streams to
    SDFM0_CLK0, SDFM0_D0

## Watch Variables
-   filter1Result - Output of filter 1

## AM263X-CC or AM263PX-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
-  Connect Sigma-Delta streams to HSEC Pin 91, HSEC Pin 72

If required, use PWM waveforms as data and clock
- Connect HSEC Pin 51 to HSEC Pin 91
- Connect HSEC Pin 49 to HSEC Pin 72

## AM263X-LP or AM263PX-LP
This example is not supported on AM263X-LP

# Supported Combinations {#EXAMPLES_DRIVERS_SDFM_FILTER_SYNC_DMAREAD_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/sdfm/sdfm_filter_sync_dmaread/

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

\ref DRIVERS_SDFM_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
SDFM filter sync CPU read Test Started ...
gSdfmFifo1Base : 50268032

Some of the copied FIFO data...
Transfer number : 0 | Data : -2731 -13654 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -1638 
Transfer number : 12 | Data : -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16 
Transfer number : 24 | Data : -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16 
Transfer number : 36 | Data : -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16 
Transfer number : 48 | Data : -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16 
Transfer number : 60 | Data : -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16384 -16 

All tests have passed!!

\endcode

