# SDFM Filter sync CPU read {#EXAMPLES_DRIVERS_SDFM_FILTER_SYNC_CPUREAD}

[TOC]

# Introduction

A SDFM example that reads filter data from CPU

In this example, SDFM filter data is read by CPU in SDFM ISR routine. The
SDFM configuration is shown below:
 -  SDFM used in this example - SDFM0
 -  Input control mode selected - MODE0
 -  Comparator settings
      - Sinc3 filter selected
      - OSR = 32
      - HLT = 0x7FFF (Higher threshold setting)
      - LLT  = 0x0000(Lower threshold setting)
 -  Data filter settings
     - All the 4 filter modules enabled
     - Sinc3 filter selected
     - OSR = 128
     - All the 4 filters are synchronized by using MFE
      (Main Filter enable bit)
     - Filter output represented in 16 bit format
     - In order to convert 25 bit Data filter
       into 16 bit format user needs to right shift by 7 bits for
       Sinc3 filter with OSR = 128
 - Interrupt module settings for SDFM filter
     - All the 4 higher threshold comparator interrupts disabled
     - All the 4 lower threshold comparator interrupts disabled
     - All the 4 modulator failure interrupts disabled
     - All the 4 filter will generate interrupt when a new filter data
       is available.

Watch  Variables
-   filter1Result - Output of filter 1
-   filter2Result - Output of filter 2
-   filter3Result - Output of filter 3
-   filter4Result - Output of filter 4

# External Connections
-  Connect Sigma-Delta streams to SDFM0_CLK0, SDFM0_D0, SDFM0_CLK1, SDFM0_D1, SDFM0_CLK2, SDFM0_D2, SDFM0_CLK3, SDFM0_D3

## AM263X-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
-  Connect Sigma-Delta streams to HSEC Pin 91, HSEC Pin 72, HSEC Pin 99, HSEC Pin 101, HSEC Pin 103, HSEC Pin 105, HSEC Pin 107, HSEC Pin 109

If required, use PWM waveforms as data and clock
- Connect HSEC Pin 51 to HSEC Pin 91/99/103/107
- Connect HSEC Pin 49 to HSEC Pin 72/101/105/109

## AM263X-LP
This example is not supported on AM263X-LP

# Supported Combinations {#EXAMPLES_DRIVERS_SDFM_FILTER_SYNC_CPUREAD_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/sdfm/sdfm_filter_sync_cpuread/

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
[Cortex_R5_0] SDFM filter sync CPU read Test Started ...
All tests have passed!!
\endcode

