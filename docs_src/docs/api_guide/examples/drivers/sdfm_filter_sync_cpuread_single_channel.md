# SDFM single channel filter sync CPU read {#EXAMPLES_DRIVERS_SDFM_FILTER_SYNC_CPUREAD_SINGLE_CHANNEL}

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
     - Single filter module enabled
     - Sinc3 filter selected
     - OSR = 128
     - Single filter is synchronized by using MFE
      (Main Filter enable bit)
     - Filter output represented in 16 bit format
     - In order to convert 25 bit Data filter
       into 16 bit format user needs to right shift by 7 bits for
       Sinc3 filter with OSR = 128
 - Interrupt module settings for SDFM filter
     - Single higher threshold comparator interrupt disabled
     - Single lower threshold comparator interrupt disabled
     - Single modulator failure interrupt disabled
     - Single filter will generate interrupt when a new filter data
       is available.

Watch  Variables
-   filterResult - Output of filter 2

# External Connections
  -  Connect Sigma-Delta streams to SDFM0_CLK1, SDFM0_D1

## AM263X-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
-  Connect Sigma-Delta streams to HSEC Pin 99, HSEC Pin 101

If required, use PWM waveforms as SDFM data and clock
- Connect HSEC Pin 51 to HSEC Pin 99
- Connect HSEC Pin 49 to HSEC Pin 101

## AM263X-LP
- Connect Sigma-Delta streams to Boosterpack header J6/J8 Pin 74, J6/J8 Pin 73

If required, use PWM waveforms as data and clock
- Connect Boosterpack header J2/J4 Pin 11 to J6/J8 Pin 73
- Connect Boosterpack header J6/J8 Pin 59 to J6/J8 Pin 74

# Supported Combinations {#EXAMPLES_DRIVERS_SDFM_FILTER_SYNC_CPUREAD_COMBOS_SINGLE_CHANNEL}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/sdfm/sdfm_filter_sync_cpuread_single_channel.c/

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

