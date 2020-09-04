# ADC SOC software {#EXAMPLES_DRIVERS_ADC_SOC_SOFTWARE}

[TOC]

# Introduction

This example uses the ADC to perform an ADC SOC conversion triggered by software.

The example does the below
- Configures SOC0 to be triggered by software.
- Configures ADC interrupt 1 to be generated at end of conversion of SOC0.
- Forces the SOC0 through software and waits for completion by polling ADC interrupt 1 status.

# External Connections
- ADC0_AIN0 pin should be connected to signals to be converted.

## AM263X-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Feed analog input (non-zero voltage) to HSEC Pin 12

## AM263X-LP
When using AM263x-LP
- Feed analog input (non-zero voltage) to boosterpack header J1/J3 Pin 23.

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_SOC_SOFTWARE_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_soc_software/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- View the ADC conversion results in UART console logs

# See Also

\ref DRIVERS_ADC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
ADC Software Triggered Conversion Test Started ...
ADC Software Triggered Conversion Test Passed!!
All tests have passed!!
\endcode
