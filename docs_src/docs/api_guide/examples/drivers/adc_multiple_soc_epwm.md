# ADC Multiple SOC EPWM {#EXAMPLES_DRIVERS_ADC_MULTIPLE_SOC_EPWM}

[TOC]

# Introduction

This example sets up ePWM0 to periodically trigger a set of conversions(SOC 0,1,2)
on ADC2 and ADC1. This example demonstrates multiple ADCs working together
to process of a batch of conversions using the available parallelism
across multiple ADCs.

\imageStyle{am263_adc_multiple_soc_epwm.png,width:50%}
\image html am263_adc_multiple_soc_epwm.png "Module Block diagram"

The example does the below
- Configures SOC0,1,2 of ADC2 and ADC1 to be triggered by EPWM0.
- Configures ADC interrupt 0 to be generated at end of conversion of SOC0.
- ADC2 Interrupt ISR is used to read results of both ADC2 and ADC1.

# Watch Variables
- The watch variables gAdc2Result0, gAdc2Result1, gAdc2Result2, gAdc1Result0, gAdc1Result1, gAdc1Result2 storing ADC conversion outputs can be used to view the results.

# External Connections
- ADC2_AIN0, ADC2_AIN1, ADC2_AIN2 and ADC1_AIN0, ADC1_AIN1, ADC1_AIN2 pins should be connected to signals to be converted.

## AM263X-CC E2
When using AM263x-CC E2 with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Feed analog inputs to the following.
    - ADC2_AIN0 - HSEC 31
    - ADC2_AIN1 - HSEC 33
    - ADC2_AIN2 - HSEC 37
    - ADC1_AIN0 - HSEC 12
    - ADC1_AIN1 - HSEC 14
    - ADC1_AIN2 - HSEC 18

## AM263X-CC E1
When using AM263x-CC E1 with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Feed analog inputs to the following.
    - ADC2_AIN0 - HSEC 24
    - ADC2_AIN1 - HSEC 26
    - ADC2_AIN2 - HSEC 28
    - ADC1_AIN0 - HSEC 18
    - ADC1_AIN1 - HSEC 20
    - ADC1_AIN2 - HSEC 21

## AM263X-LP
When using LP
- Feed the analog inputs
- Feed analog inputs to the following.
    - ADC2_AIN0 - J1/J3 25
    - ADC2_AIN1 - J5/J7 63
    - ADC2_AIN2 - J5/J7 68
    - ADC1_AIN0 - J1/J3 24
    - ADC1_AIN1 - J1/J3 29
    - ADC1_AIN2 - J5/J7 67

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_MULTIPLE_SOC_EPWM_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_multiple_soc_epwm/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section.
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Using the watch variables, view the ADC conversion results.
- View the ADC conversion results in UART console logs

# See Also

\ref DRIVERS_ADC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
ADC EPWM Triggered Conversion Test Started ...
ADC2 SOC0 : SOC1 : SOC2 : ADC1 SOC0 : SOC1: SOC2 Result register value :
3379 : 2364 : 1524 : 3371 : 2348 : 2364
1158 : 2314 : 1519 : 1306 : 2316 : 2312
1158 : 2314 : 1522 : 1306 : 2316 : 2313
1157 : 2314 : 1524 : 1306 : 2316 : 2312
1158 : 2313 : 1520 : 1306 : 2316 : 2312
1158 : 2313 : 1523 : 1306 : 2316 : 2312
1158 : 2314 : 1522 : 1306 : 2316 : 2312
1158 : 2313 : 1520 : 1307 : 2316 : 2312
1157 : 2313 : 1527 : 1306 : 2316 : 2312
1158 : 2314 : 1522 : 1306 : 2316 : 2312
1158 : 2314 : 1518 : 1306 : 2316 : 2312
ADC EPWM Triggered Conversion Test Passed
All tests have passed!!
\endcode
