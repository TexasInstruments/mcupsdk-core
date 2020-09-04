# ADC Burst Mode Oversampling {#EXAMPLES_DRIVERS_ADC_BURST_MODE_OVERSAMPLING}

[TOC]

# Introduction

This example sets up ePWM0 to periodically trigger a set of conversions (SOC0,1,12,13,14,15) on ADC0 for conversion of inputs on ADC_AIN0, ADC_AIN1 and burst mode conversion on ADC_AIN3. This demonstrates a batch of conversion on ADC0 (inputs on ADC_AIN0 and ADC_AIN1) and burst mode conversion on ADC0 (input ADC_AIN3). ADC0 Interrupt ISR is used to read results of ADC0 (i.e. digital representations of inputs ADC_AIN0, ADC_AIN1 and average of oversampled ADC_AIN3)

\imageStyle{am263_adc_burst_mode_oversampling.png,width:50%}
\image html am263_adc_burst_mode_oversampling.png "Module Block diagram"

The example does the below
- Configures SOC0,1,2 of ADC0, as high priority SOCs, to be triggered by EPWM0.
- Configures ADC interrupt 0 to be generated at end of conversion of SOC12.
- ADC0 Interrupt ISR is used to read results of ADC0 from SOC0,SOC1 and average of SOC12 through SOC15.

Watch  Variables
- The watch variables gAdc0Result0, gAdc0Result1, gAdc0Result2 storing ADC conversion outputs can be used to view the results.

# External Connections
- ADC0_AIN0, ADC0_AIN1, ADC0_AIN3 pins should be connected to signals to be converted.

## AM263X-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Feed analog inputs (non-zero voltage) to HSEC Pin 12, HSEC Pin 14, HSEC Pin 15

## AM263X-LP
When using LP
- Feed analog inputs (non-zero volatage) to Boosterpack header Pin 23,Pin 28 and Pin 2.


# Supported Combinations {#EXAMPLES_DRIVERS_ADC_BURST_MODE_OVERSAMPLING_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_burst_mode_oversampling/

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
ADC EPWM Triggered Burst Mode Conversions Test Started ...
ADC0 SOC0 : SOC1 : SOC2 : Result register value :
3379 : 2364 : 1524
1158 : 2314 : 1519
1158 : 2314 : 1522
1157 : 2314 : 1524
1158 : 2313 : 1520
1158 : 2313 : 1523
1158 : 2314 : 1522
1158 : 2313 : 1520
1157 : 2313 : 1527
1158 : 2314 : 1522
1158 : 2314 : 1518
ADC Burst Mode Oversampling Test Passed
All tests have passed!!
\endcode
