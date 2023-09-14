# ADC Differential Mode {#EXAMPLES_DRIVERS_ADC_DIFFERENTIAL_MODE}

[TOC]

# Introduction

This example sets up ePWM0 to periodically trigger a set of conversions (SOC0,1) on ADC1 for conversion of inputs on ADC_AIN0, ADC_AIN1 in differential modes (ADC_AIN0 - ADC_AIN1) on SOC0 and (ADC_AIN1 - ADC_AIN0) on SOC1.

Note:
- In differential mode, the outputs are symmetric across "2112 or 0x840".
- if there is a +1v on differential input, expected output should be around 2752 or 0xAC0.
- if there is a -1v on differential input, expected output should be around 1472 or 0x5C0.
- Expect wrapping of output if the readings are above 3.2v

ADC1 Interrupt ISR is used to read results of ADC1 (i.e. digital representations of differential inputs on ADC_AIN0 - ADC_AIN1 and ADC_AIN1 - ADC_AIN0 )

The below watch variables can be used to view ADC conversion results.

Watch Variables
- gAdc1Result0 - Digital representation of the differential voltage on pins ADC1_AIN0 - ADC1_AIN1
- gAdc1result1 - Digital representation of the differential voltage on pins ADC1_AIN1 - ADC1_AIN0

The example does the below
- Configures SOC0,1 of ADC1, to be triggered by EPWM0.
- Configures ADC interrupt 0 to be generated at end of conversion of SOC1.
- ADC1 Interrupt ISR is used to read results of ADC1 from SOC0, SOC1.

# External Connections
- ADC1_AIN0, ADC1_AIN1 pins should be connected to signals to be converted in differential mode.

## AM263X-CC E2
When using AM263x-CC E2 with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Feed analog inputs to
    - ADC1_AIN0 - HSEC Pin 12
    - ADC1_AIN1 - HSEC Pin 14

## AM263X-CC E1
When using AM263x-CC E1 with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Feed analog inputs to
    - ADC1_AIN0 - HSEC Pin 18
    - ADC1_AIN1 - HSEC Pin 20

## AM263X-LP
When using LP
- Feed analog inputs (non-zero volatage) to Boosterpack header Pin 24,Pin 29.


# Supported Combinations {#EXAMPLES_DRIVERS_ADC_DIFFERENTIAL_MODE_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_differential_mode/

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
ADC EPWM Triggered Differential Mode Conversions Test Started ...
ADC1 SOC0 : SOC1
2111 : 2185
1594 : 2627
1603 : 2620
1603 : 2620
1602 : 2620
1602 : 2621
1604 : 2619
1594 : 2627
1603 : 2621
1603 : 2620
1603 : 2620
ADC Differential Mode Test Passed
All tests have passed!!
\endcode
