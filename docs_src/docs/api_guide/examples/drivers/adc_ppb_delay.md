# ADC PPB Delay {#EXAMPLES_DRIVERS_ADC_PPB_DELAY}

[TOC]

# Introduction

## Example Description
This example demonstrates the PPB delay time stamp feature of the ADC. The PPBs (Post Processing Blocks) in ADC offer timestamping the delay between the conversion of given SOC and the trigger(associated) occurrence.


\imageStyle{am263_adc_ppb_delay.png,width:50%}
\image html am263_adc_ppb_delay.png "Delay Measurement"
### Note
- if a trigger occurred and the conversion occurred immediately, i.e., without any delay, the PPB captures a value '2'. and contains '0' as reset value. Hence, 0 in Delay represents SOC hasn't converted or there is an overflow of Delay counter.

## SOC Configurations
1.  SOC 0 :
    1. Trigger is set to EPWM0SOCA.
    2. Sample window : 17
    3. Samples on Channel 0 (arbitrary)
2. SOC 1 :
    1. Trigger is set to EPWM1SOCA.
    2. Sample window : 17
    3. Samples on Channel 1 (arbitrary)

## PPB Configurations
1. PPB 1 is set for SOC 0
2. PPB 2 is set for SOC 1
## Interrupt Configurations
INT xbar0, INT xbar1 are set for ADC 1 INT 1 and ADC 1 INT 2 respectively.

1. ADC 1 INT 1 :
    - triggered by EOC/SOC0
    - triggers ISR App_adcISR0()
2. ADC 1 INT 2 :
    - triggered by EOC/SOC1
    - triggers ISR App_adcISR1()
## ISR configurations
1. App_adcISR0 :
    - reads the delays in SOC0 and SOC1.
    - clears the INT 1 flag
2. App_adcISR1 :
    - clears the INT 1 flag
## EPWM Configurations
1. EPWM 0 :
    - TimeBase Period is set to 2048.
    - Generates EPWM0SOCA when the TimeBase counter equals TimeBase Period.
    - Counter Mode will be set to Up-count mode in the example.
2. EPWM 1 :
    - TimeBase Period is set to 9999.
    - Generates EPWM1SOCA when the TimeBase Counter equals TimeBase Period.
    - Counter Mode will be set to Up-count mode in the example.

# External Connections
## AM263Px-CC E2 or AM263x-CC E2
Feed analog inputs to channels
    - ADC1_AIN0 - HSEC Pin 12
    - ADC1_AIN1 - HSEC Pin 14
## AM263x-CC E1
Feed analog inputs to channels
    - ADC1_AIN0 - HSEC Pin 18
    - ADC1_AIN1 - HSEC Pin 20

## AM263Px-LP or AM263x-LP
Feed analog inputs to channels
    - ADC1_AIN0 - J1/3 Pin 24
    - ADC1_AIN1 - J1/3 Pin 29
## AM261x-LP
Feed analog inputs to channels
    - ADC1_AIN0 - J1 Pin 24
    - ADC1_AIN1 - J5 Pin 42
# Watch Variables
- gAdc1soc0Delay[] - holds the delay values for ADC1_SOC0
- gAdc1soc1Delay[] - holds the delay values for ADC1_SOC1
# Supported Combinations {#EXAMPLES_DRIVERS_ADC_PPB_DELAY_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_ppb_delay/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Using the watch variables, view the ADC conversion results.
- Observe 6 EPWM waveforms. After ADC input voltage exceeds the PPB limit, PWMs get tripped.

# See Also

\ref DRIVERS_ADC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\cond SOC_AM263X || SOC_AM263PX
\code
ADC PPB Delay Test Started
Delays associated with
conversion	SOC 0	SOC 1
	16	    13	    2
	24	    2	    18
	41	    35	    2
	66	    57	    2
	91	    2	    41
	108	    12	    2
	116	    2	    19
	133	    34	    2
	158	    56	    2
	183	    2	    42
	200	    11	    2
	208	    2	    20
	225	    33	    2
	250	    55	    2
ADC PPB Delay Test Passed
All tests have passed!!
\endcode
\endcond

\cond SOC_AM261X
ADC PPB Delay Test Started
Delays associated with
conversion	SOC 0	SOC 1
	16	    28	    2
	24	    2	    33
	41	    50	    2
	49	    2	    11
	66	    72	    2
	83	    5	    2
	91	    2	    56
	108	    27	    2
	116	    2	    34
	133	    49	    2
	141	    2	    12
	158	    71	    2
	175	    4	    2
	183	    2	    57
	200	    26	    2
	208	    2	    35
	225	    48	    2
	233	    2	    13
	250	    70	    2
ADC PPB Delay Test Passed
All tests have passed!!
\endcond
