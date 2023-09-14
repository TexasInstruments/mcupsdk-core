# ADC Burst Mode EPWM {#EXAMPLES_DRIVERS_ADC_BURST_MODE_EPWM}

[TOC]

# Introduction

This example sets up ePWM0 to periodically trigger a burst mode conversions of burst size 3 on ADC1 for conversion of inputs on ADC_AIN0, ADC_AIN1, ADC_AIN2, ADC_AIN3 and burst mode conversion on ADC_AIN3. This demonstrates a batch of conversion on ADC0 (inputs on ADC_AIN0 and ADC_AIN1) and burst mode conversion on ADC0 (input ADC_AIN3). ADC0 Interrupt ISR is used to read results of ADC0 (i.e. digital representations of inputs ADC_AIN0, ADC_AIN1 and average of oversampled ADC_AIN3)

\imageStyle{am263_adc_burst_mode_epwm.png,width:50%}
\image html am263_adc_burst_mode_epwm.png "Example Block diagram"
## Example Description
The example demonstrates Burst mode of ADC periodically triggered by trigger from EPWM0.

## SOC Configurations
 - SOC0 - SOC6 are configured High priority
 - SOC7 - SOC15 are configured for Burst mode, with Burst size of 3 i.e., SOC(7,8,9), SOC(10,11,12), SOC(13,14,15) act as bursts and
 - EPWM0SOCA is selected as trigger for Burst mode.
 - SOC(7,10,13, 14) sample on ADC 1 Channel 0
 - SOC(8,11)        sample on ADC 1 Channel 1
 - SOC(9,12)        sample on ADC 1 Channel 2
 - SOC(15)          sample on ADC 1 Channel 3

## Interrupt Configurations
 - ADC1 INT 1 is generated at the EOC/SOC9
 - ADC1 INT 2 is generated at the EOC/SOC12
 - ADC1 INT 3 is generated at the EOC/SOC15

## ISR
 - ADC INT 1,2,3 are configured to same ISR, App_adcISR.
 - reads which EOC generated Interrupt and reads the respective result registers.

# Watch Variables
 - gAdc1Result0[] : Digital representation of the voltage, averaged on burst sample on pin ADC0_AIN0
 - gAdc1Result1[] : Digital representation of the voltage, averaged on burst sample on pin ADC0_AIN1
 - gAdc1Result2[] : Digital representation of the voltage sampled on pin ADC0_AIN2
 - gAdc1result3[] : Digital representation of the voltage sampled on pin ADC0_AIN3

# External Connections :
## AM263x-CC E2 :
      Feed analog voltages on
      - ADC1_AIN0, i.e., HSEC PIN  12
      - ADC1_AIN1, i.e., HSEC PIN  14
      - ADC1_AIN2, i.e., HSEC PIN  18
      - ADC1_AIN3, i.e., HSEC PIN  20
## AM263x-CC E1 :
      Feed analog voltages on
      - ADC1_AIN0, i.e., HSEC PIN  18
      - ADC1_AIN1, i.e., HSEC PIN  20
      - ADC1_AIN2, i.e., HSEC PIN  21
      - ADC1_AIN3, i.e., HSEC PIN  23
## AM263x-LP :
      Feed analog voltages on
      - ADC1_AIN0, i.e., J1/3 PIN  24
      - ADC1_AIN1, i.e., J1/3 PIN  29
      - ADC1_AIN2, i.e., J5/7 PIN  67
      - ADC1_AIN3, i.e., J1/3 PIN  6


# Supported Combinations {#EXAMPLES_DRIVERS_ADC_BURST_MODE_EPWM_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_burst_mode_epwm/

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
ADC Burst Mode EPWM Test Started ...
ADC1Digital Converted Values
Channel0    :   Channel1    :   Channel2   : Channel3
    766  :   769  :   1533  :   766
    775  :   777  :   1550  :   777
    771  :   773  :   1546  :   769
    775  :   775  :   1546  :   772
    781  :   783  :   1556  :   780
    776  :   777  :   1545  :   770
    785  :   784  :   1569  :   781
    780  :   777  :   1554  :   778
    782  :   781  :   1564  :   781
    789  :   789  :   1576  :   787
    781  :   784  :   1561  :   785
ADC Burst Mode EPWM Test Passed
All tests have passed!!
\endcode
