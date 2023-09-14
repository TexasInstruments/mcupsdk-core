# ADC SOC Continuous {#EXAMPLES_DRIVERS_ADC_SOC_CONTINUOUS}

[TOC]

# Introduction

Example Description :
     This example converts ADC 1 Channel 0 on all its SOC configurations,
essentially achieving full sampling rate on the input signal on the
given channel. The ADC clock is prescaled to a factor of 3.

\imageStyle{am263_adc_soc_continuous.png,width:50%}
\image html am263_adc_soc_continuous.png "Conversions Diagram"

## SOC Configurations
1. ADC_INT1 is set to trigger SOC(0-15).
2. All SOC are set to be triggered only throguh software
3. Sampling window is 17 system clock cycles, the conversion happens for 31
   system clock cycles. (each SOC is 49 system clock cycles long).
   therefore each sample conversion happens in approximately 4MSPS
4. EOC/SOC7 provides INT1.
5. EOC/SOC15 provides INT2.

# External Connections
## AM263X-CC E2
feed analog input on ADC 1 Channel 0 - HSEC connecter pin - 12
## AM263X-CC E1
feed analog input on ADC 1 Channel 0 - HSEC connecter pin - 18
## AM263X-LP
feed analog input on ADC 1 Channel 0 - J1/J3 Pin - 24

## Watch Variables
- gAdc1Result0 : the array holds the sampled values of the ADC 1 Channel 0

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_SOC_CONTINUOUS_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_soc_continuous/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Establish connections as mentioned in External Connections section
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Using watch variables, view the ADC conversion results.
- View the ADC conversion results in UART console logs

# See Also

\ref DRIVERS_ADC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
ADC Soc Continuous Test Started
ADC 1 channel 1 output:
	782
	770
	774
	782
	785
	777
	783
	777
	790
	786
	787
ADC Soc Continuous Test Passed
All tests have passed!!
\endcode
