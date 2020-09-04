# ADC SOC EPWM {#EXAMPLES_DRIVERS_ADC_SOC_EPWM}

[TOC]

# Introduction
This examples demonstrates periodical triggering of conversion on ADC1 by EPWM0
\imageStyle{am263_adc_soc_epwm.png,width:50%}
\image html am263_adc_soc_epwm.png "Example Block diagram"
## SOC Configurations
- SOC 0 trigger is set by EPWM0SOCA
- Samples on ADC1 Channel 0
## Interrupt Configurations
- ADC1INT1 source is set to EOC/SOC0
- INTXBAR0 is set to ADC1INT1.
## ISR App_adcISR
- Reads the SOC0 result.
- Clears ADC1INT1 flag
# External Connections
## AM263x-CC
- feed analog input on ADC 1 Channel 0 - HSEC connecter pin - 18
## AM263x-LP
- feed analog input on ADC 1 Channel 0 - J1/J3 Pin - 24
# Watch Variables
- gAdc1Result0 : the array holds the sampled values of the ADC 1 Channel 0

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_SOC_EPWM_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_soc_epwm/

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
ADC Soc EPWM Test Started
ADC 1 channel 1 output:
	128
	128
	128
	128
	128
	128
	128
	128
	128
	128
	128
ADC Soc EPWM Test Passed
All tests have passed!!
\endcode
