# ADC Global Force {#EXAMPLES_DRIVERS_ADC_GLOBAL_FORCE}

[TOC]

# Introduction
This example demonstrates the Global SOC force feature. The ADCs that need to be forced globally are first selected in the Global Control using the API `SOC_enableAdcGlobalForce` with the instance number and the enable (TRUE/ FALSE) as arguments. then the API `SOC_adcSocGlobalForce` can be used to set a software trigger globally on the given SOC number at all the ADCs selected above. 

## Configurations
- ADCx SOC0 is configured for Software only trigger and generate interrupt at EOC0. x is 0,1,2,3,4.

# External Connections
External connections are *arbitrary* for this feature demonstration. for instance, ADC0 SOC0 is configured for Channel 0. 
Feed Analog voltages on ADC0_AIN2
## AM263Px-CC E2
- ADC0_AIN2, i.e., HSEC PIN  15
## AM263Px-LP
- ADC0_AIN2, i.e., J5/7 PIN  66

# Watch Variables
The below watch variables can be used to view ADC conversion results.
- gAdc0Results[] : Digital representation of the voltage converted by ADC0 when triggered by Global Software force
- gAdc1Results[] : Digital representation of the voltage converted by ADC1 when triggered by Global Software force
- gAdc2Results[] : Digital representation of the voltage converted by ADC2 when triggered by Global Software force
- gAdc3Results[] : Digital representation of the voltage converted by ADC3 when triggered by Global Software force
- gAdc4Results[] : Digital representation of the voltage converted by ADC4 when triggered by Global Software force

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_GLOBAL_FORCE_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_global_force/

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
ADC Triggered by Global Software Force Test Started ...
ADC Triggered by Global Software Force Test Passed
All tests have passed!!
\endcode
