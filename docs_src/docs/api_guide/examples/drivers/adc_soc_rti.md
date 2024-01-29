# ADC SOC RTI {#EXAMPLES_DRIVERS_ADC_SOC_RTI}

[TOC]

# Introduction
This example demostrates how the RTI and ADC to be configured so that the ADC SOC can be triggered from the RTI Events. 

## Configurations
- RTI is configured to generate trigger every 5uS. Refer to the syscfg of the RTI for the configurational details
- ADC SOC0 is configured for RTI trigger and generate interrupt at EOC0

## ISRs
App_adcISR
- ISR to read ADC SOC0 result and service the interrupt generated at EOC0.
App_rtiISR
- ISR callback to service RTI interrupt.

Both ISR have a counter to increment on each call. these counters are compared for the validation of the ADC triggers from RTI.

# External Connections
Feed Analog voltages on ADC0_AIN2
## AM263x-CC E2 or AM263Px-CC E2
- ADC0_AIN2, i.e., HSEC PIN  15
## AM263x-LP or AM263Px-LP
- ADC0_AIN2, i.e., J5/7 PIN  66

# Watch Variables
The below watch variables can be used to view ADC conversion results.
- gAdcResults[] : Digital representation of the voltage sample on pin AIN2 of ADC0, triggered by RTI
- gAdcIsrCount  : counter for the ADC ISR 
- gRtiIsrCount  : counter for the RTI ISR 

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_SOC_RTI_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_soc_rti/

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
ADC Triggered by RTI Test Started ...
ADC Triggered by RTI Test Passed
All tests have passed!!
\endcode
