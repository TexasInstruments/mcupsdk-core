# ADC SOC software sync {#EXAMPLES_DRIVERS_ADC_SOC_SOFTWARE_SYNC}

[TOC]

# Introduction

## Example Description
     This example shows synchronous operation on ADC1 and ADC2 trigger
by a software forced by toggling a GPIO. The example uses a GPIO loopb
into the INPUTBAR[5] as trigger for the SOC in the given ADC and uses
software to toggle to the loopback GPIO trigger the conversions.

\imageStyle{am263_adc_soc_software_sync.png,width:50%}
\image html am263_adc_soc_software_sync.png "Example Block diagram"
## SOC Configurations
- ADC 1
     - SOC 0,1 triggers are set to InputXbar[5]
     - SOC 0 samples Channel 0
     - SOC 1 samples Channel 1
- ADC 2
     - SOC 0,1 triggers are set to InputXbar[5]
     - SOC 0 samples Channel 0
     - SOC 1 samples Channel 2
## Interrupt Configurations
- ADC1INT1 source is set to EOC/SOC1
- INTXbar[0] is configured for ADC1INT1
## ISR
- App_adcISR read the results stored by SOC0,1 in ADC1,2.
# External Connections
## AM263x-CC E2
     - Connect loopback on GPIO 24, GPIO 23, i.e., HSEC PINS 87, 85.
     - Feed Analog voltage on
         - ADC 1 Channel 0 : HSEC PIN 12
         - ADC 1 Channel 1 : HSEC PIN 14
         - ADC 2 Channel 0 : HSEC PIN 31
         - ADC 2 Channel 1 : HSEC PIN 33

## AM263x-CC E1
     - Connect loopback on GPIO 24, GPIO 23, i.e., HSEC PINS 87, 85.
     - Feed Analog voltage on
         - ADC 1 Channel 0 : HSEC PIN 18
         - ADC 1 Channel 1 : HSEC PIN 20
         - ADC 2 Channel 0 : HSEC PIN 24
         - ADC 2 Channel 1 : HSEC PIN 26
## AM263x-LP
     - Connect loopback on GPIO 24, GPIO 23, i.e., J5/7 PINS 49,50.
     - Feed Analog voltage on
         - ADC 1 Channel 0 : J1/3 PIN 24
         - ADC 1 Channel 1 : J1/3 PIN 29
         - ADC 2 Channel 0 : J1/3 PIN 25
         - ADC 2 Channel 1 : J5/7 PIN 63
# Watch Variables
- gAdc1Result0 : Digital representation of Voltages on ADC 1 Channel 0
- gAdc1Result1 : Digital representation of Voltages on ADC 1 Channel 1
- gAdc2Result0 : Digital representation of Voltages on ADC 2 Channel 0
- gAdc2Result1 : Digital representation of Voltages on ADC 2 Channel 1

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_SOC_SOFTWARE_SYNC_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_soc_software_sync/

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
ADC Synchronous Software Triggered Conversion Test Started ...
	ADC1		ADC2
	SOC0  SOC1	SOC0  SOC1
	450 	619 	450 	619
	678 	974 	678 	974
	702 	986 	702 	986
	687 	986 	687 	986
	698 	989 	698 	989
	695 	981 	695 	981
	687 	981 	687 	981
	703 	987 	703 	987
	706 	983 	706 	983
	715 	987 	715 	987
	728 	993 	728 	993
ADC Synchronous Software Triggered Conversion Test Passed!!
All tests have passed!!
\endcode
