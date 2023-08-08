# ADC SOC Oversampling {#EXAMPLES_DRIVERS_ADC_SOC_OVERSAMPLING}

[TOC]

## Example Description :
     This example shows oversamping on a given ADC channel, periodically triggered by EPWM. ADC1 Channel 2 is sampled by SOC (2-5) and are triggered by EPWMSOCA along with SOC 0 (sampling channel 0) and SOC 1 (sampling Channel 1).

## SOC Configurations :
- SOC(0-5) are triggered by EPWMSOCA. Sample window is set at 16.
- SOC 0 samples on channel 0
- SOC 1 samples on channel 1
- SOC (2-5) sample on channel 2

## INT Configurations
- ADC1INT1 is set for EOC/SOC5

## ISR Configurations
- INTXbar0 is set for ADC1INT1.
- App_adcISR services this interrupt.
     - reads the results for the SOC(0-5),
     - freezes EPWM counter if required conversions are complete.

# External Connections
## AM263X-CC E2
     - Feed Analog Voltages on ADC 1 Channel 0 - HSEC PIN 12
     - Feed Analog Voltages on ADC 1 Channel 1 - HSEC PIN 14
     - Feed Analog Voltages on ADC 1 Channel 2 - HSEC PIN 18

## AM263X-CC E1
     - Feed Analog Voltages on ADC 1 Channel 0 - HSEC PIN 18
     - Feed Analog Voltages on ADC 1 Channel 1 - HSEC PIN 20
     - Feed Analog Voltages on ADC 1 Channel 2 - HSEC PIN 21
## AM263X-LP
     - Feed Analog Voltages on ADC 1 Channel 0 - J1/3 PIN 24
     - Feed Analog Voltages on ADC 1 Channel 1 - J1/3 PIN 29
     - Feed Analog Voltages on ADC 1 Channel 2 - J5/7 PIN 67

# Watch Variables
     - gAdc1Channel0Result - array of digital represented voltages on ADC1 channel 0
     - gAdc1Channel1Result - array of digital represented voltages on ADC1 channel 1
     - gAdc1Channel2Result - array of digital represented voltages on ADC1 channel 2

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_SOC_OVERSAMPLING_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_soc_oversampling/

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
ADC SOC Oversampling Test Started ...
ADC 1 Channel 0	Channel 1	Channel 2
	776		785		778
	777		787		780
	779		784		782
	781		787		782
	782		787		782
	783		787		784
	781		787		785
	783		789		786
	783		787		786
	783		785		784
	779		784		782
ADC SOC Oversampling Test Passed
All tests have passed!!
\endcode
