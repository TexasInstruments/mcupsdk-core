# ADC PPB Limits {#EXAMPLES_DRIVERS_ADC_PPB_LIMITS}

[TOC]

# Introduction
- Post Processing Blocks of the ADC are featured with limit checking on the PPBResult registers. the following are limits that each PPB can check,
    1. High Limit
    2. Low Limit
    3. Zero Crossing.

- PPB can generate Event flags, for selected events. Also, each PPB can generate an EVTINT based on which event has occured. ADCxEVTINT is a logical OR of all such PPBxEVTINT in given ADC.

\imageStyle{am263_adc_ppb_limits.png,width:75%}
\image html am263_adc_ppb_limits.png "Example Block diagram"
## Example Description
This example is a demonstration of such feature and uses the loopback from DAC to generate the required analog wave form (ramp wave is choosen for this example). Once the High and Low values are touched, the PPBs generate the EVTINT and ISR will check the events of PPB0, PPB1 (both configured for SOC0) and determine which limit has occured.


## SOC configuration
- SOC0 is configured to sample on AIN0
- Sample window of 20 is selected (arbitrary)
- Software only trigger is selected.
## PPB Configurations
- SOC0 is selected for PPB0 and PPB1
- PPB1 is set for Higher Limit Checking i.e., PPB Event is set for TripHigh
- PPB1EVTINT is set for TripHigh
- PPB2 is set for Lower Limit Checking i.e., PPB Event is set for TripLow
- PPB2EVTINT is set for TripLow
Note :  PPBEVTINT is a logical OR for all the PPBxEVTINT, and is source for ISR

## ISR Configuration
- App_adc_PPB_ISR is set for INTXbar0, selected for ADC1EVTINT
- ISR checks which event has occured and load the corresponding PPB result to respective arrays.
- ISR Clears the events for further generated event identification

# External Connections

## AM263PX-CC E2 or AM263X-CC E2
Loop DAC output back to ADC1 Channel 0
- Connect HSEC Pin 9 to HSEC Pin 12
## AM263X-CC E1
Loop DAC output back to ADC1 Channel 0
- Connect HSEC Pin 9 to HSEC Pin 18

## AM263PX-LP or AM263X-LP or AM261X-LP
Loop DAC output back to ADC1 Channel 0
- Connect J1/J3 Pin 30 to J1/J3 Pin 24
(for AM261X-LP, to use DAC out at J3 30, please refer to IO expander configurations from Schematics and Syscfg)

# Watch Variables
- gAdcHighLimitOutputs[] - holds the High limit trip values
- gAdcLowLimitOutputs[] - holds the Low limit trip values

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_PPB_LIMITS_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_ppb_limits/

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

\code
ADC PPB Limits Test Started
ADC trip high events occurred for Conversion Values :
	3046
	3207
	3208
	3204
	3207
ADC trip low events occurred for Conversion Values :
	90
	195
	301
	402
	503
	608
	713
	816
	920
	90
	195
	299
	402
	503
	607
	714
	817
	919
	90
	194
	297
ADC PPB Limits Test Passed
All tests have passed!!
\endcode
