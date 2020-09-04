# ADC High Priority SOC {#EXAMPLES_DRIVERS_ADC_HIGH_PRIORITY_SOC}

[TOC]

# Introduction

This example showcases the feature "High Priority SOC configuration"

\imageStyle{am263_adc_high_priority_soc.png,width:100%}
\image html am263_adc_high_priority_soc.png "Conversions timing diagram"
## High Priority SOC Configuration :
- When the SOC are configured in high priority (SOC0 through SOCx) the other SOC are configured in the Round Robin mode.
- The high priority SOC, when triggered, pre-empts the Round Robin Priority.
- If two or more High Priority SOC are triggered at same time, the SOC with lower number precedences.

## Configurations
- Triggers from EPWM0 are configured through Syscfg.
- the counter is configured in Stop and Freeze mode.
- the example has to change the Counter mode to get the counter running and EPWM0 can generate triggers to ADC.

- NOTE
    - PPB Delay Time Stamp cannot be used if the SOC trigger is software. hence, the example uses EPWM0 to trigger SOC conversions.
    - This example intends to showcase the Priority modes in the SOC. the input channels can be configured through Syscfg as per need.

- ADC0  (done through Syscfg)
    - SOC 0-3 are in Round Robin (no high priority selected)
    - all SOC are triggered by EPWM0SOCA signal.
    - Conversion sequence : SOC 0 --> SOC 1 --> SOC 2 --> SOC 3

- ADC1  (done through Syscfg)
    - SOC 0-3 are in Round Robin (no high priority selected)
    - SOC 1,2 are triggered by EPWM0SOCA signal.
    - SOC 0,3 are triggered by EPWM0SOCB signal.
    - Conversion sequence : SOC 1 --> SOC 2 --> SOC 3 --> SOC 0

- ADC2  (done through Syscfg)
    - SOC 0-3 are in High Priority (SOC 4-15 are in low priority mode by default)
    - all SOC are triggered by EPWM0SOCA signal.
    - Conversion sequence : SOC 0 --> SOC 1 --> SOC 2 --> SOC 3

- ADC3  (done through Syscfg)
    - SOC 0-3 are in High Priority (SOC 4-15 are in low priority mode by default)
    - SOC 1,2 are triggered by EPWM0SOCA signal.
    - SOC 0,3 are triggered by EPWM0SOCB signal.
    - Conversion sequence : SOC 1 --> SOC 2 --> SOC 0 --> SOC 3

- ADC4  (done through Syscfg)
    - SOC 0 are in High Priority (SOC 1-15 are in low priority mode by default)
    - SOC 0-3 are triggered by EPWM0SOCA signal.
    - Conversion sequence : SOC 0 --> SOC 1 --> SOC 2 --> SOC 3

- ADC4  (reset and done through the code for reference. Can also be set from syscfg)
    - SOC 0 are in High Priority (SOC 1-15 are in low priority mode by default)
    - SOC 1,2 are triggered by EPWM0SOCA signal.
    - SOC 0,3 are triggered by EPWM0SOCB signal.
    - Conversion sequence : SOC 1 --> SOC 2 --> SOC 0 --> SOC 3

PPB Configurations :
- (done through Syscfg for ADC 0-3. done through Code below for ADC 4 for reference)
- All PPBx in ADCy are configured for respective SOCx for delay in trigger to signal capture.



The example does the below
- Configures SOC0,1,2 of ADC0, as high priority SOCs, to be triggered by EPWM0.
- Configures ADC interrupt 0 to be generated at end of conversion of SOC12.
- ADC0 Interrupt ISR is used to read results of ADC0 from SOC0,SOC1 and average of SOC12 through SOC15.

Watch  Variables
- The watch variables gAdc0PpbDelay, gAdc1PpbDelay, gAdc2PpbDelay, gAdc3PpbDelay, gAdc4PpbDelay, gAdc4PpbDelay_case2 storing ADC conversion delays can be used to view the respective delays for the SOCs configured.

# External Connections
- For this example demonstration, SOCs in respective ADCs are configured for Channel 0 (arbitrarily, any channel can be selected).


# Supported Combinations {#EXAMPLES_DRIVERS_ADC_HIGH_PRIORITY_SOC_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_high_priority_soc/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- (optional) Establish connections as mentioned in External Connections section.
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Using the watch variables, view the ADC conversion results.
- View the ADC conversion results in UART console logs

# See Also

\ref DRIVERS_ADC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
ADC High Priority SOC Test Started ...

ADC0: All SOC in round robin. All SOC triggered by EPWM0SOCA
Expected: delay(SOC0)<delay(SOC1)<delay(SOC2)<delay(SOC3)
delay(SOC0)	delay(SOC1)	delay(SOC2)	delay(SOC3)
2		59		116		173
2		59		116		173

ADC1: All SOC in round robin. SOC1,2 triggered by EPWM0SOCA.SOC0,3 triggered by EPWM0SOCB
Expected: delay(SOC1)<delay(SOC2) delay(SOC3)<delay(SOC0)
delay(SOC0)	delay(SOC1)	delay(SOC2)	delay(SOC3)
59		2		59		2
59		2		59		2

ADC2: All SOC in high Priority. All SOC triggered by EPWM0SOCA
Expected: delay(SOC0)<delay(SOC1)<delay(SOC2)<delay(SOC3)
delay(SOC0)	delay(SOC1)	delay(SOC2)	delay(SOC3)
2		59		116		173
2		59		116		173

ADC3: All SOC in High Priority. SOC1,2 triggered by EPWM0SOCA.SOC0,3 triggered by EPWM0SOCB
Expected: delay(SOC1)<delay(SOC2) delay(SOC0)<delay(SOC3)
delay(SOC0)	delay(SOC1)	delay(SOC2)	delay(SOC3)
2		2		59		59
2		2		59		59

ADC4: SOC0 in high Priority. All SOC triggered by EPWM0SOCA
Expected: delay(SOC0)<delay(SOC1)<delay(SOC2)<delay(SOC3)
delay(SOC0)	delay(SOC1)	delay(SOC2)	delay(SOC3)
2		59		116		173
2		59		116		173

ADC4: SOC0 in round robin. SOC1,2 triggered by EPWM0SOCA.SOC0,3 triggered by EPWM0SOCB
Expected: delay(SOC0) should be between 0-1 conversions
	delay(SOC1) : 0 conversions, SOC2 : 2 Conversions, SOC3: 2 conversions

delay(SOC0)	delay(SOC1)	delay(SOC2)	delay(SOC3)
2		2		59		59
2		2		59		59

ADC High Priority Test Passed!!
All tests have passed!!

\endcode
