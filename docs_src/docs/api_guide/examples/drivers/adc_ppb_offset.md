# ADC PPB Offset {#EXAMPLES_DRIVERS_ADC_PPB_OFFSET}

[TOC]

# Introduction
## Example Description
    This example demonstrates the PPB offset features of Post Processing Blocks of ADC. The PPB offset features are the following,
1. PPB Calibration Offset
    - This is used for correcting the calibration offset of the selected SOC result.
    - For example, when set the ADC result corresponding to the set SOC ADC_result = ADC_result - PPB Calibration value.
    - If more than one PPB are set for the given SOC, then the result with higher value after calculation will be implemented.
2. PPB Reference Offset
    - This can be used to calculate an offset from a given reference.
    - The result will be specific to the PPB registers and is stored seperately from the SOC result.
    - For example, when set, the PPB result corresponding to the set SOC ADC_result, PPB_result = ADC_result - PPB Reference value.
    - If "Twos complement mode" is selected, then, PPB_result = PPB Reference value - ADC_result
## SOC Configurations
ADC SOC0 and SOC1 are set for Channel 0, with sample window 17 each and are triggered by only software.

## PPB Configurations
1. PPB1
    - Set to SOC1
    - Calibration Offset Value is set to -100
    - Reference Offset Value is set to 0
    - Twos Complement mode is disabled
1. PPB2
    - Set to SOC1
    - Calibration Offset Value is set to 50
    - Reference Offset Value is set to 50
    - Twos Complement mode is disabled
1. PPB3
    - Set to SOC1
    - Calibration Offset Value is set to 100
    - Reference Offset Value is set to 50
    - Twos Complement mode is enabled

## Interrupt Configurations
- ADC1_INT1 is set to EOC/SOC1.

## External Connections
### AM263X-CC
- Feed Analog Input to ADC1_AIN0, HSEC Pin 18
### AM263X-LP
- Feed Analog Input to ADC1_AIN0, J1/3 Pin 24

# Watch Variables
- gAdc1Soc0Result[] holds the digital representation of the Analog signal on ADC1_AIN0
- gAdc1Soc1Result[] holds the digital representation of the Analog signal on ADC1_AIN0 with Calibration offset
- gAdc1PPB1Result[] holds the digital representation of the Analog signal on ADC1_AIN0 with Calibration offset with Reference Offset of 0
- gAdc1PPB1Result[] holds the digital representation of the Analog signal on ADC1_AIN0 with Calibration offset with Reference Offset of 50
- gAdc1PPB1Result[] holds the digital representation of the Analog signal on ADC1_AIN0 with Calibration offset with Reference Offset of 50 and Twos complement enabled.

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_PPB_OFFSET_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_ppb_offset/

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
ADC PPB Offset Test Started
SOC0    SOC1(SOC0 result + calibration offset)  PPB1(SOC1 result)   PPB2(SOC1 result + ref offset) PPB3(PPB2 result in 2's complement)
	777	        675	        675	        625	        -625
	776	        675	        675	        625	        -625
	772	        674	        674	        624	        -624
	776	        676	        676	        626	        -626
	776	        676	        676	        626	        -626
	775	        675	        675	        625	        -625
	775	        674	        674	        624	        -624
	777	        675	        675	        625	        -625
ADC PPB Offset Test Passed
All tests have passed!!
\endcode
