# ADC Alternate DMA trigger {#EXAMPLES_DRIVERS_ADC_ALTERNATE_DMA_TRIGGER}

[TOC]

# Example Description
This example showcases the usage of the alternate dma trigger from ADC, in order to trigger DMA once the results have been latched in the result space, instead of triggering the DMA at the end of conversion. ADC_enableAltDMATiming() will enable this trigger.

## Configurations
The Example Configures the SOC0 in both ADC0, ADC1 to be triggered by ADC_INT1. The DMA channels 0,1 are configured for triggers from ADC0_INT1, ADC1_INT1 respectively.

## ISRs
App_dmach0ISR
- ISR triggered at the end of the DMA transfers to stop ADC SOCs from getting further triggers.

## External Connections
ADC0-SOC0 Samples on Channel 2, where as ADC1-SOC1 samples on Channel 0.
 - on AM263Px CC E2, with HSEC Dock 
     - Feed Analog input to ADC0_AIN2 - HSEC PIN 15  
     - Feed Analog input to ADC1_AIN0 - HSEC PIN 12  
 - on AM263Px LP
     - Feed Analog Input to the ADC0_AIN2 - J7 Pin 63
     - Feed Analog Input to the ADC1_AIN0 - J3 Pin 24

## Watch Variables 
 gAdc0DataBuffer[] - Buffer to store ADC0 SOC0 results, copied by DMA.
 gAdc1DataBuffer[] - Buffer to store ADC1 SOC0 results, copied by DMA.
 
# Supported Combinations {#EXAMPLES_DRIVERS_ADC_ALTERNATE_DMA_TRIGGER_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_alternate_dma_trigger/

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
ADC Alternate DMA trigger Test Started ...
ADC1 : ADC2 Result register value -
4095 : 1082
4095 : 1056
4095 : 1071
4095 : 1083
4095 : 1091
4095 : 1055
4095 : 1056
4095 : 1081
4095 : 1091
4095 : 1054
4095 : 1078
ADC Alternate DMA trigger Test Passed
All tests have passed!!
\endcode
