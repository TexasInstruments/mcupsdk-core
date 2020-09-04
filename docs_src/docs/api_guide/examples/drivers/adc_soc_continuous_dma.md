# ADC SOC Continuous DMA {#EXAMPLES_DRIVERS_ADC_SOC_CONTINUOUS_DMA}

[TOC]

# Introduction

This example sets up two ADC channels to convert simultaneously. The results will be
transferred by DMA into a buffer in RAM.

\imageStyle{am263_adc_soc_continuous_dma.png,width:50%}
\image html am263_adc_soc_continuous_dma.png "Module Block diagram"

The example does the below
- Configures ePWM0 to trigger SOC0 on ADC0 and ADC1. This is used to trigger the first ADC conversion.
- INT0 of ADC0 is configured to generate interrupt after first conversion and will then disable EPWM SOC generation.
- Configure ADC INT1 of both ADC's to enable continuous conversion. The interrupts will act as trigger for next conversions.
- Configure DMA channel 0 to be triggered at EOC0 of ADC0 and copy conversion result to a buffer in RAM.
- Configure DMA channel 1 to be triggered at EOC0 of ADC1 and copy conversion result to another buffer in RAM.
- Configure DMA to generate interrupt after the buffer is filled and stop conversion on both ADC's.
- The DMA destination buffers (watch variables) can be used to view the ADC conversion results.

Watch  Variables
- gAdc0DataBuffer, gAdc1DataBuffer - Digital representation of the voltage on pin ADC0_AIN0 and ADC1_AIN0

# External Connections
- ADC0_AIN0 and ADC1_AIN0 pin should be connected to the signals to be converted.

## AM263X-CC
When using AM263x-CC with TMDSHSECDOCK (HSEC180 controlCARD Baseboard Docking Station)
- Feed analog inputs (non-zero voltage) to HSEC Pin 12 and HSEC Pin 18

## AM263X-LP
When using AM263x-LP
- Feed analog inputs (non-zero voltage) to boosterpack header J1/J3 Pin 23 and J1/J3 Pin 24.

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_SOC_CONTINUOUS_DMA_COMBOS}

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_soc_continuous_dma/

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
ADC Continuous DMA transfer Test Started ...
ADC0 : ADC1 Result register value -
3329 : 3362
1048 : 1072
1046 : 1082
1063 : 1042
1083 : 1059
1065 : 1041
1064 : 1059
1083 : 1082
1061 : 1071
1083 : 1059
1073 : 1072
ADC Continuous DMA transfer Test Passed
All tests have passed!!
\endcode
