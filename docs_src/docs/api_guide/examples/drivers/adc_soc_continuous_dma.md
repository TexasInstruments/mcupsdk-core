# ADC SOC Continuous DMA {#EXAMPLES_DRIVERS_ADC_SOC_CONTINUOUS_DMA}

[TOC]

# Introduction

This example sets up two ADC channels to convert simultaneously. The results will be
transferred by DMA into a buffer in RAM.

\imageStyle{am263_adc_soc_continuous_dma.png,width:75%}
\image html am263_adc_soc_continuous_dma.png "Module Block diagram"

## Errata i2355
errata i2355 for AM263x
- DMA trigger from the ADC INT (Late) occurs just before the Result space updation, as a result of which the stale data from the result space will be transferred by DMA.
- Workaround
    use an empty channel transfer before the Actual transfer.

The example does the below
- Configures ePWM0 to trigger SOC0 on ADC1 and ADC2. This is used to trigger the first ADC conversion.
- INT0 of ADC1 is configured to generate interrupt after first conversion and will then disable EPWM SOC generation.
- Configure ADC INT1 of both ADC's to enable continuous conversion. The interrupts will act as trigger for next conversions.
- Configure DMA channel 0 to be triggered at EOC0 of ADC1 and copy conversion result to an empty buffer, triggerring another channel channel 1 to transfer the ADC result to a result buffer in RAM.
- Configure DMA channel 2 to be triggered at EOC0 of ADC2 and copy conversion result to an empty buffer, triggerring another channel channel 3 to transfer the ADC result to another result buffer in RAM.
- Configure DMA to generate interrupt after the buffer is filled and stop conversion on both ADC's.
- The DMA destination buffers (watch variables) can be used to view the ADC conversion results.

Watch  Variables
- gAdc0DataBuffer, gAdc1DataBuffer - Digital representation of the voltage on pin ADC0_AIN0 and ADC1_AIN0

# External Connections
- ADC1_AIN0 and ADC2_AIN0 pin should be connected to the signals to be converted.
## AM263PX-CC E2 or AM263X-CC E2
 - Feed the External Volatage to the following
     - ADC1_AIN0 :   HSEC-PIN 12
     - ADC2_AIN0 :   HSEC-PIN 31
## AM263X-CC E1
 - Feed the External Volatage to the following
     - ADC1_AIN0 :   HSEC-PIN 18
     - ADC2_AIN0 :   HSEC-PIN 24
## AM263PX-LP or AM263X-LP
 - Feed the External Volatage to the following
     - ADC1_AIN0 :   J1/3 24
     - ADC2_AIN0 :   J1/3 25

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_SOC_CONTINUOUS_DMA_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

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
1048 : 1072
1048 : 1072
1048 : 1072
1048 : 1072
1048 : 1072
1048 : 1072
1048 : 1072
1048 : 1072
1048 : 1072
1048 : 1072
1048 : 1072
ADC Continuous DMA transfer Test Passed
All tests have passed!!
\endcode
