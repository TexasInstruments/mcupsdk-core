# ADC Early Interrupt Offset {#EXAMPLES_DRIVERS_ADC_EARLY_INTERRUPT_OFFSET}

[TOC]

# Introduction
     The early interrupt feature allows generation (or setting) of INT flag right before the Conversion starts, but after the sample and hold (acquisition) window. This is to facilitate the early updation to the system that the ADC result could be expected. The configurability of offset for the early interrupt allows even more flexibility as of after how many sysclk cycles from the Sample and hold windw should the interrupt be generated.

\imageStyle{am263_adc_early_interrupt_offset.png,width:50%}
\image html am263_adc_early_interrupt_offset.png "Diagram showcasing Early Interrupt and Offset feature of ADC"
## Example Description
    This example demonstrates the early interrupt feature and its offset configurability for the ADC. SOC 1 is set to be triggered when ADC1_INT1 is set, which in turn is set for EOC/SOC0. Hence, SOC1 are supposed to happen at EOC0. The ADC1_INT1 is set as an early interrupt and the offset is varied. which delays the triggering of the SOC1. Although triggered, the SOC1 should wait until the conversion of SOC0 is complete. hence a delay from trigger to acquisition is present in SOC1. This can be obtained by setting the PPB (Post Processing Block) to the SOC1.
    So, by reading the varied delay against offset value, the example records the offset configurability of the early interrupt.
- Note :
    - The PPB Delay reads minimum of 2 sysclk cycles even for immediate conversion.
    - ADC Clock is set to prescale factore of 3.
## SOC Configurations
- SOC 0 :
    1. Trigger is set to software.
    2. Sample window : 16
    3. Samples on Channel 0 (arbitrary)
- SOC 1 :
    1. Trigger is set to software, ADC1_INT1.
    2. Sample window : 16
    3. Samples on Channel 0 (arbitrary)
## PPB Configurations
1. PPB 1 is set for SOC 1
## Interrupt Configurations
INT xbar0 is set for ADC 1 INT 2.
- ADC 1 INT 1 :
    1. triggered by EOC/SOC0
- ADC 1 INT 2 :
    1. triggered by EOC/SOC1
    2. triggers ISR App_adcISR()
## ISR configurations
- App_adcISR0 :
    - reads the delays in SOC1.
    - clears the INT 1, 2 flags
# External Connections
## Am263x-CC E2
Feed analog inputs to channels
         ADC1_AIN0 - HSEC Pin 12

## Am263x-CC E1
Feed analog inputs to channels
         ADC1_AIN0 - HSEC Pin 18
## Am263x-LP
Feed analog inputs to channels
         ADC1_AIN0 - J1/3 Pin 24
# Watch Variables
         gAdc1soc1Delay[] - holds the delay values for ADC1_SOC1

# Supported Combinations {#EXAMPLES_DRIVERS_ADC_EARLY_INTERRUPT_OFFSET_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/adc/adc_early_interrupt_offset/

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

ADC Early Interrupt and Configurable Offset Test Started
	Offset	Delay
	0	    31
	1	    30
	2	    29
	3	    28
	4	    27
	5	    26
	6	    25
	7	    24
	8	    23
	9	    22
	10	    21
	11	    20
	12	    19
	13	    18
	14	    17
	15	    16
	16	    15
	17	    14
	18	    13
	19	    12
	20	    11
	21	    10
	22	    9
	23	    8
	24	    7
	25	    6
	26	    5
	27	    4
	28	    3
	29	    2
	30	    2
	31	    2
ADC Early Interrupt and Configurable Offset Test Passed
All tests have passed!!

\endcode
