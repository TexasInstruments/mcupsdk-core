# GPADC Group Channel Read{#EXAMPLES_DRIVERS_GPADC_GROUP_CHANNEL_READ}

[TOC]

# Introduction

This example configures a GPADC configuration for multi channel and triggers the multi channel conversion. Upon successful converted adc conversion data is displayed. Conversion happens based on the parameters provided through syscfg

# Supported Combinations {#EXAMPLES_DRIVERS_GPADC_GROUP_CHANNEL_READ_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/gpadc/gpadc_group_channel_read/

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_GPADC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
Channel adc conversion successful
 Channel	HW_CH		ADC Value	Volt
    0		ADC_IN0	0x000002ac	1202mV
    1		ADC_IN1	0x00000000	0000mV
    2		ADC_IN2	0x00000000	0000mV
    3		ADC_IN3	0x00000000	0000mV
    4		ADC_IN4	0x00000000	0000mV
    5		ADC_IN5	0x00000000	0000mV
    6		ADC_IN6	0x00000000	0000mV
    7		ADC_IN7	0x00000000	0000mV
    8		ADC_IN8	0x00000000	0000mV
All tests have passed!!
\endcode
