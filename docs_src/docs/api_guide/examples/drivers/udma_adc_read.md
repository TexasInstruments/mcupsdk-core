# UDMA ADC Read {#EXAMPLES_DRIVERS_UDMA_ADC_READ}

[TOC]

# Introduction

This example performs PDMA RX data capture from ADC.

ADC is configured in single shot mode and captures APP_ADC_NUM_CH channel
of ADC data. The FIFO is configured to generate a DMA trigger after all
channel data is captured.

The application opens and configures a Packet DMA (PKTDMA) channel.
It configures the PDMA parameter for transfer from ADC. The PDMA element count
is set to the number of ADC samples - APP_ADC_NUM_CH.

This uses Host Packet Descriptor (HPD) to receive data from ADC PDMA channel
into the destination buffer.

The ADC is configured to tag the channel/step ID as part of ADC data using \ref ADCStepIdTagEnable API.
The application uses this to compare that the DMA read data is in proper
sequence and prints pass/fail accordingly.

# Supported Combinations {#EXAMPLES_DRIVERS_UDMA_ADC_READ_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/drivers/udma/udma_adc_read

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/drivers/udma/udma_adc_read

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref DRIVERS_UDMA_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[UDMA] ADC read application started ...
CH 0 ADC Voltage: 1106 mV
CH 1 ADC Voltage: 1107 mV
CH 2 ADC Voltage: 738 mV
CH 3 ADC Voltage: 490 mV
CH 4 ADC Voltage: 322 mV
CH 5 ADC Voltage: 205 mV
CH 6 ADC Voltage: 120 mV
CH 7 ADC Voltage: 53 mV
Loop Count: 1 completed!!

CH 0 ADC Voltage: 1107 mV
CH 1 ADC Voltage: 1107 mV
CH 2 ADC Voltage: 738 mV
CH 3 ADC Voltage: 490 mV
CH 4 ADC Voltage: 322 mV
CH 5 ADC Voltage: 205 mV
CH 6 ADC Voltage: 120 mV
CH 7 ADC Voltage: 53 mV
Loop Count: 2 completed!!

CH 0 ADC Voltage: 1107 mV
CH 1 ADC Voltage: 1107 mV
CH 2 ADC Voltage: 738 mV
CH 3 ADC Voltage: 490 mV
CH 4 ADC Voltage: 322 mV
CH 5 ADC Voltage: 204 mV
CH 6 ADC Voltage: 120 mV
CH 7 ADC Voltage: 53 mV
Loop Count: 3 completed!!

CH 0 ADC Voltage: 1107 mV
CH 1 ADC Voltage: 1108 mV
CH 2 ADC Voltage: 738 mV
CH 3 ADC Voltage: 491 mV
CH 4 ADC Voltage: 322 mV
CH 5 ADC Voltage: 205 mV
CH 6 ADC Voltage: 120 mV
CH 7 ADC Voltage: 53 mV
Loop Count: 4 completed!!

CH 0 ADC Voltage: 1107 mV
CH 1 ADC Voltage: 1107 mV
CH 2 ADC Voltage: 738 mV
CH 3 ADC Voltage: 490 mV
CH 4 ADC Voltage: 322 mV
CH 5 ADC Voltage: 205 mV
CH 6 ADC Voltage: 120 mV
CH 7 ADC Voltage: 53 mV
Loop Count: 5 completed!!

CH 0 ADC Voltage: 1107 mV
CH 1 ADC Voltage: 1108 mV
CH 2 ADC Voltage: 738 mV
CH 3 ADC Voltage: 491 mV
CH 4 ADC Voltage: 322 mV
CH 5 ADC Voltage: 205 mV
CH 6 ADC Voltage: 120 mV
CH 7 ADC Voltage: 53 mV
Loop Count: 6 completed!!

CH 0 ADC Voltage: 1107 mV
CH 1 ADC Voltage: 1108 mV
CH 2 ADC Voltage: 738 mV
CH 3 ADC Voltage: 491 mV
CH 4 ADC Voltage: 322 mV
CH 5 ADC Voltage: 205 mV
CH 6 ADC Voltage: 120 mV
CH 7 ADC Voltage: 53 mV
Loop Count: 7 completed!!

CH 0 ADC Voltage: 1106 mV
CH 1 ADC Voltage: 1107 mV
CH 2 ADC Voltage: 738 mV
CH 3 ADC Voltage: 491 mV
CH 4 ADC Voltage: 322 mV
CH 5 ADC Voltage: 205 mV
CH 6 ADC Voltage: 120 mV
CH 7 ADC Voltage: 53 mV
Loop Count: 8 completed!!

CH 0 ADC Voltage: 1106 mV
CH 1 ADC Voltage: 1107 mV
CH 2 ADC Voltage: 738 mV
CH 3 ADC Voltage: 490 mV
CH 4 ADC Voltage: 322 mV
CH 5 ADC Voltage: 204 mV
CH 6 ADC Voltage: 120 mV
CH 7 ADC Voltage: 53 mV
Loop Count: 9 completed!!

CH 0 ADC Voltage: 1107 mV
CH 1 ADC Voltage: 1107 mV
CH 2 ADC Voltage: 738 mV
CH 3 ADC Voltage: 490 mV
CH 4 ADC Voltage: 322 mV
CH 5 ADC Voltage: 205 mV
CH 6 ADC Voltage: 120 mV
CH 7 ADC Voltage: 53 mV
Loop Count: 10 completed!!

All tests have passed!!
\endcode
