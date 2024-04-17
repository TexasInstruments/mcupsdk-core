# PRU ADC {#DRIVERS_PRU_ADC}

[TOC]

## Introduction

The support for some ADCs from ADS85x8 family , ADS127L11 AND ADS131M08 are present as an example project in [MCU-PLUS-SDK-AM64X](https://www.ti.com/tool/download/MCU-PLUS-SDK-AM64X/) latest release. You can find more details for that at \ref EXAMPLES_PRU_ADC.
The ADS85x8 family ICs provide us with serial, 8 bit parallel or 16 bit parallel interface options with upto 18 bits resolution and upto 500kSPS conversion speed.
The ADS127L11 IC provide us with serial interface option with upto 24 bits resolution and upto 1.068MSPS conversion speed.
The ADS131M08 IC provide us with multi channel serial interface option with upto 24 bits resolution and upto 32KSPS conversion speed.

## General Program Flow

The App starts running on R5F core on AM64x device, which then offloads the ADC interfacing part to the PRU core.

The PRU program starts and goes into an idle state waiting for commands from R5F core to execute specific code sections as shown in the flow chart below:

  \imageStyle{pru_flow_cntrl.png,width:40%}
  \image html pru_flow_cntrl.png " "

The R5F program does the initialization of PRU (loading firmware, resetting and starting pru core) and controls the PRU to configure and interface with ADC to get the samples and send them to R5F memory.

On the R5F side we continously receive samples from PRU and can do post processing on them and can use networking modules also to send the data over network.