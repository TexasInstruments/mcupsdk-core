# SENT Decoder Design {#SENT_DESIGN}

[TOC]

## Introduction

This section describes the PRU firmware implementation details of SENT Protocol Decoder .

## SENT Protocol Decoder

It is a unidirectional communication method that connects the sensor/transmitting device and the controller/receiving device without the use of a coordination signal from the controller/receiving device. It is intended for usage in situations where high-resolution data must be transferred from a sensor to an engine control unit (ECU). It serves as a simpler, less expensive alternative to CAN, LIN, or PSI5 as well as a replacement for lesser resolution techniques such as traditional sensors that provide analog output voltage and PWM. Automotive applications for SENT-compatible sensor devices include applications for pedal sensing, mass airflow sensing, pressure sensing, temperature sensing, humidity sensing, and many others.

The AM263x SENT decoder is intended to receive transmission from sensors with SENT interface and provides ECU(Engine Control Unit) with digital information from the sensor. The following pulses (all with nominal timings) make up the transmitting sequence:

1.  Synchronization/calibration pulse, first (56 clock ticks)
2.  A single pulse with a 4 bit Status and Serial Communication header (12 to 27 clock ticks)
3.  A series of one to six data nibble pulses, each reflecting the values of the signal(s) to be conveyed and lasting 12 to 27 clock ticks. Since the trailing falling edge of the SENT transmission CRC nibble also serves as the leading falling edge of the subsequent SENT transmission synchronization/calibration pulse, the number of nibbles will be fixed for each application of the encoding scheme (such as throttle position sensors, mass air flow, etc.).
4.  One 4-bit checksum(CRC) nibble pulse (12 to 27 clock ticks)
5.  One optional pause pulse

\imageStyle{sent_signal.png,width:80%}
\image html sent_signal.png "SENT Signal"

## System Overview

### Sitara™ AM263x Processor

Refer TRM for details

#### PRU-ICSS

Refer PRU-ICSS chapter of AM263x Technical Reference Manual

## Software Description

At start-up, the application running on the ARM Cortex-R5 initializes the module clocks and configures the pinmux. The PRU is initialized and the PRU firmware is loaded on both PRU slice of the ICSS instance. Once the PRU0 starts executing, the sampling logic of SENT decoder interface is operational and next PRU 1 starts executing where decoding logic is running to extract sent values from the SENT pulse and also calculate CRC. The application can use it to communicate with a SENT sensor. The application then waits until it receives an indication of complete decoding of SENT frame on each channel by the firmware through the interface before displaying the result.

### PRU0 Firmware Design (Sampling)

Once the firmware is loaded in both the PRU, PRU0 starts sampling loop which checks for falling edges on all channels, once there is falling edge, it registers the timestamp using IEP timer and sends the data to scratch pad for PRU1 to process it. The same sequence is repeated for all the channels.
\imageStyle{sent3.png,width:80%}
\image html sent3.png "Sampling"
\imageStyle{sent4.png,width:80%}
\image html sent4.png "Timestamp"
\imageStyle{sent5.png,width:80%}
\image html sent5.png "Sending data to pru1"

### PRU1 Firmware Design (Decoding)
Firmware is loaded and PRU1 starts running, it waits for PRU0 to send data using scratch pad. Once it receives data it signals PRU0 using acknowledgement bit in scratch pad. Each channel has a channel state register which maintains information about the next state. During initialization channel status register is initialized with chx_sync state, i.e. first state of sync detection and calibration. PRU1 implements a state machine which has following states Sync calculation, Status and COMM Nibble, Data 0-5 and CRC state(as shown the diagram below). After initialization, the first expected pulse is synchronization pulse. Once a valid synchronization pulse is detected, ```chx_sync``` calculates the look-up table for decoding data nibble from next pulse sent by PRU0, updates the channel state register and proceeds to next channel and computes based on respective channel state register. In case of other channel state like status and communication nibble decoding, binary search is implemented to find the associated value from 0 to 15. In case there is no match found, the channel state register goes to ```data_error``` state. The binary search implementation also accommodates for +-20% variation in pulse length as allowed in the spec. Once binary search return a valid result, next step is to store in respective channel buffer located in scratch pad. Then CRC is calculated using look up table and stored in temporary buffer. This way CRC is updated as each nibble gets decoded. At ```chx_crc``` state the aggregated CRC is match with the received CRC nibble, if check passes then the data stored in scratch pad buffer is sent to shared memory signal is sent to R5F and buffer is recycled.
\imageStyle{sent_decoder_flow.png,width:80%}
\image html sent_decoder_flow.png "State Machine for Decoder"
\imageStyle{sent10.png,width:80%}
\image html sent10.png "Send data to R5F"

##  SENT DECODER DESIGN USING PRUICSS ECAP

### R5F Configuration
Each Sent channel is mapped to GPIO Signals , GPIO Interrupt signals generated on fall edge of sent signal and are given as input to (GPIO XBAR) then OUPUT of (GPIO XBAR) is given as input to (TIME SYNC XBAR) & ICSS IEP ECAP CAP uses output of (TIME SYNC XBAR) to get time stamp , IEP ECAP registers are updated on fall edge of sent signal

### PRU0 Firmware Design (Decoding) 
Once the firmware is loaded in PRU0, ECAP ,PRU 0 starts reading the time stamps of all the channels from capture registers, Once the time stamps are updated & it follows the state machine which is same as above PRU1 implementation.
\image html sent_using_pruicss_iep_ecap.png "State Machine for Decoder Using PRUICSS IEP ECAP"
