# HDSL Protocol Design {#HDSL_DESIGN}

[TOC]

## Introduction

This document presents the firmware implementation details of the Hiperface DSL protocol (SICK STEGMANN, 2010) for the PRU0 in ICSS0 on the AM64x/AM243x EVM.

## System Overview

### Sitara™ AM64x/AM243x Processor

Refer TRM for details

#### PRU-ICSS

Refer PRU-ICSS chapter of AM64x/AM243x Technical Reference Manual

## Software Architecture

The firmware consists of two layers .On the one hand, there is the datalink layer, which is responsible for establishing a communication link to the encoder, monitoring the connection quality and preparing the data.
On the other hand, there is the transport layer that processes the data and determines what information is sent over the parameter channel. Figure "Layer Model" illustrates the relationship between the two layers.
The datalink layer assembles the information from the different channels and puts the data symbol by symbol to the channel buffers. The channel buffers are large enough to carry the data of a whole V-Frame for each channel. .
The transport layer controls the data sent over the parameter channel by setting the symbol to send for the next H-Frame in the parameter channel buffer. This buffer can carry only one symbol.
Both layers have direct access to the register interface that is provided to the higher layers.


\image html hdsl_layer_model.png "Layer Model "

Hiperface DSL specifies a state machine for the Receiver. This implementation features an additional state for loading new firmware to the PRU. Figure "State Machine" depicts the modified state machine.
Furthermore, this implementation exhibits three code sections in the firmware. The first one is for initializing the state machine up to the LOADFW state.

\image html hdsl_state_machine.png "State Machine"

The second section contains datalink functionalities that are needed for the startup phase as well as for the normal workflow. The transport layer functionalities reside in the third section.

### Datalink Layer

The datalink layer is responsible for handling the communication link to the encoder. This includes the sampling, cable delay compensation, DC line balancing, encoding and decoding of data and the monitoring of the connection quality.

#### Sampling
During the reception of the salve answer, the SCU oversamples the data by factor 8. This allows the firmware to compensate signal deficits, such as delay. During the LEARN state the receiver calculates the sample edge based on the first received bit.
Assuming the oversampled data is exactly aligned with one bit, the best position for the sample edge would be either bit 3 or bit 4. An unalignment of the oversampled data with the actual bit results in a shift of the sample edge. The unalignment can be measured by counting the number of ‘1’ in the data, whereas a count of 4 equals the worst alignment and a count of 0 or 8 equals perfect alignment. The number of ’1’ (n) in the oversampled data is determined using a LUT and the following calculation provides the position for the sample edge (E):

E=(4+n)%8

\image html hdsl_sampling.png "Sampling"

#### Delay Measurement and Compensation
During the LEARN state the encoder sends a test pattern to the receiver. This is used to determine the cable delay. While the test pattern is sent, the receiver records all incoming bits and searches for the beginning of the test pattern. The offset, where the test pattern starts, is the cable delay in units of bits.

\image html hdsl_test_pattern.png "Test Pattern"

After the cable delay is measured, the receiver uses this knowledge to compensate the cable delay in subsequent states. This is performed by waiting for the calculated amount of bits as soon as the encoder answer window starts. The next bit on the line is the first bit of the actual encoder answer.

#### Data Encoding and Decoding
The datalink layer has the responsibility to decode and encode the data according to an 8b/10b scheme (Franaszek, 1983). The 8b/10b encoding/decoding is split into two parts, 3b/4b and 5b/6b encoding/decoding. Each of the encoding and decoding processes is performed by using a LUT. Hiperface DSL assumes a transmission with LSB first. Therefore, in the encoding procedure, the index of the LUT is in MSB first order, while the LUT entries are in LSB first order (and vice versa when decoding data).  This way, the firmware does not need to handle the reversing of the bit order. When encoding the data, the firmware handles the sending of the correct polarity of the sub-blocks using the measured line disparity.
During receive, the firmware detects byte errors and special characters by checking the received encoded data according to the paper (Franaszek, 1983)

#### Received Signal Strength Indication (RSSI)

The RSSI is calculated by determining the number of samples between two edges during a bit period. The samples that form the longest sequence between two edges represent the stable bit period, which is used to calculate the RSSI. Instead of calculating the stable period in the firmware, a pre-calculated LUT is utilized to speed up the process. First, the edges in a bit period are determined, which is performed by a XOR operation (Figure: hdsl_rssi)]. The searched RSSI value is looked-up in the table by using the result of the XOR operation as the index.

\image html hdsl_rssi.png "Test Pattern"


#### Cyclic Redundancy Check Algorithm

A 16bit CRC verification of the data is used on multiple occasions. It is used for the vertical channel, secondary channel and messages. In order to distribute the computation load equally over all H-Frames, the firmware calculates a running CRC for those data (except for short messages). The algorithm uses a LUT with 256 entries and 2 bytes per entry, whereas each entry is the 16bit CRC for the corresponding LUT index. The basic approach for the calculation of the 16bit CRC is shown as C code in the following:

uint16_t calc_crc(uint8_t *data, uint32_t size) { uint16_t crc = 0; uint32_t i; for(i = 0; i < size; ++i) { crc = ((*data) << 8) ^ crc; crc = lut[crc>>8] ^ (crc << 8); } return (crc ^ 0xff); }

### Transport Layer

The transport layer processes the channel information which was prepared by the datalink layer. This includes the calculation of the fast position as well as the handling of messages.

#### ∆∆Position Estimation
During normal workflow, it can occur that the received ∆∆Position data cannot be used for calculations. This is the case on either a transmission error or an internal encoder error. In order to check for a transmission error, the transport layer checks if the datalink layer detected a byte error and verifies the CRC in the acceleration channel. If no transmission error occurred, the transport layer searches for the occurrence of two K29.7 to recognize an internal encoder error. In case one of the verification of the data fails, the estimation algorithm shown in Figure

\image html hdsl_delta_pos.png "Estimation Algorithm for ∆∆Position"


## Messages
The transport layer handles the messaging. Since it is possible that the higher layers send a long and a short message at the same time, the transport layer has to decide which message to send first. In this implementation short messages are always favored over long messages.


### Short Message
Remote (DSL motor feedback system) registers that indicate interface information are mirrored in the DSL Receiver under register addresses 40h to 7Fh. These remote registers are addressed in the same way as DSL Receiver registers. As the values of remote registers are transmitted via the Parameters Channel and hence via the DSL cables, the delay between polling and answer for "short message" transactions depends on the connection cables of the systems in question. There is no delay, as this information is stored directly in the IP-Core S_PC_DATA.. The Parameters Channel can only transmit one "short message" at a time. Several remote registers can only be polled in sequence, i.e. after the previous answer has been received.

Note: It should be noted that a "short message" can be triggered during a running "long message" transaction.

