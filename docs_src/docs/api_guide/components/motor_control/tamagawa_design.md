# Tamagawa Protocol Design {#TAMAGAWA_DESIGN}

[TOC]

## Introduction

This document presents the firmware implementation details of the Tamagawa encoder on the AM64x/AM243x EVM.

## Tamagawa Encoder

It is an encoder technology used for obtaining high-precision position information in machine tools, robotics, and so forth. Tamagawa rotary encoders consist broadly of two types: incremental or absolute. Incremental encoders provide a train of pulses, while the absolute-type provides digital values. The absolute encoder group contains the single-turn types that provide outputs which can be open collector or emitter follower. The absolute encoder types include the pure digital encoder types, which provide a digital word output through a line driver such as an RS485, or a semi-absolute encoder, which provides both digital word and pulse train outputs. Of the RS485 line-driver output absolute encoders that provide only digital output, another classification is the full absolute encoder. A full absolute encoder provides multi-turn digital data, which is known as SmartAbs, and is compatible with the Tamagawa Smartceiver AU5561N1. Another type of encoders, known as SmartInc, provide single-turn information in digital format with an RS485 line driver output. The AM64x/AM243x Tamagawa receiver implementation is equivalent to the Smartceiver AU5561N1, which can communicate with Tamagawa SmartAbs as well as SmartInc encoders.

The AM64x/AM243x Tamagawa receiver communicates with Tamagawa SmartAbs and SmartInc encoders and provides drive control with digital information to and from the encoder. Tamagawa communication is broadly classified into three types: data readout, reset, and EEPROM transactions. Four data readout transactions occur: absolute data in one revolution, multi-turn data, encoder ID, and a combination of all of these along with the encoder error status. The reset transaction always returns the absolute data in one revolution while performing different types of resets. Three types of reset are available: reset of absolute data in one revolution, reset of multi-turn data, and error reset. The EEPROM transaction allows the system to read and write to the EEPROM in the encoder. Each transaction has a unique data ID and consists of different fields, namely control, status, data, cyclic redundancy check (CRC), EEPROM address, and EEPROM data depending on the type of transaction, that is, data ID.

Each field is 10-bits long, beginning with a start bit and ending with a delimiter. The 8 bits between these start bits and delimiters depends on the field type. The control field contains the data ID information. Data, status, and CRC fields similarly contain data, status, and CRC in those 8 bits. The receiver initially sends the control field to start the communication. This action indicates the type of transaction to the encoder and the encoder returns this information based on the data ID, as the previous paragraph explains. The encoder always returns the control field back to the receiver. In the case of data readout and reset transactions, the encoder returns the control field followed by the status, data, and ending with the CRC field at the end. In the case of an EEPROM read or write, the receiver, in addition to the control field, sends the EEPROM address field (and EEPROM data field for write) followed by the CRC. The encoder returns the control field, followed by the EEPROM address, EEPROM data, and CRC fields. The physical layer communication is RS422/RS485 based.

## System Overview

### Sitara™ AM64x/AM243x Processor

Refer TRM for details

#### PRU-ICSS

Refer PRU-ICSS chapter of AM64x/AM243x Technical Reference Manual

## Software Description

At start-up, the application running on the ARM Cortex-A9 initializes the module clocks and configures the pinmux. The PRU is initialized and the PRU firmware is loaded on PRU0 of PRU-ICSS0. After the PRU0 starts executing, the Tamagawa interface is operational and the application can use it to communicate with an encoder. Use the Tamagawa diagnostic example in the PRU-ICSS-INDUSTRIAL-DRIVES package to learn more about initialization and communication with the Tamagawa interface. This Tamagawa diagnostic example in the PRU-ICSS-INDUSTRIAL-DRIVES package (available at the path "examples/tamagawa_diagnostic" in the installed directory), also provides an easy way to validate the Tamagawa transactions. The diagnostic example provides menu options on the host PC in a serial terminal application (like TeraTerm), where the user can select the data ID code to be sent. Based on the data ID code, the application updates the Tamagwa interface with the data ID code and trigger transaction. The application then waits until it receives an indication of complete transaction by the firmware through the interface before displaying the result

### PRU Firmware Design
The firmware first initializes the PRU hardware, after which it waits until a command has been triggered through the interface. Upon triggering, the transmit data is set up based on the data ID code and the data is transmitted. The data ID code then waits until receiving all the data that depends on the data ID. The parsing over the received data then commences, which is again based on the data ID, and the interface is updated with the result. The CRC verification occurs next and the interface indicates command completion. The firmware then waits for the next command trigger from the interface.

\image html Tamagawa_flow_chart.png "Overview Flow Chart"

### Initialization
The ARM application (Tamagawa diagnostic from PRU-ICSS-INDUSTRIAL-DRIVES package) performs SoC-specific initializations like pinmux, module clock enabling , and PRU-ICSS initialization before executing the PRU firmware. After enabling the open core protocol (OCP) main ports, the transmit is disabled (a GPO is used to enable and disable transmit). Set up the UART for a 2.5-Mbps baud rate by setting the divisors appropriately. First-in-first-out (FIFO) control is enabled and the transmit and receive FIFO are cleared as well. The next step is setting the protocol for 8-bit data, 1 stop bit, and no parity.

\image html Tamagawa_initialization_flow_chart.png "Initialization Flow Chart"

### Setup Transmit Data
The transmit and receive sizes are determined based on the data ID in the interface. Then copy the transmit data from the interface to the local buffer

\image html Tamagawa_setup_tx_data.png "Setup Transmit Data Flow Chart"

### Transmit and Receive
The GPO initially enables the transmit. Write one byte at the beginning of the buffer from the local transmit buffer that has the data to be transmitted. The firmware then waits until the transmitted data returns (note that the receive is always enabled, so transmitted data always reflects back). The firmware then copies the reflected data to the receive buffer from the receive FIFO and continues until all of the data has been transmitted, after which it disables transmit. At this point, the encoder starts sending the data and the firmware copies the receive FIFO contents onto the receive buffer, individually, until all the data has been received.

\image html Tamagawa_tx_rx_flow_chart.png "Transmit and Receive Flow Chart"

### Receive Data Parse
Depending on the data ID used for initiating the transfer, the firmware parses the received data and copies it onto relevant fields in the interface, accordingly.

\image html Tamagawa_parse_data.png "Receive Data Parse Flow Chart"

### Verify CRC
The CRC is the last byte of the received data. The firmware then calculates the CRC of the received data excluding the last byte, compares it with the received CRC value, and updates the CRC status in the interface.

\image html Tamagawa_verify_crc.png "Verify CRC Flow Chart"

