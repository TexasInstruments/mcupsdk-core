# Tamagawa {#TAMAGAWA}

[TOC]

## Introduction

The Tamagawa receiver firmware running on PRU-ICSS provides a defined well interface to execute the Tamagawa protocol. The Tamagawa diagnostic application interacts with the Tamagawa receiver firmware interface.
\note
Starting with MCU+ SDK version 08.05.00, the Tamagawa firmware and examples are based on EnDAT hardware interface from PRU-ICSSG.

## Features Supported

-  Supports full-absolute SmartAbs & SmartInc encoders compatible with Smartceiver AU5561N1
-  Channel selection
-  Baud rate selection
-  2.5 Mbps and 5 Mbps encoder support
-  Supports all Data Readout, Reset and EEPROM commands

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not supported, including the below
-  Other baud rates.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:
- Selecting the ICSSG instance
- Selecting the PRUx slice (Tested on ICSSG0-PRU1)
- Configuring PINMUX, GPIO and ICSS clock to 200MHz
- Channel selection
- Baud rate selection

## Tamagawa Design

\subpage TAMAGAWA_DESIGN explains the design in detail.

## Example

- \ref EXAMPLE_MOTORCONTROL_TAMAGAWA
