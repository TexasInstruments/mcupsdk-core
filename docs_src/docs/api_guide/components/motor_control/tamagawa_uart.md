# TAMAGAWA OVER UART {#TAMAGAWA_OVER_UART}

[TOC]

## Introduction
The Tamagawa over UART module provides a support for SoC UART instance to execute the Tamagawa protocol.
## Features Supported

-  Single channel
-  Baud rate selection
-  2.5 Mbps and 5 Mbps encoder support
-  Supports all Data Readout, Reset and EEPROM commands


## Features Not Supported

-  Other baud rates
-  Long cable length

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:
- Baud rate selection(2461538 bps for 2.5 Mbps encoder, 4923076 bps for 5Mbps encoder)
- Communication mode selection (Tested on polling mode)
- Configuring GPIO62 signal with J2 pin (RTSn pin for software based flow control)
- UART instance selection


## Example

- \ref EXAMPLE_MOTORCONTROL_TAMAGAWA_OVER_UART
