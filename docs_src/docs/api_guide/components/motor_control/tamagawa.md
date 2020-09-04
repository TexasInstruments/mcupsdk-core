# Tamagawa {#TAMAGAWA}

[TOC]

## Introduction

The Tamagawa receiver firmware running on ICSS-PRU provides a defined well interface to execute the Tamagawa protocol.

## Features Supported

-  PRU Tamagawa receiver Firmware source in C
-  Supports full-absolute SmartAbs & SmartInc encoders compatible with Smartceiver AU5561N1
-  2.5Mbps encoder support
-  Supports all Data readout & Reset frames

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not supported, including the below
 -  EEPROM commands
 -  Other baud rates.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:
- Selecting the ICSSG0PRUx instance.(Tested on ICSSG0-PRU1)
- Configuring PINMUX.

## Tamagawa Design

\subpage TAMAGAWA_DESIGN explains the design in detail.

## Example

- \ref EXAMPLE_MOTORCONTROL_TAMAGAWA
