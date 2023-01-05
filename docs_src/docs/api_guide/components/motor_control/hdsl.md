# HDSL {#HDSL}

[TOC]

## Introduction

The HDSL firmware running on ICSS-PRU provides a defined well interface to execute the HDSL protocol.

## Features Supported

-  External pulse synchronization
-  Safe position
-  Supports upto 100m cable
-  Communication status
-  Register interface to be compatible with SICK HDSL FPGA IP Core.
   (except registers that have different functionality for read and
    write)
-  Fast position, speed
-  Parameter channel communication
	short message write
	Short message read
	Long message read
	Long message write

## Features Not Supported

In general, peripherals or features not mentioned as part of "Features Supported" section are not
supported, including the below
 -  Pipeline channel
 -  Safety

 ## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure things mentioned below:
- Selecting the ICSSG0PRUx instance.(Tested on ICSSG0-PRU1)
- Configuring PINMUX.

## HDSL Design

\subpage HDSL_DESIGN explains the design in detail.

## Example

- \ref EXAMPLE_MOTORCONTROL_HDSL
- \ref EXAMPLE_MOTORCONTROL_HDSL_TRACE

## API
\ref HDSL_API_MODULE