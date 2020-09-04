# RTI {#DRIVERS_RTI_PAGE}

[TOC]

The Real Time Interrupt module provides timer functionality and is used to generate periodic interrupts.
The counters also allow you to benchmark certain areas of code by reading their values at the beginning and the end of the desired code range and calculating the difference between the values.

## Features Supported

- Support for two independent counter blocks.
- Support for four compare blocks for generating events. They can use any of the two counter blocks.
- Support for fast enabling/disabling of events.
- Two Capture functions, one for each counter block.
- Support to enable/disable continue on debug/emulation suspend. If enabled, the counters countinue to work in debug mode.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- RTI instance name
- Input clock frequency to be used for RTI module
- Enable/Disable Continue on suspend
- Support for enabling and configuring two counter blocks
- Support for enabling and configuring four compare blocks
- Support to attach callback functions to each of the compare events
- Enabling/Disable DMA trigger generation in each compare block

## Features NOT Supported

- NTU input to FRC0

## Example Usage

Include the below file to access the APIs
\snippet Rti_sample.c include

Example usage to start the timer
\snippet Rti_sample.c start

Example usage to stop the timer
\snippet Rti_sample.c stop

## API

\ref DRV_RTI_MODULE
