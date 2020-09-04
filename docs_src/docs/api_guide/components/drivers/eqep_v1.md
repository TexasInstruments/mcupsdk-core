# EQEP {#DRIVERS_EQEP_PAGE}

[TOC]

The EQEP driver provides API to configure the EQEP module.
Below are the high level features supported by the EQEP driver.

## Features Supported

- Specify behaviour of timers after an emulation event.
- 32-bit timer to generate periodic interrupts for velocity calculations.
- Integrated edge capture unit to measure the elapsed time between unit position events.
- Position compare unit to generate sync output and/or interrupt on a position compare match.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Configuring Quadrature Decoder Unit by specifying input polarity, position counter source and resolution.
- Specifying position counter mode, maximum position and value.
- Specifying edge capture prescaler and unit position event prescaler.
- Specifying unit timer period.
- Configuring EQEP Watchdog timer by specifying its period and value.
- Enabling and specifying interrupt sources.
- Specifying the emulation mode for EQEP.

## Features NOT Supported

NA

## Important Usage Guidelines

NA

## Example Usage

Include the below file to access the APIs
\snippet Eqep_sample.c include

## API

\ref DRV_EQEP_MODULE
