# ECAP {#DRIVERS_ECAP_PAGE}

[TOC]

The ECAP driver provides API to configure the ECAP module.
Below are the high level features supported by the ECAP driver.

## Features Supported

- Period and duty cycle measurements of pulse train signals.
- Speed measurements of rotating machinery.
- Edge polarity selection for upto four sequenced time-stamp capture events.
- Single shot capture of upto four event time-stamps.
- Continuous mode capture of time stamps in a four-deep circular buffer.
- Absolute time-stamp capture and Delta mode time-stamp capture.
- When not used in capture mode, the eCAP module can be configured as a single channel PWM output.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Selection of ECAP instance.
- Specifying the behaviour of Time-Stamp counter during an emulation event.
- Specifying the ECAP mode.
- Specifying the event polarity in capture mode.
- Specifying interrupt source.
- Specifying ECAP input.
- Specifying APWM polarity, period and compare values in APWM mode.

## Features NOT Supported

NA

## Important Usage Guidelines

NA

## Example Usage

Include the below file to access the APIs
\snippet Ecap_sample.c include

## API

\ref DRV_ECAP_MODULE
