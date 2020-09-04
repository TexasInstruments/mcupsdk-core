# SDFM {#DRIVERS_SDFM_PAGE}

[TOC]

The SDFM driver provides API to configure the SDFM module.
Below are the high level features supported by the SDFM driver.

## Features Supported

- Ability to detect over-value, under-value and threshold-crossing conditions.
- A programmable mode FIFO in data filter unit to reduce interrupt overhead.
- FIFO can interrupt CPU after programmable number of data-ready events.
- Ability to ignore data-ready events until the PWM synchronization signal is received.
- PWMs can be used to generate a modulator clock for sigma-delta modulators.
- Ability to use one filter channel clock to provide clock to other filter clock channels.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

## Features NOT Supported

NA

## Important Usage Guidelines

NA

## Example Usage

Include the below file to access the APIs
\snippet Sdfm_sample.c include

## API

\ref DRV_SDFM_MODULE
