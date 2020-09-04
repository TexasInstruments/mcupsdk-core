# DAC {#DRIVERS_DAC_PAGE}

[TOC]

The DAC driver provides API to configure the DAC module.
Below are the high level features supported by the DAC driver.

## Features Supported

- 12-bit resolution.
- Selectable reference voltage source.
- Ability to synchronize with EPWMSYNCPER.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Selecting reference voltage for DAC.
- Selecting the load mode for DAC.
- Specifying the shadow value for DAC.

## Features NOT Supported

NA

## Important Usage Guidelines

NA

## Example Usage

Include the below file to access the APIs
\snippet Dac_sample.c include

## API

\ref DRV_DAC_MODULE
