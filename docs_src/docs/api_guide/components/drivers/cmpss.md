# CMPSS {#DRIVERS_CMPSS_PAGE}

[TOC]

The CMPSS driver provides API to configure the CMPSS module.
Below are the high level features supported by the CMPSS driver.

## Features Supported

- Each instance has 2 analog comparators, 2 programmable reference 12-bit DACs and a ramp generator.
- Ability to generate trip signals for ePWM and for external signalling.
- Ramp generator to produce a falling ramp input for the high reference 12-bit DAC when selected.
- Ability to latch and invert output and also option to use hysteresis on output.
- Option to choose VDDA or VDAC to be the DAC reference voltage.
- Ability to synchronize submodules with EPWMSYNCPER.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Selection of CMPSS instance.
- Configuring reference voltage and value source of DAC in the CMPSS module.
- Configuring the low and high comparators by selecting negative input source, setting comparator DAC value and other options.

## Features NOT Supported

NA

## Important Usage Guidelines

NA

## Example Usage

Include the below file to access the APIs
\snippet Cmpss_sample.c include

## API

\ref DRV_CMPSS_MODULE
