# ADC {#DRIVERS_ADC_PAGE}

[TOC]

The ADC driver provides API to configure the ADC module.
Below are the high level features supported by the ADC driver.

## Features Supported

- Each ADC has 6 selectable single ended or 3 selectable differential inputs.
- 12-bit resolution with 4 MSPS sampling rate max.
- 16 configurable SOCs and 16 individually addressable result registers for each SOC.
- Multiple trigger sources - software, all ePWMs ADCSOC A or B, ADCINT1/2, etc.
- Burst mode.
- Four post processing blocks each with saturating offset calibration, error from setpoint calculation, high, low or zero crossing compare, trigger to sample delay capture.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Selecting ADC Clock Prescaler.
- Selecting ADC Signal Mode - Single ended or Differential.
- Specifying channels, triggers and sample window size for each ADC SOC.
- Enabling and configuring ADC interrupts.
- Configuring Post Processing Blocks.
- Specifying Burst mode trigger and size.

## Features NOT Supported

NA

## Important Usage Guidelines

- ADC has a power up time of 500us. Therefore this delay must be allowed between powering up the ADC and starting the sampling of the signal. Sysconfig already handles this, but if ADC is powered up manually by using the API, then this power up delay must be added.

## Example Usage

Include the below file to access the APIs
\snippet Adc_sample.c include

## API

\ref DRV_ADC_MODULE
