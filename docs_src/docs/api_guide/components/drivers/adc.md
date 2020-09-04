# ADC {#DRIVERS_ADC_PAGE}

[TOC]

The ADC driver provides API to configure the ADC module.
Below are the high level features supported by the ADC driver.

## Features Supported

- Configuration of parameters: mode, open delay etc
- Configuration of  FIFO threshold levels
- Configuration of range of data for the ADC conversion result
- Generation of interrupt when the the conversion result exceeds the range
- Programmable averaging of input samples. The number of samples taken for averaging can be configured
- Support for getting the number of words present in the FIFO and also reading the FIFO data
- Support for enabling/disabling various interrupts
- Get ADC interrupt status register to determine the source of interrupt

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Selection of input channels.

## Features NOT Supported

- Big Endian

## Important Usage Guidelines

- Global initialization of surrounding modules must be done after a device reset (Please refer TRM)
- A minimum wait for 4 us is required before starting a conversion to provide time for AFE to power up. The user needs to ensure this.

## Example Usage

Include the below file to access the APIs
\snippet Adc_sample.c include

Initialize ADC
\snippet Adc_sample.c init_adc

Get FIFO threshold level
\snippet Adc_sample.c get_fifo_threshold

Check if ADC is powered up or not
\snippet Adc_sample.c adc_powered_up

## API

\ref DRV_ADC_MODULE
