# GPADC {#DRIVERS_GPADC_PAGE}

[TOC]

The GPADC driver provides API for safety monitoring of the inputs, such as the temperature sensor, voltage regulators to the device.

## Features Supported

- 10-bit ADC
- Supports Inter Frame Monitoring Mode(IFM)
- Software command mechanism to trigger the conversion
– Storage of min, max, and sum of the samples captured per channel

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Selection of Channel read mode
- Set Channel index number to read from
- Set Channel Bitmap
- Option to set the conversion mode
- Set Channel Configuration values

## Features NOT Supported

 Continous Time Monitoring Mode(CTM) 

## Example Usage

Include the below file to access the APIs
\snippet Gpadc_sample.c include

GPADC Single Buffer Read
\snippet Gpadc_sample.c singleBufferRead

GPADC Group Buffer Read
\snippet Gpadc_sample.c groupBufferRead

GPADC Temperature Sensor Read
\snippet Gpadc_sample.c tempSensorRead

## API

\ref DRV_GPADC_MODULE
