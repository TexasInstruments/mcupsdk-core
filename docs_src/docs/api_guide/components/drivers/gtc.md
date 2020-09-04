# GTC {#DRIVERS_GTC_PAGE}

[TOC]

The Global Timebase Counter (GTC) driver provides API to enable GTC and to set FID for GTC.

## Features Supported

- Enables GTC
- Sets FID(Frequency indicator) for GTC
- Get counter value

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Enable GTC

## Features NOT Supported

NA

## Example Usage

Include the below file to access the APIs
\snippet Gtc_sample.c include

Enable GTC
\snippet Gtc_sample.c enableGTC

## API

\ref DRV_GTC_MODULE