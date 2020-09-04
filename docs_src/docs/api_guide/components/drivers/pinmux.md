# PINMUX {#DRIVERS_PINMUX_PAGE}

[TOC]

The Pinmux driver provides API to cofigure the pinmux for a particular device

## Features Supported

- API to set pinmux for various domains present in the device

## SysConfig Features

- Pinmux driver is integrated with each of the peripheral drivers. User need not perform any explicit configuration for this driver

## Features NOT Supported

NA

## Important Usage Guidelines

In the \ref Pinmux_config API, the last entry should be initialized with #PINMUX_END so that this function knows the end of configuration.

## Example Usage

Include the below file to access the APIs
\snippet Pinmux_sample.c include

Pinmux configuration
\snippet Pinmux_sample.c config

## API

\ref DRV_PINMUX_MODULE
