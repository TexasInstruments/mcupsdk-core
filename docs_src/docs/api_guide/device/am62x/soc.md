# SOC {#DRIVERS_SOC_PAGE}

[TOC]

The SOC driver provides API to configure SOC specific features like clocks.

## Features Supported

- API to enable/disable a module clock
- API to set clock frequency for module
- Generic utility API like get CPU clock, get core name string, control module MMR lock/unlock API

## SysConfig Features

- SOC driver is integrated with each of the drivers. User need not perform any explicit configuration for this driver

## Features NOT Supported

NA

## Important Usage Guidelines

- Most of these APIs are already integrated with SysConfig tool and the generated code
does the required call to enable a module, set the required clock and so on.
User need to use these APIs in their application only for exceptional scenarios.

## Example Usage

Include the below file to access the APIs
\snippet Soc_am64x_sample.c include

Get Core Name String
\snippet Soc_am64x_sample.c get_corename

Get CPU Clock Frequency
\snippet Soc_am64x_sample.c get_selfcpuclk

## API

\ref DRV_SOC_MODULE
