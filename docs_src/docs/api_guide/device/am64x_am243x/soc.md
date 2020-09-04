# SOC {#DRIVERS_SOC_PAGE}

[TOC]

The SOC driver provides API to configure SOC specific features like clocks.

## Features Supported

- API to enable/disable a module clock
- API to set clock frequency for module
- Generic utility API like get CPU clock, get core name string, control module MMR lock/unlock API
- Address Translation APIs for all the R5F cores
- SOC Software Warm Reset/POR Reset APIs

## SysConfig Features

- SOC driver is integrated with each of the drivers. User need not perform any explicit configuration for this driver

## Features NOT Supported

NA

## Important Usage Guidelines

- Most of these APIs are already integrated with SysConfig tool and the generated code
does the required call to enable a module, set the required clock and so on.
User need to use these APIs in their application only for exceptional scenarios.

\cond SOC_AM64X
\note The \ref SOC_controlModuleLockMMR API will be ineffective in the case of AM64x SOC, since AM64x is mostly used in conjunction with Linux, and the linux kernel assumes MMRs to be unlocked. This is protected with a `#if` statement in the SOC driver. Please remove this condition if the SDK is used without linux.
\endcond

- All MCU domain resets act as a main reset to the whole device. (MAIN and MCU domains)

- MAIN domain resets will only reset the MAIN domain.

- Please refer "Reset" chapter in techical reference manual for more details.

## Example Usage

Include the below file to access the APIs
\snippet Soc_am64x_sample.c include

Get Core Name String
\snippet Soc_am64x_sample.c get_corename

Get CPU Clock Frequency
\snippet Soc_am64x_sample.c get_selfcpuclk

## API

\ref DRV_SOC_MODULE
