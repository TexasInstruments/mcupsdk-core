# SOC {#DRIVERS_SOC_PAGE}

[TOC]

The SOC driver provides API to configure SOC specific features like clocks.

## Features Supported

- API to enable/disable a module clock
- API to set clock frequency for module
- Generic utility API like get CPU clock, get core name string, control module MMR lock/unlock API
- API's to configure all the SOC and Time Sync XBAR's
- Address Translation APIs for all the R5F cores
- SOC Software Warm Reset APIs. Please refer "Reset" chapter in techical reference manual for more details.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- SOC driver is integrated with each of the drivers. User need not perform any explicit configuration for this driver
- The Input for every XBAR output instance can be configured through SysConfig
  (Input-Signals going into the XBAR, Instance-Each output signal from the XBAR)
- Refer to the following examples on how to configure XBAR through SysConfig - \ref EXAMPLES_DRIVERS_EPWM_HR_DUTY_CYCLE, \ref EXAMPLES_DRIVERS_ECAP_APWM_MODE, \ref EXAMPLES_DRIVERS_EQEP_FREQUENCY_MEASUREMENT
- XBAR outputs routed to other XBAR's get automatically configured when
input is selected for the end XBAR

\imageStyle{xbar_config_sample.png,width:70%}
\image html xbar_config_sample.png "Sample XBAR Configuration"

- XBAR configuration also possible from the SysConfig module to which it is connected. For example, DMA Trigger XBAR can be configured from EDMA module.

## Features NOT Supported

NA

## Important Usage Guidelines

- Most of these APIs are already integrated with SysConfig tool and the generated code
does the required call to enable a module, set the required clock and so on.
User need to use these APIs in their application only for exceptional scenarios.

## Example Usage

Include the below file to access the APIs
\snippet Soc_am263x_sample.c include

Get Core Name String
\snippet Soc_am263x_sample.c get_corename

Get CPU Clock Frequency
\snippet Soc_am263x_sample.c get_selfcpuclk

## API

\ref DRV_SOC_MODULE

\ref DRV_SOC_RCM_MODULE

\ref DRV_SOC_XBAR_MODULE
