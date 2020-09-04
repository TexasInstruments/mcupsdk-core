# WATCHDOG {#DRIVERS_WATCHDOG_PAGE}

[TOC]

The WATCHDOG driver provides API for safety diagnostic which can detect a runaway CPU and
generate either a reset or NMI (non-maskable interrupt) response. It generates resets or NMIs after a
programmable period, or if no correct key sequence was written to the RTIWDKEY register.


## Features Supported
\cond SOC_AM273X || SOC_AWR294X || SOC_AM263X
- Supports Watchdog reset mode.
\endcond
\cond SOC_AM64X || SOC_AM243X
- Supports Watchdog interrupt mode.
\endcond
- Supports digital windowed Watchdog feature.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Selection of Watchdog instances.
- Option to select Watchdog Window size.
- Option to select Watchdog expiry time in millisecond.

## Features NOT Supported
\cond SOC_AM263X
- As ESM module not integrated in sdk still, Watchdog interrupt mode is not supported.
\endcond
\cond SOC_AM273X || SOC_AWR294X
- NA
\endcond

## Important Usage Guidelines

- Important Usage Guildelines

## Example Usage

Include the below file to access the APIs
\snippet Watchdog_sample.c include

Instance Open Example
\snippet Watchdog_sample.c open

Instance Close Example
\snippet Watchdog_sample.c close

Watchdog Service Example
\snippet Watchdog_sample.c Watchdog_service

## API

\ref DRV_WDT_MODULE
