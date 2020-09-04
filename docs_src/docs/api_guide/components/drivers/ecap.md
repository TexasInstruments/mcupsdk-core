# ECAP {#DRIVERS_ECAP_PAGE}

[TOC]

The eCAP driver provides API to configure eCAP module.
Below are the high level features supported by the driver.

## Features Supported

- Configuration of timestamp capture event
- Configuration of one-shot/continuous mode
- Configuration of independent edge polarity (rising / falling edge) selection for all 4 capture events
- Configuration of interrupt on any of the 4 capture events
- Configuration of input signal prescaling
- Configuration of mode to aPWM when not used in capture mode

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure below parameters apart from common configuration like Clock, MPU, RAT and others.
- ECAP instances and pin mux configurations.

## Features NOT Supported

NA

## Important Usage Guidelines

NA

## Example Usage

Include the below file to access the APIs
\snippet Ecap_sample.c include

ECAP Interrupt Registration
\snippet Ecap_sample.c App_ecapIntrReg

ECAP Operating Mode Configuration
\snippet Ecap_sample.c App_ecapConfigOperMode

ECAP One Shot Mode Configuration
\snippet Ecap_sample.c App_ecapConfigOneShotMode

ECAP Capture Event Polarity Configuration
\snippet Ecap_sample.c App_ecapCaptureEvtPolarityConfig

ECAP Interrupt Service Routine
\snippet Ecap_sample.c App_ecapIntrISR

ECAP Interrupt De-Registration
\snippet Ecap_sample.c App_ecapIntrDeReg

## API

\ref DRV_ECAP_MODULE
