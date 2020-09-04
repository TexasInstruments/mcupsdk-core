# EQEP {#DRIVERS_EQEP_PAGE}

[TOC]

The eQEP driver provides API to configure eQEP module.
Below are the high level features supported by the driver.

## Features Supported

- Configuration of position counter and control unit for position and direction measurement
- Configuration of unit time base for speed and frequency measurement
- Configuration of quadrature edge capture unit for low-speed measurement
- Configuration of watchdog timer for detecting stalls

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure below parameters apart from common configuration like Clock, MPU, RAT and others.
- EQEP instances and pin mux configurations.

## Features NOT Supported

NA

## Important Usage Guidelines

NA

## Example Usage

Include the below file to access the APIs
\snippet Eqep_sample.c include

EQEP Interrupt Registration
\snippet Eqep_sample.c App_eqepIntrReg

EQEP Quadrature Mode Position Measurement Configuration
\snippet Eqep_sample.c App_eqepConfigPositionMeasurementMode

EQEP Quadrature Mode Frequency Measurement Configuration
\snippet Eqep_sample.c App_eqepConfigFrequencyMeasurementMode

EQEP Interrupt Service Routine
\snippet Eqep_sample.c App_eqepIntrISR

EQEP Interrupt De-Registration
\snippet Eqep_sample.c App_eqepIntrDeReg

## API

\ref DRV_EQEP_MODULE
