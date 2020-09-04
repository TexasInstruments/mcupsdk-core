# EPWM {#DRIVERS_EPWM_PAGE}

[TOC]

The EPWM driver provides API to configure the EPWM module.
Below are the high level features supported by the EPWM driver.

## Features Supported

- Scaling the time-base clock relative to ePWM clock using TimeBase submodule.
- Count-up, Count-down and Count-up-and-down modes for time-base counter.
- Configuring time-base counter behaviour during emulation.
- Synchronizing time-base counter of different ePWM instances.
- Specifying the PWM duty cycle for output ePWMxA and ePWMxB using Counter Compare submodule.
- Specifying actions taken on different events using Action Qualifier submodule.
- Configuring rising-edge-delay and falling-edge-delay using Deadband submodule.
- Creating chopping (carrier) frequency using Chopper submodule.
- Configuring ePWM to react to one, all or none of the trip zone signals or digital comapre events.
- Triggering an ADC Start of conversion using Event trigger submodule.
- Specifying rate at which events cause trigger using Event trigger submodule.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Selecting the group of the EPWM instance.
- Specifying the EPWM clock prescaler by selecting Time-Base clock divider and High Speed clock divider.
- Specifying the behaviour of the Time Base Counter on an emulation event.
- Specifying the Time Base period.
- Selecting Time Base period load mode and event.
- Specifying Time Base Counter Mode.
- Configuring Counter Compare value and selecting counter compare shadow load event.
- Specifying action for various events in the Action Qualifier module.
- Specifying actions for various trip zone events.
- Configuring Digital Compare events.
- Configuring EPWM Dead Band module by specifying values and polarity of Rising Edge Delay and Falling Edge Delay.
- Configuring the chopper module by specifying duty cycle and frequency.
- Enabling and configuring interrupts in the EPWM event trigger submodule.

## Features NOT Supported

- APIs for OTTOCAL module.

## Important Usage Guidelines

NA

## Example Usage

Include the below file to access the APIs
\snippet Epwm_sample.c include

## API

\ref DRV_EPWM_MODULE
