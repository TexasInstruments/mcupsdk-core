# EPWM {#DRIVERS_EPWM_PAGE}

[TOC]

The ePWM driver provides API to configure various sub-modules within the ePWM module.
Below are the high level features supported by the driver.

## Features Supported

- Configuration of Time Base submodule to set time-base clock, counter mode,
- Configuration of Counter Compare submodule to specify duty cycle
- Configuration of Action Qualifier submodule to specify the type of action to take when time-base or counter-compare event occurs
- Configuration of Dead Band submodule to set rising-edge and falling-edge delay or bypass the module
- Configuration of Chopper submodule to generate a chopper frequency, set pulse width of the first pulse in the chopped pulse train or bypass the module
- Configuration of Trip Zone submodule to specify the tripping action to take when a fault occurs
- Configuration of event-trigger submodule to trigger an interrupt and rate at which these events occur

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Selection of output channels.
- Configuring pinmux based on selected pin.
- Enabling TimeBase Clock based on selected ePWM instance.

## Features NOT Supported

- ePWM digital comparator modules
- ePWM high-resolution modules

## Important Usage Guidelines

NA

## Example Usage

Include the below file to access the APIs
\snippet Epwm_sample.c include

Check external Synchronization signal
\snippet Epwm_sample.c check_ext_sync

Get Timebase Counter direction
\snippet Epwm_sample.c get_timebase_direction

Configure Counter compare
\snippet Epwm_sample.c config_counter_compare

## API

\ref DRV_EPWM_MODULE
