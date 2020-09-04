# I2C {#DRIVERS_I2C_PAGE}

[TOC]

I2C module provides an interface to any I2C bus compatible device
accessible via I2C serial bus. External components attached to I2C bus
can serially transmit/receive data to/from the CPU through two wire interface.
I2C driver provides API to perform transmit/receive to any of the I2C peripherals on the board, with the multiple modes of operation.

## Features Supported

- Controller and Target mode of operation
- Interrupt, Polled Mode
- Blocking and Non-blocking (callback) transfers
- Queueing of I2C transactions
- I2C Bus Recovery

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure below parameters apart from common configuration like Clock,MPU,RAT and others.
- I2C module configuration parmaters like bitrate, target addresses to probe.
- I2C instances and pin configurations.
- Interrupt mode enable option.If you disable it, configures to polling mode.
- Based on above parameters, the SysConfig generated code does below as part of Drivers_open and Drivers_close functions
    - Set I2C instance parameter configuration.
    - Driver ISR registration if Interrupt Mode is enabled.

## Features NOT Supported

- Target mode is not supported in polling mode.

## Important Usage Guidelines

NA

## Example Usage

Include the below file to access the APIs
\snippet I2c_sample.c include

Instance Open Example
\snippet I2c_sample.c open

Instance Close Example
\snippet I2c_sample.c close

I2c Transfer Example
\snippet I2c_sample.c i2c_transfer_blocking

## API

\ref DRV_I2C_MODULE
