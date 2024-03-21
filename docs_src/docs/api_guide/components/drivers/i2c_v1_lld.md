# I2C Low Level Driver {#DRIVERS_I2C_LLD_PAGE}

## Features Supported

- Controller and Target mode of operation
- Interrupt, Polled Mode
- Non-blocking (callback) transfers
- I2C Bus Recovery

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure below parameters apart from common configuration like Clock,MPU,RAT and others.
- I2C module configuration parmaters like bitrate, target addresses to probe.
- I2C instances and pin configurations.
- Based on above parameters, the SysConfig generated code does below as part of Drivers_open and Drivers_close functions
    - Set I2C instance parameter configuration.

## Features NOT Supported

- Target mode is not supported in polling mode.

## Usage Overview

### API Sequence

To use the I2C LLD driver to transmit/receive data over the I2C bus, probe target and set bus frequency the application
calls the following APIs:

- #I2C_lld_init(): Initialize the I2C driver.

- #I2C_lld_write() / #I2C_lld_writeIntr():
  Write data in polling and interrupt mode respectively.

- #I2C_lld_read() / #I2C_lld_readIntr():
  Read data in polling and interrupt mode respectively.

- #I2C_lld_mem_write() / #I2C_lld_mem_writeIntr():
  Write data to internal memory of i2c device in polling and interrupt mode respectively.

- #I2C_lld_mem_read() / #I2C_lld_mem_readIntr():
  Read data from the internal memory of i2c device in polling and interrupt mode respectively.

- #I2C_lld_transferPoll() / #I2C_lld_transferIntr():
  Read and Write data in polling and interrupt mode respectively.

- #I2C_lld_targetTransferIntr():
  Read and Write data as target device in interrupt mode.

- I2C_lld_probe():
  Probe I2C Target.

- I2C_lld_setBusFrequency():
  Set Bus frequency.

- #I2C_lld_deInit(): De-initialize the I2C LLD instance.

### Initializing the I2C LLD Driver

#I2C_lld_init() must be called before any other I2C APIs. This function
iterates till the channel count. This function uses i2c lld handle to initialize each instance.

Calling #I2C_lld_init() a second time with the same handle
previously passed to #I2C_lld_init() will result in an error.  You can,
though, re-use the handle if the instance is closed via #I2C_lld_deInit().

Please note that initializing I2C LLD driver is taken care by the
SysConfig generated code.

### I2C Transfer Mode

The I2C driver supports two transfer modes of operation: Polling and Interrupt Mode.

In Polling mode transfer can be initiated with all the polling mode APIs

In Interrupt mode Users have to register the appropriate ISR and assign a callback function before calling the interrupt mode APIs

## Example Usage

Include the below file to access the APIs
\snippet I2c_lld_sample_v1.c include

Instance Open Example
\snippet I2c_lld_sample_v1.c open

Instance Close Example
\snippet I2c_lld_sample_v1.c close

Non-Blocking Transfer Example
\snippet I2c_lld_sample_v1.c transfer_nonblocking

Non-Blocking Example transfer callback
\snippet I2c_lld_sample_v1.c transfer_callback

Non-Blocking Example ISR CALL callback
\snippet I2c_lld_sample_v1.c isr_call

## API

\ref DRV_I2C_LLD_MODULE
