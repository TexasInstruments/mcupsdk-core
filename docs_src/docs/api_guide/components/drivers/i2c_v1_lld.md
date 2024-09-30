# I2C Low Level Driver {#DRIVERS_I2C_LLD_PAGE}

## Features Supported

- Controller and Target mode of Operation.
- Interrupt, Polled Mode.
- Non-blocking (Callback) Transfer.
- I2C Bus Recovery.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure below parameters apart from common configuration like Clock,MPU,RAT and others.
- I2C module configuration parameters like bitrate, Own target address.
- I2C instances and pin configurations.
- Based on above parameters, the SysConfig generated code does below as part of Drivers_open and Drivers_close functions
    - Set I2C instance parameter configuration.

## Features NOT Supported

- DMA Mode Operation.

## Usage Overview

### API Sequence

To use the I2C LLD driver to transmit/receive data over the I2C bus, probe target and set bus frequency the application may call the following APIs:

- #I2C_lld_init(): Initialize the I2C LLD Instance.

- #I2C_lld_deInit(): De-initialize the I2C LLD instance.

- #I2C_lld_Transaction_init(): Fill Default Value in Transaction object.

- #I2C_lld_Message_init(): Fill Default Value in Message object.

- #I2C_lld_write() / #I2C_lld_writeIntr():
  Write data as Controller in polling and interrupt mode respectively.

- #I2C_lld_target_write() / #I2C_lld_target_writeIntr():
  Write data as Target in polling and interrupt mode respectively.

- #I2C_lld_read() / #I2C_lld_readIntr():
  Read data as Controller in polling and interrupt mode respectively.

- #I2C_lld_target_read() / #I2C_lld_target_readIntr():
  Read data as Target in polling and interrupt mode respectively.

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

### Initializing the I2C LLD Driver

#I2C_lld_init() must be called before any other I2C APIs.
This function uses I2C LLD handle to initialize each instance.

Calling #I2C_lld_init() a second time with the same handle
previously passed to #I2C_lld_init() will result in an error.
Though, handle can be re-used the instance is closed using #I2C_lld_deInit().

Please note that the initialization of I2C LLD instances is taken care by the
SysConfig generated code.

### I2C Transfer Mode

The I2C driver supports two transfer modes of operation: Polling and Interrupt Mode.

In Polling mode transfer can be initiated with all the polling mode APIs.

In Interrupt mode Users have to register the appropriate ISR and assign a Callback function before calling the interrupt mode APIs.

## Example Usage

Include the below files to access the APIs
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
