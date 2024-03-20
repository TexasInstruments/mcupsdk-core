# QSPI Low Level Driver{#DRIVERS_QSPI_LLD_PAGE}

[TOC]

## Features Supported

- Support for Single, Dual or Quad read operation.
- Support for Interrupt mode.
- Support for DMA read.
- Programmable Signal Polarities
- Programmable Active Clock Edge
- Programmable Delay between chip select activation and output data from 0 to 3 QSPI clock cycles

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- QSPI instance name
- Input clock frequency to be used for QSPI module
- Input clock divider which decides the baud-rate at which the flash will be read
- Chip Select
- Enabling DMA mode
- In advanced config, you can choose various parameters like frame format, decoder chip select polarity, read dummy cycles, SDK Infra and QSPI Interrupt.


## Features NOT Supported

- Dual and Quad writes are not supported.

## Usage Overview

### API Sequence

To use the QSPI driver to send data over the SPI bus, the application
calls the following APIs:

- #QSPI_lld_init(): Initialize the QSPI driver.
- #QSPI_lld_read() / #QSPI_lld_readCmdIntr() / #QSPI_lld_readDma():
  Read data in Polling, Interrupt, and DMA mode respectively.
- #QSPI_lld_writeCmd / #QSPI_lld_writeCmdIntr():
  Write data in Config mode and Interrupt mode.
- #QSPI_lld_deInit():  De-initialize the QSPI instance.

## Example Usage

Include the below file to access the APIs
\snippet Qspi_lld_sample.c include

Instance Open Example
\snippet Qspi_lld_sample.c open

Instance Close Example
\snippet Qspi_lld_sample.c close

Blocking Read Example
\snippet Qspi_lld_sample.c blocking_writeRead

Non-Blocking Example ISR Register
\snippet Qspi_lld_sample.c isr_call

Non-Blocking Transfer Example
\snippet Qspi_lld_sample.c transfer_nonblocking

Non-Blocking Example ISR callback
\snippet Qspi_lld_sample.c transfer_nonblocking_callback

## API

\ref DRV_QSPI_LLD_MODULE
