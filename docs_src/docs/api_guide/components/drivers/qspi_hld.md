# QSPI High Level Driver{#DRIVERS_QSPI_HLD_PAGE}

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
- In advanced config, you can choose various parameters like frame format, decoder chip select polarity, read dummy cycles, SDK Infra and QSPI Interrupt


## Features NOT Supported

- Dual and Quad writes are not supported.

## Usage Overview

### API Sequence

To use the QSPI driver to send data over the SPI bus, the application
calls the following APIs:

- #QSPI_open(): Initialize the QSPI driver.
- #QSPI_readMemMapMode() / #QSPI_readConfigModeIntr():
  Read data in Memory mapped mode, DMA mode and Interrupt.
- #QSPI_writeConfigMode() / #QSPI_writeConfigModeIntr():
  Write data in Config mode and Interrupt mode.
- #QSPI_close():  De-initialize the QSPI instance.


## Example Usage

Include the below file to access the APIs
\snippet Qspi_sample.c include

Instance Open Example
\snippet Qspi_sample.c open

Instance Close Example
\snippet Qspi_sample.c close

Blocking Write and Read Example
\snippet Qspi_sample.c transfer_blocking

Non-Blocking Write and Read Example
\snippet Qspi_sample.c transfer_nonblocking

## API

\ref DRV_QSPI_MODULE
