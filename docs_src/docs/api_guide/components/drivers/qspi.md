# QSPI {#DRIVERS_QSPI_PAGE}

[TOC]

The Quad Serial Peripheral Interface (QSPI) module is a kind of Serial Peripheral Interface (SPI) module
which allows single, dual or quad read access to external flash devices.
The QSPI module is used to transfer data, either in a memory mapped mode (for example a
processor wishing to execute code directly from external flash memory), or in an configuration mode where the
module is set-up to silently perform some requested operation, signaling its completion via interrupts or
status registers.

## Features Supported

- Support for single, dual or quad read operation.
- Programmable signal polarities
- Programmable active clock edge
- Programmable delay between chip select activation and output data from 0 to 3 QSPI clock cycles

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- QSPI instance name
- Input clock frequency to be used for QSPI module
- Input clock divider which decides the baud-rate at which the flash will be read
- Chip Select
- Enabling DMA mode
- In advanced config, you can choose various parameters like frame format, decoder chip select polarity, read dummy cycles etc


## Features NOT Supported

- Interrupt mode is not supported yet.
- Dual and Quad writes are not supported.

## Example Usage

Include the below file to access the APIs
\snippet Qspi_sample.c include

Instance Open Example
\snippet Qspi_sample.c open

Instance Close Example
\snippet Qspi_sample.c close

## API

\ref DRV_QSPI_MODULE
