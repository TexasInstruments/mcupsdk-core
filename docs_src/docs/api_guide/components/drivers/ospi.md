# OSPI {#DRIVERS_OSPI_PAGE}

[TOC]

The Octal Serial Peripheral Interface (OSPI) module is a kind of Serial Peripheral Interface (SPI) module
which allows single, dual, quad or octal read and write access to external flash devices.
The OSPI module is used to transfer data, either in a memory mapped direct mode (for example a
processor wishing to execute code directly from external flash memory), or in an indirect mode where the
module is set-up to silently perform some requested operation, signaling its completion via interrupts or
status registers.

## Features Supported

- Support for single, dual, quad (QSPI mode) or octal I/O instructions.
- Supports dual Quad-SPI mode for fast boot applications.
- Memory mapped ‘direct’ mode of operation for performing flash data transfers and executing code from flash memory.
- Programmable delays between transactions.
- Legacy mode allowing software direct access to low level transmit and receive FIFOs, bypassing the higher layer processes.
- An independent reference clock to decouple bus clock from SPI clock – allows slow system clocks.
- Programmable baud rate generator to generate OSPI clocks.
- Supports BOOT mode.
- Handling ECC errors for flash devices with embedded correction engine.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- OSPI instance name
- Input clock frequency to be used for OSPI module
- Input clock divider which decides the baud-rate at which the flash will be read
- Chip Select
- Enabling of various features like DMA, PHY mode(not supported yet), XIP(not supported yet)
- In advanced config, you can choose various parameters like frame format, decoder chip select, read dummy cycles etc.
- Pinmux configurations for the OSPI instance


## Features not Supported

- PHY mode is not supported yet.
- Interrupt mode is not supported yet.

## Example Usage

Include the below file to access the APIs
\snippet Ospi_sample.c include

Instance Open Example
\snippet Ospi_sample.c open

Instance Close Example
\snippet Ospi_sample.c close

## API

\ref DRV_OSPI_MODULE