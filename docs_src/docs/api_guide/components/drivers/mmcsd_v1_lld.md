# MMCSD Low Level Driver {#DRIVERS_MMCSD_V1_LLD_PAGE}

\cond SOC_AM263X
\note EMMC Flash card is not available in AM263X board.
\endcond

\cond SOC_AM263PX
\note EMMC Flash card is not available in AM263PX board.
\endcond

\cond SOC_AM261X
\note EMMC Flash card is not available in AM261X board.
\endcond

[TOC]

Multi-Media Card Secure Digital (MMCSD) peripheral is a driver which provides an interface with storage devices which follows MMC/SD/SDIO protocol. The driver supports single bit and four bit data lines to communicate with the connected media. The MMCSD controller provides accessibility to external MMC/SD/SDIO devices using a programmed IO method. There is only one MMCSD module inside the SOC - MMCSD0.

## Features Supported

- 64-bit data, address width System Bus Interface
- SD command/response sets as defined in the SD Physical Layer specification v2.00
- Operating voltage range VCC = 3.3 V
- Default-speed & High-speed MMCSD protocol.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

## Features not Supported

- No integrated DMA Controller Support.
- MMCSD instance does not support DDR mode.

## Usage Overview

### API Sequence

To use the MMCSD LLD driver to write/read data to or from a MMC/SD Device, the application
calls the following APIs:

- #MMCSD_lld_init(): Initialize the MMCSD driver.

- #MMCSD_lld_deInit(): De-Initialize the MMCSD driver.

- #MMCSD_lld_getBlockSize(): Get the max block Supported by the Device.

- #MMCSD_lld_write_SD_Poll() / #MMCSD_lld_write_SD_Intr():
  Write data to SD device in polling and interrupt mode respectively.

- #MMCSD_lld_read_SD_Poll() / #MMCSD_lld_read_SD_Intr():
  Read data from SD device in polling and interrupt mode respectively.

### Initializing the MMCSD LLD Driver

#MMCSD_lld_init() must be called before any other MMCSD APIs. This function uses mmcsd lld handle to initialize each instance.

Calling #MMCSD_lld_init() a second time with the same handle
previously passed to #MMCSD_lld_init() will result in an error.  You can,
though, re-use the handle if the instance is closed via #MMCSD_lld_deInit().

Please note that initializing MMCSD LLD driver is taken care by the
SysConfig generated code.

### MMCSD Transfer Mode

The MMCSD driver supports two transfer modes of operation: Polling and Interrupt Mode.

In Polling mode transfer can be initiated with all the polling mode APIs.

In Interrupt mode Users have to register the appropriate ISR and assign a callback function before calling the interrupt mode APIs.

DMA can enabled in any mode of operation if enabled.

## Example Usage

Include the below file to access the APIs
\snippet Mmcsd_lld_v1_sample.c include

Instance Open Example
\snippet Mmcsd_lld_v1_sample.c open

Instance Close Example
\snippet Mmcsd_lld_v1_sample.c close

Non-Blocking Transfer Example
\snippet Mmcsd_lld_v1_sample.c transfer_nonblocking

Non-Blocking Example transfer callback
\snippet Mmcsd_lld_v1_sample.c transfer_callback

Non-Blocking Example ISR CALL callback
\snippet Mmcsd_lld_v1_sample.c isr_call

## API

\ref DRV_MMCSD_LLD_MODULE