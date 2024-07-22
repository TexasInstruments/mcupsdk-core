# MMCSD {#DRIVERS_MMCSD_PAGE}

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\note EMMC Flash card is not available in AM263x board.
\endcond

[TOC]

Multi-Media Card Secure Digital (MMCSD) peripheral is a driver which provides an interface with storage devices which follows MMC/SD/SDIO protocol. The driver supports single bit, four bit data lines to communicate with the connected media. The MMCSD controller provides accessibility to external MMC/SD/SDIO devices using a programmed IO method. There is only one MMCSD module inside the SOC - MMCSD0.

## Features Supported

- 64-bit data, address width System Bus Interface
- SD command/response sets as defined in the SD Physical Layer specification v2.00
- Operating voltage range VCC = 3.3 V
- High-speed MMCSD protocol, Clock frequency : 0-200MHz

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

## Features not Supported

- No integrated DMA Controller support
- MMCSD instance does not support DDR mode

## Example Usage

Include the below file to access the APIs
\snippet Mmcsd_sample.c include

Instance Open Example
\snippet Mmcsd_sample.c open

Instance Close Example
\snippet Mmcsd_sample.c close

## API

\ref DRV_MMCSD_MODULE