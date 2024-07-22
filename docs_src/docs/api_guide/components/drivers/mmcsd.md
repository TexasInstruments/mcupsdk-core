# MMCSD {#DRIVERS_MMCSD_PAGE}

\cond SOC_AM64X
\note EMMC Flash card is not available in AM64xSK board.
\endcond

[TOC]

Multi-Media Card Secure Digital (MMCSD) peripheral is a driver which provides an interface with storage devices which follows MMC/SD/SDIO protocol. The driver supports single bit, four bit and eight bit data lines to communicate with the connected media. The MMCSD controller provides accessibility to external MMC/SD/SDIO devices using a programmed IO method or DMA data transfer method. There are two MMCSD modules inside the SOC - MMCSD0 and MMCSD1.

## Features Supported

\if SOC_AM65X
- Integrated DMA Controller supporting SD Advanced DMA - SDMA, ADMA2
\else
- Integrated DMA Controller supporting SD Advanced DMA - SDMA, ADMA2 and ADMA3
\endif
- 64-bit data, address width System Bus Interface
- MMCSD0 supports eMMC 5.1, and also backward compatible withe earlier eMMC standards
- MMCSD1 supports SD card HC 4.10 and SD Physical Layer v3.01, SDIO v3.00

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

## Features not Supported

- MMCSD0 does not support SD card, SDIO and voltages 3.3V, 3.0V, 1.2V and HS400 DDR
- MMCSD1 does not support MMC card, UHS-II SD card and SDR104

## Example Usage

Include the below file to access the APIs
\snippet Mmcsd_sample.c include

Instance Open Example
\snippet Mmcsd_sample.c open

Instance Close Example
\snippet Mmcsd_sample.c close

## API

\ref DRV_MMCSD_MODULE