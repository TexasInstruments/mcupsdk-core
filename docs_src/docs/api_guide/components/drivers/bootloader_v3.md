# BOOTLOADER {#DRIVERS_BOOTLOADER_PAGE}

[TOC]

The Bootloader module provides APIs to write bootloader applications for various boot media like OSPI, UART, SOC memory etc.

## Features Supported

- OSPI Boot
- MEM Boot (Boot media is SOC memory)
- API to parse multicore appimage
- Separate APIs to boot self and non-self cores

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Bootloader instance name
- Boot Media to be used
- Boot Image offset

## R5 Dual Core Support

RBL boots the R5 in eFUSE default, which is **split mode**. SBL (Secondary Boot Loader) follows the same and keeps the R5s in split mode. As of now the lock step configuration of dual R5 is not supported from bootloader.

## Example Usage

Include the below file to access the APIs
\snippet Bootloader_sample.c include

Instance Open Example
\snippet Bootloader_sample.c open

Booting Cores Example
\if SOC_AM65X
\snippet Bootloader_sample.c bootcores_am65x
\else
\snippet Bootloader_sample.c bootcores
\endif

Instance Close Example
\snippet Bootloader_sample.c close

## API

\ref DRV_BOOTLOADER_MODULE
