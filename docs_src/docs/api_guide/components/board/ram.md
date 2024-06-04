# RAM {#BOARD_RAM_PAGE}

[TOC]

The Ram driver provides API to read and write to parallel or serial RAM devices interfaced with the board.
The driver takes care of configuring the specific interface necessary to interact with RAM device.

## Features Supported

- APIs to read and write to a RAM offset.
- Supports 16-bit parallel pSRAM devices.

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Option to select RAM topology based on board
\cond SOC_AM64X || SOC_AM243X
- Supported RAM devices
    - IS67WVE4M16EBLL70BLA1
\endcond
\cond SOC_AM263X
- Supported RAM devices
    - IS67WVE4M16EBLL70BLA1
\endcond

## Features NOT Supported

- Serial RAM devices
- Burst read & write is not supported
- DMA support

## Important Usage Guidelines

None

## Example Usage

Include the below file to access the APIs
\snippet Ram_sample.c include

RAM Read API
\snippet Ram_sample.c read

RAM Write API
\snippet Ram_sample.c write


## API

\ref BOARD_RAM_MODULE