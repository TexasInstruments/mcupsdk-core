# EDMA {#DRIVERS_EDMA_PAGE}

[TOC]

EDMA driver provides API to perform DMA tranfers.

## Features Supported

- DMA, QDMA channels
- Interrupt and Polled mode of operations
- PaRAM Linking
- Channel Chaining
- Manual and Event triggered transfers

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- EDMA instances selection
- Region Id selection
- Interrupt mode selection
- DMA Channel management
- QDMA Channel management
- TCC management
- PaRAM management


## Features NOT Supported

- None

## Usage Overview

### API Sequence

To transfer the data using edma, application calls following APIs

- #EDMA_init(): Initialize the EDMA driver.
- #EDMA_open(): Open an instance of the EDMA driver
- #EDMA_getBaseAddr(): Get the edma base address corresponding to the driver
  handle. This is used in further APIs for initiating the transfers.
- #EDMA_configureChannelRegion(): Configure the DMA channel used for transfer
- #EDMA_setPaRAM(): Configure the PaRAM corresponding to the transfer
- #EDMA_enableTransferRegion(): Enable the data transfer
- #EDMA_close():  Closes the EDMA instance.
- #EDMA_deinit(): De-Initialize the EDMA driver.

## Important Usage Guidelines

- None

## Example Usage

Include the below file to access the APIs
\snippet Edma_sample.c include

Instance Open Example
\snippet Edma_sample.c open

Instance Close Example
\snippet Edma_sample.c close

Instance transfer Example
\snippet Edma_sample.c transfer

## API

\ref DRV_EDMA_MODULE
