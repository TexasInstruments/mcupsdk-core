# UDMA {#DRIVERS_UDMA_PAGE}

[TOC]

The UDMA driver provides API to program the DMA module of the DMSS subsystem
to setup and initiate DMA transfers.

## Features Supported

- Supports both BCDMA and Packet DMA instances
- Supports all DMA operations from both instances from all the cores in the SOC except M4F core
- UDMA block copy for memory to memory transfer
- PDMA module to initiate transfers to/from PDMA peripherals like UART, McASP, McSPI, ADC, MCAN
- DMA transfer to/from from native PSIL peripherals like CPSW, SA2UL
- Event and interrupt management like DMA completion, channel chaining, interrupt sharing using Interrupt Aggregator (IA)
- Resources management across instances and cores for DMA channels, RX flow, Interrupt Aggregator (IA), Interrupt Routers (IR), Global events, Ring Accelerator (RA)
- Interaction with DMSC RM module via SCICLIENT for all non-RealTime (NRT) configuration

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Selection of UDMA instances - BCDMA, PKTDMA
- Option to skip default global event registration done as part of \ref Udma_init API
- Option to provide user function for virtual to physical and physical to virtual address translation
- Ability of add and configure BCDMA block copy channels
    - Ability to enable interrupt mode for the channel
    - Ability to specify the number of ring entries for the channel
    - Based on above parameters, the SysConfig generated code does below as part of Drivers_open and Drivers_close functions
        - Channel open/close - the handle can be retrieved by the application using g<User_Config_Name>BlkCopyChHandle global variable
        - Set default channel configuration
        - Allocates required ring memories and pass them to channel configuration
        - Register user specified callback when interrupt mode is enabled

## Features NOT Supported

- UDMA driver is not supported for M4F core as the DMSS is present only in the main domain

## Important Usage Guidelines

-  UDMA driver doesn't manage/allocate the descriptor and RA memory. The caller need to allocate and provide the required memory.
-  UDMA driver doesn't use any global variables. All the required object memory like channel, driver instance, event etc should be allocated by the caller

## DMSS Overview

The primary goal of the Data Movement Subsystem (DMSS) is to ensure that data can be efficiently transferred from a producer to a consumer so that the real time requirements of the system can be met.
The Data Movement architecture aims to facilitate Direct Memory Access (DMA) and to provide a consistent Application Programming Interface (API) to the host software.
Data movement tasks are commonly offloaded from the host processor to peripheral hardware to increase system performance.
Significant performance gains may result from careful design of the interface between the host software and the underlying acceleration hardware.
In networking applications packet transmission and reception are critical tasks.
In general purpose compute, ping pong buffer pre-fetch and store are critical tasks as are general misaligned block copy operations.

The block diagram provides a high level picture of not only the 2 different interconnect fabrics but also some key standard data movement components that have been defined and placed in the various parts of the low cost compliant SoC.
Packet DMA (PKTDMA) and Block Copy DMA (BCDMA) which are the two instances of the DMSS specification serving different use cases.

\imageStyle{dmss_blobk_diagram.PNG,width:30%}
\image html dmss_blobk_diagram.PNG "DMSS Block Diagram"

### Packet DMA (PKTDMA)

The PKTDMA is intended to perform similar functions as the packet oriented DMA.
The PKTDMA module supports the transmission and reception of various packet types.
The PKTDMA is architected to facilitate the segmentation and reassembly of DMA data structure compliant packets to/from smaller data blocks that are natively compatible with the specific requirements of each connected peripheral.
Multiple TX and RX channels are provided within the DMA which allow multiple segmentation or reassembly operations to be ongoing.
The DMA controller maintains state information for each of the channels which allows packet segmentation and reassembly operations to be time division multiplexed between channels in order to share the underlying DMA hardware.
An internal DMA scheduler is used to control the ordering and rate at which this multiplexing occurs for Transmit operations.
The ordering and rate of Receive operations is indirectly controlled by the order in which blocks are pushed into the DMA on the RX PSI-L interface.

\imageStyle{pktdma_blobk_diagram.PNG,width:50%}
\image html pktdma_blobk_diagram.PNG "Packet DMA Block Diagram"

### Block Copy DMA (BCDMA)

The Block Copy DMA is intended to perform similar functions as the EDMA or the UDMA-P/UTC.
The BCDMA module moves data from a memory mapped source address set to a corresponding memory mapped address set.
The BCDMA maintains state information for each of the channels which allows data copy operations to be time division multiplexed between channels in order to share the underlying DMA hardware.
An internal DMA scheduler is used to control the ordering and rate at which this multiplexing occurs.

\imageStyle{bcdma_blobk_diagram.PNG,width:50%}
\image html bcdma_blobk_diagram.PNG "Block Copy DMA Block Diagram"

## DMSS Transfer Overciew

Below section describes the high level flow of the driver for the data transfer

### Transfer Request (TR) Record

Transfer configuration is specified in the TR record. Size of TR is variable from 16 bytes to 64 bytes. Specified via TR Type in FLAGS field

\imageStyle{tr_record_diagram.PNG,width:50%}
\image html tr_record_diagram.PNG "TR record fields"

Below table summarizes different TR types and the transfer type for which they are used

TR Type  | Descriptrion
---------|-----------------------------------------------------------------------------
Type 0   | 1D (word0-3)
Type 1   | 2D (word0-4)
Type 2   | 3D (word0-6)
Type 3   | 4D (word0-8)
Type 5   | Cache warm (word0-15) (MSMC DRU ONLY)
Type 8   | 4D Block Copy (word0-15)
Type 9   | 4D Block Copy with reformatting (word0-15) (MSMC DRU ONLY)
Type 10  | 2D Block Copy (word0-15)
Type 11  | 2D Block Copy with reformatting (word0-15) (MSMC DRU ONLY)
Type 15  | 4D Block Copy with reformatting and indirection (word0-15) (MSMC DRU ONLY)

### UDMA Setup/Flow

Below diagram shows the high level flow for the transfer requests from application and driver

\imageStyle{trpd_flow.PNG,width:50%}
\image html trpd_flow.PNG "UDMA setup TRPD flow"

Below diagram shows the UDMA transfer API flow

\imageStyle{udma_api_flow.PNG,width:50%}
\image html udma_api_flow.PNG "UDMA API flow"

## Additional Documentation

- \htmllink{../am64x_am243x/Migrating_Applications_from_EDMA_to_UDMA_using_TI-RTOS.pdf, EDMA to UDMA Migration} document.
- \htmllink{../am64x_am243x/Migrating_Applications_from_NAVSS_UDMA_to_DMSS_using_TI-RTOS.pdf, UDMA to DMSS Migration} document.

## Example Usage

Include the below file to access the APIs
\snippet Udma_sample.c include

Channel Open Example
\snippet Udma_sample.c ch_open

Channel Close Example
\snippet Udma_sample.c ch_close

## API

\ref DRV_UDMA_MODULE
