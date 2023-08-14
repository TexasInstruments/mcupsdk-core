# UDMA {#DRIVERS_UDMA_PAGE}

[TOC]

The UDMA driver provides API to program the DMA module of the NAVSS subsystem
to setup and initiate DMA transfers.

## Features Supported

- Supports all DMA operations from both instances from all the cores in the SOC except M3F core
- UDMA block copy for memory to memory transfer
- PDMA module to initiate transfers to/from PDMA peripherals like UART, McASP, McSPI, ADC, MCAN
- DMA transfer to/from from native PSIL peripherals like CPSW, SA2UL
- Event and interrupt management like DMA completion, channel chaining, interrupt sharing using Interrupt Aggregator (IA)
- Resources management across instances and cores for DMA channels, RX flow, Interrupt Aggregator (IA), Interrupt Routers (IR), Global events, Ring Accelerator (RA)
- Interaction with DMSC RM module via SCICLIENT for all non-RealTime (NRT) configuration

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

- Selection of UDMA instance
- Option to skip default global event registration done as part of \ref Udma_init API
- Option to provide user function for virtual to physical and physical to virtual address translation
- Ability of add and configure DMA block copy channels
    - Ability to enable interrupt mode for the channel
    - Ability to specify the number of ring entries for the channel
    - Based on above parameters, the SysConfig generated code does below as part of Drivers_open and Drivers_close functions
        - Channel open/close - the handle can be retrieved by the application using g<User_Config_Name>BlkCopyChHandle global variable
        - Set default channel configuration
        - Allocates required ring memories and pass them to channel configuration
        - Register user specified callback when interrupt mode is enabled

## Features NOT Supported

- UDMA driver is not supported for M3F core as the DMSS is present only in the main domain

## Important Usage Guidelines

-  UDMA driver doesn't manage/allocate the descriptor and RA memory. The caller need to allocate and provide the required memory.
-  UDMA driver doesn't use any global variables. All the required object memory like channel, driver instance, event etc should be allocated by the caller

## NAVSS Overview

The DMA architecture specifies the data structures used by Texas Instruments standard communications modules to facilitate direct memory access (DMA) and to provide a consistent application programming interface (API) to the host software in multi-core devices. The data structures and the API used to manipulate them will be jointly referred to as NAVSS. Frequent tasks are commonly offloaded from the host processor to peripheral hardware to increase
system performance. Significant performance gains may result from careful design of the host software and communication module interface. In networking systems, packet transmission and reception are critical tasks. Texas Instruments has developed the NAVSS standard, which is aimed at maximizing the
efficiency of interaction between the host software and communications modules.

The Navigator Subsystem (NAVSS hardware) is a container which groups together as much as possible the components which provide the Hardare to Software boundary for data movement control in the system. All of the components which control work handoff (queuing, interrupts, monitoring and debugging)
are included in the NAVSS which in turn is located as close as possible to the various host processors to which services are provided. Other DMA components such as DRUs, UTC, and PDMAs exist outside the NAVSS but all are controlled by the root complexes provided in NAVSS.

\imageStyle{navss_block_diagram.PNG,width:50%}
\image html navss_block_diagram.PNG "NAVSS Block Diagram"

### Unified DMA Controller (UDMA-C)

The Unified DMA 3rd-Party Channel Controller is intended to perform similar (but significantly upgraded) functions to the EDMA Channel Controller engine on previous SoC devices. The UDMA-C module supports the transmission of Transfer Request packets from memory mapped data structures to data
channels within UTCs in the system and reception of Transfer Response packets from data channels in UTCs in the system into memory mapped data structures. Up to 512 (configuration specific) third-party.DMA control channels are provided within the UDMA-C. Each control channel corresponds to a single third party DMA data channel in a separate UTC. Transfer Request messages provide transfer parameters which are to be used by the UTC channels to move data in the system (previously implemented as PARAM entries). Transfer Response messages are sent back from the UTC in a one to one relationship
with Transfer Request packets and provide an indication of both completion of a data transfer and whether or not any exception or error occurred during the data transfer.

The UDMA-C controller maintains state information for each of its channels which allows Transfer Request packet transmission and Transfer Response packet reception operations to be time division multiplexed between channels in order to share the underlying DMA hardware. An internal DMA scheduler is used to control the ordering and rate at which this multiplexing occurs for the transmission of Transfer Request packets. The ordering and rate of Transfer Response receive operations is directly controlled by the order in which those packets are received on the Rx PSI-L interface.

\imageStyle{udmac_block_diagram.PNG,width:50%}
\image html udmac_block_diagram.PNG "UDMA-C Block Diagram"

### UDMA Peripheral (UDMA-P)

The UDMA-P is intended to perform similar (but significantly upgraded) functions as the packet-oriented DMA used on previous SoC devices. The UDMA-P module supports the transmission and reception of various packet types. The UDMA-P is architected to facilitate the segmentation and reassembly of SoC
DMA data structure compliant packets to/from smaller data blocks that are natively compatible with the specific requirements of each connected peripheral. Multiple Tx and Rx channels are provided within the DMA which allow multiple segmentation or reassembly operations to be ongoing. The DMA controller maintains state information for each of the channels which allows packet segmentation and reassembly operations to be time division multiplexed between channels in order to share the underlying DMA hardware. An external DMA scheduler is used to control the ordering and rate at which this multiplexing occurs for Transmit operations. The ordering and rate of Receive operations is indirectly controlled by the order in which blocks are pushed into the DMA on the Rx PSI-L interface.

The UDMA-P also supports acting as both a UTC and UDMA-C for its internal channels. Channels in the UDMA-P can be configured to be either Packet-Based or Third-Party channels on a channel by channel basis.

\imageStyle{udmap_block_diagram.PNG,width:50%}
\image html udmap_block_diagram.PNG "UDMA-P Block Diagram"

## NAVSS Transfer Overview

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

## Example Usage

Include the below file to access the APIs
\snippet Udma_sample.c include

Channel Open Example
\snippet Udma_sample.c ch_open

Channel Close Example
\snippet Udma_sample.c ch_close

## API

\ref DRV_UDMA_MODULE
