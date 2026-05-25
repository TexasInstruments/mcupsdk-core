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

\cond SOC_AM64X || SOC_AM243X
## PDMA Transfer Size Limitation (12-bit Counter)

**Maximum PDMA transfer: 4,095 words per transaction**

When performing DMA transfers to/from PDMA-serviced peripherals (McSPI, UART, ADC, MCAN, McASP), the transfer is limited by the 12-bit transfer counter in the PDMA hardware.

### Understanding the 12-bit Counter Limitation

The transfer counter field is **12 bits wide**, limiting the maximum count to **4,095 decimal (0xFFF hexadecimal)**.

**Important:** The counter tracks **WORDS**, not bytes. The relationship between words and bytes depends on the configured data width.

### Word to Bytes Conversion

A **word** is a configurable unit of data:
- **8-bit word length:** 1 word = 1 byte
- **16-bit word length:** 1 word = 2 bytes
- **32-bit word length:** 1 word = 4 bytes

### Maximum Transfer Sizes by Data Width

| Data Width | Word Size | Max Words | Max Bytes | Register Value |
|------------|-----------|-----------|-----------|-----------------|
| 8-bit  | 1 byte  | 4,095 | **4,095 bytes** | 0xFFF |
| 16-bit | 2 bytes | 4,095 | **8,190 bytes** | 0xFFF |
| 32-bit | 4 bytes | 4,095 | **16,380 bytes** | 0xFFF |

### Danger Zone: Register Overflow

**Attempting a transfer exceeding the 12-bit limit causes hardware overflow:**

```
Transfer attempt: 4,096 bytes with 8-bit data
  → Word count needed: 4,096 (decimal)
  → Hex representation: 0x1000 (requires 13 bits)
  → Register overflow: Wraps to 0x000
  → Result: Transfer fails, system may hang
```

### Peripheral-Specific Examples

**McSPI 8-bit data transfer:**
- Element size: 1 byte
- Max count register value: 4,095
- Max bytes: 4,095

**McSPI 16-bit data transfer:**
- Element size: 2 bytes
- Max count register value: 4,095
- Max bytes: 8,190

**UART (typically 8-bit):**
- Element size: 1 byte
- Max count register value: 4,095
- Max bytes: 4,095

### Workaround for Large Transfers (> 4,095 words)

To transfer data exceeding the PDMA limit:

1. **Split into multiple transactions:**
   - Each transaction ≤ 4,095 words
   - Manage peripheral address appropriately
   - Handle synchronization between transfers

2. **Use DMA Chaining (if available):**
   - Link multiple TR descriptors
   - Automatic transition between transfers
   - Minimal software overhead

3. **Manual Loop-based Transfer:**
   - Submit transfers one at a time
   - Poll for completion or use interrupts
   - Update source/destination addresses

### Example Calculation

**Transferring 10,000 bytes via McSPI (8-bit data):**
```
Total bytes needed: 10,000
Max per transaction: 4,095
Transactions required: ceil(10,000 / 4,095) = 3 transactions

Transaction 1: 4,095 bytes (count = 4,095)
Transaction 2: 4,095 bytes (count = 4,095)
Transaction 3: 1,810 bytes (count = 1,810)
```

### Validation in Driver

When submitting DMA transfers via UDMA API:
```c
// For PDMA peripheral with 8-bit data width
if (transferSizeInBytes > 4095)
{
    // Split transfer or use chaining
    // Attempting in single transaction will fail
}
```

\endcond

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
