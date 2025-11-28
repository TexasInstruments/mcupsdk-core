# ICSS EMAC Queue Management Design {#ICSS_EMAC_QUEUE_MANAGEMENT_DESIGN}

[TOC]

## Overview

This page describes the queue management implementation in the ICSS EMAC driver. The queue management system is responsible for managing the flow of packets between the PRU firmware and the host processor.

## Ports in ICSS EMAC

The ICSS-EMAC architecture uses a specific port numbering convention that is fundamental to understanding queue management. Although there are only two physical Ethernet ports on every ICSS instance, the system treats the host processor as a logical third port for convenience and protocol compatibility.

The ports are referred to as:
- Host Port (in Switch mode): \ref ICSS_EMAC_PORT_0
- Physical Port 0: \ref ICSS_EMAC_PORT_1
- Physical Port 1: \ref ICSS_EMAC_PORT_2

## Port-Based Queue Organization

Each port has its own set of queues, and the total number varies by operating mode.

### Queue Distribution

**Switch Mode**: 15 queues total
- 4 host receive queues (\ref ICSS_EMAC_PORT_0)
- 4 transmit queues for Physical Port 0 (\ref ICSS_EMAC_PORT_1)
- 4 transmit queues for Physical Port 1 (\ref ICSS_EMAC_PORT_2)
- 3 collision queues (1 per port)

**EMAC Mode**: 12 queues total
- 2 host receive queues for Physical Port 0 (\ref ICSS_EMAC_PORT_0)
- 2 host receive queues for Physical Port 1 (\ref ICSS_EMAC_PORT_0)
- 4 transmit queues for Physical Port 0 (\ref ICSS_EMAC_PORT_1)
- 4 transmit queues for Physical Port 1 (\ref ICSS_EMAC_PORT_2)
- No collision queues (due to independent operation)

Each queue is a block of memory used to store packet data. Queue sizes may vary and are configurable using \ref ICSS_EMAC_FwDynamicMmap (which is a member of \ref ICSS_EMAC_Params passed while calling \ref ICSS_EMAC_open).

\image html ICSS_EMAC_Queues.PNG

### Default Queue Sizes

The default queue sizes are shown below. The sizes are denoted by blocks, where each block is 32 bytes in size.

```c
/* Physical Port queue size. Same for both ports */
#define ICSS_EMAC_DEFAULT_FW_QUEUE_1_SIZE        ((uint32_t)97U)       /* Network Management high */
#define ICSS_EMAC_DEFAULT_FW_QUEUE_2_SIZE        ((uint32_t)97U)       /* Network Management low */
#define ICSS_EMAC_DEFAULT_FW_QUEUE_3_SIZE        ((uint32_t)97U)       /* Protocol specific */
#define ICSS_EMAC_DEFAULT_FW_QUEUE_4_SIZE        ((uint32_t)97U)       /* NRT (IP,ARP, ICMP, ?) */

/* HOST PORT QUEUES can buffer up to 4 full sized frames per queue */
#define ICSS_EMAC_DEFAULT_FW_HOST_QUEUE_1_SIZE   ((uint32_t)194U)      /* Protocol and/or VLAN priority 7 and 6 */
#define ICSS_EMAC_DEFAULT_FW_HOST_QUEUE_2_SIZE   ((uint32_t)194U)      /* Protocol mid */
#define ICSS_EMAC_DEFAULT_FW_HOST_QUEUE_3_SIZE   ((uint32_t)194U)      /* Protocol low */
#define ICSS_EMAC_DEFAULT_FW_HOST_QUEUE_4_SIZE   ((uint32_t)194U)      /* NRT (IP, ARP, ICMP ?) */

#define ICSS_EMAC_DEFAULT_FW_COLLISION_QUEUE_SIZE (48U)
```

## Introduction to Queue Management Units

### Buffer

A buffer is the actual memory space where packet data is stored.

- **Size**: Fixed 32-byte blocks
- **Location**: Stored in MSRAM
- **Content**: Raw Ethernet packet data
- **Alignment**: 32-byte aligned

Buffers are stored in memory starting at the offset: `icssEmacHandle->attrs->l3OcmcBaseAddr + rxQueue->buffer_offset`

The start MSRAM address where the packet content is actually stored is calculated by the driver via: `icssEmacHandle->attrs->l3OcmcBaseAddr + rxQueue->buffer_offset + buffer_desc_num`

**Buffer Example (32 bytes of packet data):**

```
┌──────────────────────────────────────────────────────────────┐
│ Ethernet Header      | IP Header (partial) | Data continues  │
│ (14 bytes)           | (18 bytes)          | in next buffer  │
└──────────────────────────────────────────────────────────────┘
│◄──────────────────── 32 bytes total ───────────────────────►│
```

\note A single buffer holds exactly 32 bytes. If a packet is larger, it spans multiple buffers. For example, a typical Ethernet frame with headers would need multiple 32-byte buffers to store the complete packet.

### Buffer Descriptor (BD)

A buffer descriptor is metadata that describes and points to buffer(s) containing packet data.

- **Size**: 32 bits (4 bytes) per descriptor
- **Location**: Stored in ICSS Shared RAM in a contiguous manner
- **Purpose**: Contains packet metadata and buffer management info
- **Relationship**: Multiple BDs can point to different 32-byte blocks of the same packet

BDs are stored in memory starting at the offset: `ICSS_EMAC_QueueParams *rxQueue = icssEmacHandle->object->switchPort[ICSS_EMAC_PORT_0]->queue[x]->buffer_desc_offset`

For calculating the first buffer descriptor corresponding to a packet, the driver does the following: `buffer_desc_num = (queue_rd_ptr - rxQueue->buffer_desc_offset) / (ICSS_EMAC_DEFAULT_FW_BD_SIZE (4))`

**Buffer Descriptor Structure (32-bit):**

| Bit 31 | Bit 30 | Bit 29 | Bits 28-18 | Bits 17-16 | Bits 15-0 |
|--------|--------|--------|------------|------------|-----------|
| Error Flag | Broadcast Flag | VLAN Tag | Length (11 bits) | Port (2 bits) | Unused/Protocol Specific |

**Example: 100-byte packet needs 4 buffer descriptors:**

```
Packet (100 bytes) split across buffers:
┌────────────────────┬────────────────────┬────────────────────┬────────────────────┐
│ BD1 → Buffer1 (32B)│ BD2 → Buffer2 (32B)│ BD3 → Buffer3 (32B)│ BD4 → Buffer4 (4B) │
│ [Length=100]       │ [Length=0]         │ [Length=0]         │ [Length=0]         │
│ [Port=1]           │ [Port=0]           │ [Port=0]           │ [Port=0]           │
│ [Error=0]          │ [Error=0]          │ [Error=0]          │ [Error=0]          │
└────────────────────┴────────────────────┴────────────────────┴────────────────────┘
```

\note The first BD contains the complete packet length and metadata, while subsequent BDs point to additional 32-byte data blocks.

### Queue

A queue is a logical collection of buffers organized to store multiple packets in a FIFO manner.

- **Organization**: Circular buffer structure
- **Capacity**: Configurable size in terms of 32-byte blocks
- **Types**: Host receive queues, Port transmit queues, Collision queues
- **Priority**: 4 priority levels per port (Queue 0=highest, Queue 3=lowest)
- **Location**: Stored in MSRAM

**Queue Structure (Logical View):**

```
┌─────────────────────────────────────────────────────────────┐
│ Queue N (e.g., P0_Q1)                                       │
├─────────────────────────────────────────────────────────────┤
│ Packet 1 Buffers:                                           │
│ ┌──────┐ ┌──────┐ ┌──────┐                                  │
│ │Buffer│ │Buffer│ │Buffer│                                  │
│ │  1   │ │  2   │ │  3   │                                  │
│ │ 32B  │ │ 32B  │ │  4B  │                                  │
│ └──────┘ └──────┘ └──────┘                                  │
│                                                             │
│ Packet 2 Buffers:                                           │
│ ┌──────┐ ┌──────┐                                           │
│ │Buffer│ │Buffer│                                           │
│ │  4   │ │  5   │                                           │
│ │ 32B  │ │ 32B  │                                           │
│ └──────┘ └──────┘                                           │
│                                                             │
│ ... (more packets)                                          │
└─────────────────────────────────────────────────────────────┘
```

### Queue Descriptor (QD)

A queue descriptor is the control structure that manages a queue's state and pointers.

- **Size**: 64 bits (8 bytes) per descriptor
- **Location**: Stored in ICSS Data RAM 1 (Switch Mode) or ICSS Shared RAM (EMAC Mode)
- **Purpose**: Tracks queue fill level, read/write positions, and status
- **Shared**: Accessed by both PRU firmware and host driver

**Queue Descriptor Fields:**

- **Rd_ptr**: Read pointer - Points to the next buffer descriptor to be read by the driver/host. Driver increments this after reading data.
- **Wr_ptr**: Write pointer - Points to the next buffer descriptor where firmware will write new data. Firmware increments this after writing new data. When read pointer equals write pointer, queue is empty.
- **busy_s**: Driver sets when copying data (prevents firmware corruption)
- **status**: Queue operational status
- **max_fill_level**: Peak queue utilization in bytes
- **overflow_cnt**: Number of overflow events

For switch mode, the QDs are stored starting at offset: `pruicssHwAttrs->pru1DramBase + pStaticMMap->p0QueueDescOffset`

For checking the various QDs in Switch mode, the driver accesses them via: `pruicssHwAttrs->pru1DramBase + pStaticMMap->p0QueueDescOffset + (queueNum * ICSS_EMAC_DEFAULT_FW_QD_SIZE (8))`

**Queue Descriptor Structure (64-bit):**

| Bits 63-56 | Bits 55-48 | Bits 47-40 | Bits 39-32 | Bits 31-16 | Bits 15-0 |
|------------|------------|------------|------------|------------|-----------|
| overflow_cnt | max_fill_level | status | busy_s | wr_ptr | rd_ptr |

## Detailed Component Structure and Relationships

The following diagram shows the hierarchical relationship between all queue management components:

```
┌────────────────────────────────────────────────────────────────────────────┐
│ Component Detailed Structure                                               │
├────────────────────────────────────────────────────────────────────────────┤
│                                                                            │
│ ┌────────────────────────────────────────────────────────────────────────┐ │
│ │ QUEUE (Logical Entity)                                                 │ │
│ │ ┌────────────────────────────────────────────────────────────────────┐ │ │
│ │ │ Purpose: Organize multiple packets in FIFO order                   │ │ │
│ │ │ Types: Host Rx (4), Port1 Tx (4), Port2 Tx (4), Collision (3)      │ │ │
│ │ │ Organization: Circular buffer with configurable size               │ │ │
│ │ │ Priority: 0=Highest (Network Mgmt), 3=Lowest (NRT)                 │ │ │
│ │ └────────────────────────────────────────────────────────────────────┘ │ │
│ │                           │                                            │ │
│ │                           │ Managed by                                 │ │
│ │                           ▼                                            │ │
│ │ ┌────────────────────────────────────────────────────────────────────┐ │ │
│ │ │ QUEUE DESCRIPTOR (Control Structure)                               │ │ │
│ │ │ Location: PRU1 Data RAM (Switch) / Shared RAM (EMAC)               │ │ │
│ │ │ Size: 64 bits (8 bytes)                                            │ │ │
│ │ │ ┌────────────────────────────────────────────────────────────────┐ │ │ │
│ │ │ │ Bits 63-56   |  55-48   |  47-40 | 39-32  |  31-16 |  15-0     │ │ │ │
│ │ │ │ overflow_cnt | max_fill | status | busy_s | Wr_ptr | Rd_ptr    │ │ │ │
│ │ │ └────────────────────────────────────────────────────────────────┘ │ │ │
│ │ │ • Rd_ptr: Next BD to be read by driver (consumer pointer)          │ │ │
│ │ │ • Wr_ptr: Next BD for firmware to write (producer pointer)         │ │ │
│ │ │ • busy_s: Driver sets during data copy operations                  │ │ │
│ │ │ • Queue Empty: Rd_ptr == Wr_ptr                                    │ │ │
│ │ └────────────────────────────────────────────────────────────────────┘ │ │
│ │                           │                                            │ │
│ │                           │ Points to                                  │ │
│ │                           ▼                                            │ │
│ │ ┌────────────────────────────────────────────────────────────────────┐ │ │
│ │ │ BUFFER DESCRIPTORS (Packet Metadata)                               │ │ │
│ │ │ Location: ICSS Shared RAM                                          │ │ │
│ │ │ Size: 32 bits (4 bytes) per descriptor                             │ │ │
│ │ │ ┌────────────────────────────────────────────────────────────────┐ │ │ │
│ │ │ │ Bit 31 | 30    | 29   | 28-18  | 17-16 | Bits 15-0             │ │ │ │
│ │ │ │ Error  | Bcast | VLAN | Length | Port  | Unused/Protocol       │ │ │ │
│ │ │ │ Flag   | Flag  | Tag  | (11b)  | (2b)  | Specific              │ │ │ │
│ │ │ └────────────────────────────────────────────────────────────────┘ │ │ │
│ │ │ • First BD: Contains complete packet length and metadata           │ │ │
│ │ │ • Subsequent BDs: Point to additional 32-byte data blocks          │ │ │
│ │ │ • One BD per 32-byte data block                                    │ │ │
│ │ └────────────────────────────────────────────────────────────────────┘ │ │
│ │                           │                                            │ │
│ │                           │ Points to                                  │ │
│ │                           ▼                                            │ │
│ │ ┌────────────────────────────────────────────────────────────────────┐ │ │
│ │ │ BUFFERS (Packet Data Storage)                                      │ │ │
│ │ │ Location: MSRAM (L3 OCMC RAM)                                      │ │ │
│ │ │ Size: 32 bytes per buffer block                                    │ │ │
│ │ │ ┌────────────────────────────────────────────────────────────────┐ │ │ │
│ │ │ │ Buffer 1 (32B) | Buffer 2 (32B) | Buffer 3 (32B) | Buffer N    │ │ │ │
│ │ │ │ ┌────────────┐ │ ┌────────────┐ │ ┌────────────┐ │ ┌────────┐  │ │ │ │
│ │ │ │ │Eth Header  │ │ │IP Header   │ │ │TCP Header  │ │ │Payload │  │ │ │ │
│ │ │ │ │+ IP start  │ │ │+ TCP start │ │ │+ Payload   │ │ │(remain)│  │ │ │ │
│ │ │ │ │(32 bytes)  │ │ │(32 bytes)  │ │ │(32 bytes)  │ │ │(≤32B)  │  │ │ │ │
│ │ │ │ └────────────┘ │ └────────────┘ │ └────────────┘ │ └────────┘  │ │ │ │
│ │ │ └────────────────────────────────────────────────────────────────┘ │ │ │
│ │ │ • 32-byte aligned blocks                                           │ │ │
│ │ │ • Variable number of blocks per packet (based on packet size)      │ │ │
│ │ │ • Fast access from MSRAM (faster than DDR)                         │ │ │
│ │ └────────────────────────────────────────────────────────────────────┘ │ │
│ └────────────────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────────────┘
```

## Collision Queue

### What is Queue Contention?

Queue contention occurs when two PRUs or a PRU and the Host want to acquire the same queue at the same time. This creates a race condition that must be resolved to prevent data corruption and ensure proper packet handling.

There are two main scenarios where contention happens:

**Host Queue Contention**: This occurs when both PRU0 and PRU1 try to write received packets to the same host queue simultaneously. For example, when both ports receive broadcast packets at the same time, both PRUs will attempt to store the packet in the host receive queue.

**Port Queue Contention**: This happens when the Host and a PRU try to access the same port transmit queue. For instance, the host application might want to transmit a packet while the PRU is performing a store-and-forward operation on the same queue.

### Primary-Secondary Arbitration System

To resolve these conflicts, ICSS-EMAC uses a Primary-Secondary arbitration scheme where different entities are assigned priority based on the queue type:

- **For Host Queues (\ref ICSS_EMAC_PORT_0)**: PRU0 acts as the primary arbiter and PRU1 acts as the secondary arbiter. This gives PRU0 priority when both PRUs need to access host queues.
- **For Physical Port 0 Transmit Queues (\ref ICSS_EMAC_PORT_1)**: PRU1 acts as the primary arbiter and the Host acts as the secondary arbiter, since PRU1 manages Physical Port 0 transmit operations.
- **For Physical Port 1 Transmit Queues (\ref ICSS_EMAC_PORT_2)**: PRU0 acts as the primary arbiter and the Host acts as the secondary arbiter, since PRU0 manages Physical Port 1 transmit operations.

Each queue descriptor contains ownership bits - one for secondary ownership and one for primary ownership. These bits are used to coordinate access between the competing entities.

### How the Arbitration Works

**Primary Arbiter Process**: When the primary arbiter wants to acquire a queue, it first checks the secondary ownership bit. If the secondary has already acquired the queue, the primary gives up and stores its packet in a collision queue instead. If the queue is free, the primary sets its ownership bit and proceeds to use the main queue.

**Secondary Arbiter Process**: The secondary arbiter process is more complex because it needs to handle race conditions. First, the secondary checks if the primary ownership bit is set. If so, it goes directly to the collision queue. If the primary bit is clear, the secondary sets its own ownership bit and then immediately re-checks the primary bit. This double-check is crucial because the primary arbiter might have acquired the queue between the secondary's initial check and when it set its own bit. If the primary did acquire the queue during this window, the secondary clears its ownership bit and falls back to using the collision queue.

### Collision Queue Mechanism

Collision queues serve as temporary storage when the main queue is busy. Each port has its own collision buffer in MSRAM.

The critical limitation is that each collision queue can only hold a single packet, regardless of the packet size. If a collision queue is already occupied and another contention occurs, the new packet will be dropped.

## Reception Flow Example

This section demonstrates how a packet is received from the network and delivered to the host application.

\note The offsets used here are for demonstration purposes only - the actual offsets when application is running on the target may differ.

### Initial State (Empty Queues)

```
Queue State: Empty
┌─────────────────────────────────────────────────────────────┐
│ Queue Descriptor (P0_Q1 - Host Receive Queue)               │
│ ┌─────────────────────────────────────────────────────────┐ │
│ │ Rd_ptr: 0x0400 ← Points to last valid BD (none yet)     │ │
│ │ Wr_ptr: 0x0400 ← Points to first BD to read (none)      │ │
│ │ busy_s: 0      ← Not busy                               │ │
│ └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘

Available Buffer Descriptors in ICSS Shared RAM:
┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐
│ BD1  │ │ BD2  │ │ BD3  │ │ BD4  │ ... (unused)
│@0x400│ │@0x404│ │@0x408│ │@0x40C│
└──────┘ └──────┘ └──────┘ └──────┘

Available Buffers in MSRAM:
┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐
│Buf1  │ │Buf2  │ │Buf3  │ │Buf4  │ ... (empty)
│ 32B  │ │ 32B  │ │ 32B  │ │ 32B  │
└──────┘ └──────┘ └──────┘ └──────┘
```

### Packet Arrives at PRU (Firmware Receives)

```
Incoming Packet: 68 bytes
┌─────────────────────────────────────────────────────────────┐
│ Ethernet Header | IP Header | TCP Header | Payload Data     │
│ (14 bytes)      |(20 bytes) |(20 bytes)  | (14 bytes)       │
└─────────────────────────────────────────────────────────────┘
│◄──────────────── 68 bytes total ────────────────────────────►│
```

### Firmware (Producer) Actions

**1. Firmware Stores Packet Data in Buffers (MSRAM)**

PRU Firmware splits 68-byte packet into 32-byte blocks:

```
Buffer 1 (32 bytes):
┌─────────────────────────────────────────────────────────────┐
│ Ethernet Header + Partial IP Header                         │
│ (14 + 18 = 32 bytes used)                                   │
└─────────────────────────────────────────────────────────────┘

Buffer 2 (32 bytes):
┌─────────────────────────────────────────────────────────────┐
│ Remaining IP Header + TCP Header + Partial Payload          │
│ (2 + 20 + 10 = 32 bytes used)                               │
└─────────────────────────────────────────────────────────────┘

Buffer 3 (32 bytes):
┌─────────────────────────────────────────────────────────────┐
│ Remaining Payload Data                                      │
│ (4 bytes used, 28 bytes unused in this block)               │
└─────────────────────────────────────────────────────────────┘
```

**2. Firmware Creates Buffer Descriptors (ICSS Shared RAM)**

```
Buffer Descriptors created by PRU:

BD1 @ 0x0400 (First BD - contains packet metadata):
┌───────┬────────┬──────┬──────────┬───────┬──────────┐
│  31   │   30   │  29  │  28-18   │ 17-16 │  15-0    │
├───────┼────────┼──────┼──────────┼───────┼──────────┤
│   0   │   0    │  0   │    68    │   1   │    0     │
│ Error │ B'cast │ VLAN │  Length  │ Port  │ Unused   │
│  No   │   No   │  No  │ 68 bytes │ Port1 │          │
└───────┴────────┴──────┴──────────┴───────┴──────────┘
Points to Buffer 1 in MSRAM

BD2 @ 0x0404 (Second BD):
┌───────┬────────┬──────┬──────────┬───────┬──────────┐
│   0   │   0    │  0   │    0     │   0   │    0     │
│ Error │ B'cast │ VLAN │  Length  │ Port  │ Unused   │
│  No   │   No   │  No  │No length │   -   │          │
└───────┴────────┴──────┴──────────┴───────┴──────────┘
Points to Buffer 2 in MSRAM

BD3 @ 0x0408 (Third BD):
┌───────┬────────┬──────┬──────────┬───────┬──────────┐
│   0   │   0    │  0   │    0     │   0   │    0     │
│ Error │ B'cast │ VLAN │  Length  │ Port  │ Unused   │
│  No   │   No   │  No  │No length │   -   │          │
└───────┴────────┴──────┴──────────┴───────┴──────────┘
Points to Buffer 3 in MSRAM
```

**3. Firmware Updates Queue Descriptor**

PRU Firmware updates wr_ptr (producer pointer):

```
Queue Descriptor After PRU Processing:
┌─────────────────────────────────────────────────────────────┐
│ Rd_ptr: 0x0400 ← UNCHANGED: Still points to BD1 (first)     │
│ Wr_ptr: 0x040C ← PRU UPDATED: Points past BD3 (last valid)  │
│ busy_s: 0      ← Not busy                                   │
└─────────────────────────────────────────────────────────────┘

Queue State: Has Data (Wr_ptr > Rd_ptr)
Available for host consumption: 1 packet (68 bytes)
```

**4. Firmware Generates Host Interrupt**

The PRU firmware triggers interrupt to the host processor via ICSS INTC - notifying that a packet is ready to be received.

### Host (Consumer) Actions

1. **Host Detects New Packet**: `ICSS_EMAC_rxInterruptHandler` wakes up RxTask (`ICSS_EMAC_osRxTaskFnc`) and `ICSS_EMAC_pollPkt()` is called.

2. **Driver Process**: Within `ICSS_EMAC_pollPkt()`, the function `ICSS_EMAC_rxPktInfo2()`:
   - Gets the Queue Descriptor and checks if read and write pointers differ
   - Detects a difference and calculates buffer descriptor number
   - Extracts packet metadata (length, port, etc.) from the first BD
   - Compares queue number against `ethPrioQueue` to determine RT/NRT callback
   - Calls `ICSS_EMAC_rxPktGet()` which copies packet data to application buffer and updates the read pointer

## Transmission Flow Example

This section demonstrates how a packet is transmitted from the host application to the network.

\note The offsets used here are for demonstration purposes only - the actual offsets when application is running on the target may differ.

### Initial State (Empty Transmit Queue)

```
Port 1 Transmit Queue (P1_Q1) - Initial State:
┌─────────────────────────────────────────────────────────────┐
│ Queue Descriptor (P1_Q1 - Port 1 Transmit Queue)            │
│ ┌─────────────────────────────────────────────────────────┐ │
│ │ Rd_ptr: 0x1020 ← Points to next BD for driver to read   │ │
│ │ Wr_ptr: 0x1020 ← Points to next BD for firmware write   │ │
│ │ busy_s: 0      ← Not busy                               │ │
│ │ status: empty  ← Queue is empty (Rd_ptr == Wr_ptr)      │ │
│ └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘

Available Buffer Descriptors in ICSS Shared RAM:
┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐
│ BD1  │ │ BD2  │ │ BD3  │ │ BD4  │ ... (unused)
│@1020 │ │@1024 │ │@1028 │ │@102C │
└──────┘ └──────┘ └──────┘ └──────┘

Available Buffers in MSRAM:
┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐
│Buf1  │ │Buf2  │ │Buf3  │ │Buf4  │ ... (empty)
│ 32B  │ │ 32B  │ │ 32B  │ │ 32B  │
└──────┘ └──────┘ └──────┘ └──────┘
```

### Application Calls ICSS_EMAC_txPacket

```
Application Request:
┌─────────────────────────────────────────────────────────────┐
│ ICSS_EMAC_TxArgument txArgs:                                │
│ ├─ icssEmacHandle: handle                                   │
│ ├─ srcAddress: &applicationBuffer[0]                        │
│ ├─ portNumber: ICSS_EMAC_PORT_1                             │
│ ├─ queuePriority: ICSS_EMAC_QUEUE1                          │
│ └─ lengthOfPacket: 100 bytes                                │
└─────────────────────────────────────────────────────────────┘

Application Buffer (100 bytes):
┌─────────────────────────────────────────────────────────────┐
│ Ethernet Header | IP Header | TCP Header | Payload Data     │
│ (14 bytes)      |(20 bytes) |(20 bytes)  | (46 bytes)       │
└─────────────────────────────────────────────────────────────┘
│◄──────────────── 100 bytes total ───────────────────────────►│
```

### Driver (Producer) Actions in ICSS_EMAC_txPacketEnqueue

1. **Queue Validation and Selection**: The driver validates port number, queue priority, packet length, and link status, then selects the appropriate transmit queue.

**2. Queue Contention Check (Switch Mode)**

```
Driver checks queue availability:

Queue Descriptor Check:
┌────────────────────────────────────────────────────────────────────┐
│ Read queue descriptor busy_m bit (Primary Arbiter busy bit)        │
│ If busy_m == 0: Queue available                                    │
│ If busy_m == 1: Queue busy, use collision queue                    │
└────────────────────────────────────────────────────────────────────┘

Assume queue is available, so driver acquires it:
1. Set busy_s bit (Secondary Arbiter busy) to acquire queue
2. Re-check busy_m bit to ensure successful acquisition
3. If acquisition failed, fall back to collision queue
```

**3. Driver Sets Busy Flag and Calculates Space**

```
Queue Descriptor After Driver Acquisition:
┌─────────────────────────────────────────────────────────────┐
│ Rd_ptr: 0x1020 ← Unchanged (firmware will read from here)   │
│ Wr_ptr: 0x1020 ← Current write position                     │
│ busy_s: 1      ← DRIVER SET: Acquiring queue                │
│ status: busy   ← Driver is writing                          │
└─────────────────────────────────────────────────────────────┘

Space Calculation:
- Packet size: 100 bytes
- Blocks needed: ⌈100/32⌉ = 4 blocks (32+32+32+4 bytes)
- Buffer descriptors needed: 4 BDs
- New Wr_ptr will be: 0x1020 + (4 × 4) = 0x1030
```

**4. Driver Creates Buffer Descriptors**

```
Buffer Descriptors created by Driver:

BD1 @ 0x1020 (First BD - contains packet metadata):
┌─────┬─────┬─────┬─────────┬─────┬─────────────────────────┐
│ 31  │ 30  │ 29  │  28-18  │17-16│         15-0            │
├─────┼─────┼─────┼─────────┼─────┼─────────────────────────┤
│  0  │  0  │  0  │   100   │  1  │           0             │
│Error│Bcast│VLAN │ Length  │Port │        Unused           │
│ No  │ No  │ No  │100 bytes│Port1│                         │
└─────┴─────┴─────┴─────────┴─────┴─────────────────────────┘
Points to Buffer 1 in MSRAM

BD2 @ 0x1024, BD3 @ 0x1028, BD4 @ 0x102C:
┌─────┬─────┬─────┬─────────┬─────┬─────────────────────────┐
│  0  │  0  │  0  │    0    │  0  │           0             │
│Error│Bcast│VLAN │ Length  │Port │        Unused           │
│ No  │ No  │ No  │No length│  -  │                         │
└─────┴─────┴─────┴─────────┴─────┴─────────────────────────┘
Point to Buffers 2, 3, 4 respectively
```

**5. Driver Copies Data to Buffers (MSRAM)**

Driver copies the 100-byte packet from the application buffer to the MSRAM. The logical partitioning of the 100-byte packet in terms of buffers of 32 bytes would look like this:

```
Buffer 1 (32 bytes) @ MSRAM offset:
┌─────────────────────────────────────────────────────────────┐
│ Ethernet Header + Partial IP Header                         │
│ (14 + 18 = 32 bytes used)                                   │
└─────────────────────────────────────────────────────────────┘

Buffer 2 (32 bytes):
┌─────────────────────────────────────────────────────────────┐
│ Remaining IP Header + TCP Header + Partial Payload          │
│ (2 + 20 + 10 = 32 bytes used)                               │
└─────────────────────────────────────────────────────────────┘

Buffer 3 (32 bytes):
┌─────────────────────────────────────────────────────────────┐
│ Payload Data (continuing)                                   │
│ (32 bytes used)                                             │
└─────────────────────────────────────────────────────────────┘

Buffer 4 (32 bytes):
┌─────────────────────────────────────────────────────────────┐
│ Remaining Payload Data                                      │
│ (4 bytes used, 28 bytes unused)                             │
└─────────────────────────────────────────────────────────────┘
```

**6. Driver Updates Queue Descriptor**

```
Queue Descriptor After Driver Data Copy:
┌─────────────────────────────────────────────────────────────┐
│ Rd_ptr: 0x1020 ← Unchanged (firmware reads from here)       │
│ Wr_ptr: 0x1030 ← DRIVER UPDATED: Points to next free BD     │
│ busy_s: 0      ← DRIVER CLEARED: Copy complete              │
│ status: ready  ← Packet ready for firmware transmission     │
└─────────────────────────────────────────────────────────────┘

Queue State: Has Data (Wr_ptr > Rd_ptr)
Available for firmware transmission: 1 packet (100 bytes)
```

### Firmware (Consumer) Actions

**1. Firmware Detects New Packet**

```
PRU Firmware (Transmit Task):
1. Micro-scheduler calls TX Task in round-robin manner
2. TX Task scans transmit queues (highest priority first)
3. Finds P1_Q1 has data: Wr_ptr (0x1030) > Rd_ptr (0x1020)
4. Reads BD1 to get packet info: Length=100, Port=1
5. Sets XMT_active flag to indicate transmission in progress
```

**2. Firmware Transmission Process**

```
PRU Transmit Task Phases:

XMT_FB (Transmit First Block):
├─ Initialize TX context from BD1
├─ Fetch first 32 bytes from Buffer 1
└─ Push first 32 bytes to TX FIFO

XMT_NB (Transmit Next Block):
├─ Monitor TX FIFO fill level
├─ Fetch next 32-byte blocks (Buffers 2, 3)
├─ Push data to TX FIFO as space becomes available
└─ Continue until last block

XMT_LB (Transmit Last Block):
├─ Push remaining bytes from Buffer 4 (4 bytes)
├─ Append CRC to frame
├─ Wait for transmission completion (TX_EOF event)
└─ Update queue descriptor
```

**3. Firmware Updates Queue Descriptor**

```
Queue Descriptor After Firmware Transmission:
┌─────────────────────────────────────────────────────────────┐
│ Rd_ptr: 0x1030 ← FIRMWARE UPDATED: Advanced past packet     │
│ Wr_ptr: 0x1030 ← Unchanged                                  │
│ busy_s: 0      ← Not busy                                   │
│ status: empty  ← Queue is empty again (Rd_ptr == Wr_ptr)    │
└─────────────────────────────────────────────────────────────┘

Queue State: Empty again
Buffers 1, 2, 3, 4 are now available for reuse
Buffer Descriptors BD1, BD2, BD3, BD4 are available for reuse
```

**4. Packet Transmitted on Ethernet**

```
PRU Output:
┌────────────────────────────────────────────────────────────────┐
│ Original 100-byte packet + 4-byte CRC = 104 bytes on wire      │
│ ┌────────────────────────────────────────────────────────────┐ │
│ │ Ethernet Header | IP | TCP | Payload    | CRC (auto-added) │ │
│ │ (14 bytes)      |20B |20B  | (46 bytes) | (4 bytes)        │ │
│ └────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────┘
```
