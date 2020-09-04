/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ICSS_EMAC_MMAP_H_
#define ICSS_EMAC_MMAP_H_

#ifdef __cplusplus
extern "C"
{
#endif


/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <networking/icss_emac/icss_emac.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/************************************************************************************
*  VLAN/Multicast filter defines & offsets, present on both PRU0 and PRU1 DRAM
*************************************************************************************/
/* Feature enable/disable values for multicast filtering */
#define ICSS_EMAC_FW_MULTICAST_FILTER_CTRL_DISABLED             0x00
#define ICSS_EMAC_FW_MULTICAST_FILTER_CTRL_ENABLED              0x01

/* Feature enable/disable values  for VLAN filtering */
#define ICSS_EMAC_FW_VLAN_FILTER_CTRL_DISABLED                  0x00
#define ICSS_EMAC_FW_VLAN_FILTER_CTRL_ENABLED                   0x01

/* Add/remove multicast mac id for filtering bin */
#define ICSS_EMAC_FW_MULTICAST_FILTER_HOST_RCV_ALLOWED          0x01
#define ICSS_EMAC_FW_MULTICAST_FILTER_HOST_RCV_NOT_ALLOWED      0x00

/* Default HASH value for the multicast filtering Mask */
#define ICSS_EMAC_FW_MULTICAST_FILTER_INIT_VAL                  0xFF

/* Size requirements for Multicast filtering feature */
#define ICSS_EMAC_FW_MULTICAST_TABLE_SIZE_BYTES                        256
#define ICSS_EMAC_FW_MULTICAST_FILTER_MASK_SIZE_BYTES                    6
#define ICSS_EMAC_FW_MULTICAST_FILTER_CTRL_SIZE_BYTES                    1
#define ICSS_EMAC_FW_MULTICAST_FILTER_MASK_OVERRIDE_STATUS_SIZE_BYTES    1
#define ICSS_EMAC_FW_MULTICAST_FILTER_DROP_CNT_SIZE_BYTES                4

/* Size requirements for VLAN filtering feature : 4096 bits = 512 bytes */
#define ICSS_EMAC_FW_VLAN_FILTER_TABLE_SIZE_BYTES                      512
#define ICSS_EMAC_FW_VLAN_FILTER_CTRL_SIZE_BYTES                         1
#define ICSS_EMAC_FW_VLAN_FILTER_DROP_CNT_SIZE_BYTES                     4

/* Mask override set status */
#define ICSS_EMAC_FW_MULTICAST_FILTER_MASK_OVERRIDE_SET                  1
/* Mask override not set status */
#define ICSS_EMAC_FW_MULTICAST_FILTER_MASK_OVERRIDE_NOT_SET              0
/* 6 bytes HASH Mask for the MAC */
#define ICSS_EMAC_FW_MULTICAST_FILTER_MASK_OFFSET         ( 0xF4 )
/* 0 -> multicast filtering disabled | 1 -> multicast filtering enabled */
#define ICSS_EMAC_FW_MULTICAST_FILTER_CTRL_OFFSET        ( ICSS_EMAC_FW_MULTICAST_FILTER_MASK_OFFSET + ICSS_EMAC_FW_MULTICAST_FILTER_MASK_SIZE_BYTES )
/* Status indicating if the HASH override is done or not: 0: no, 1: yes */
#define ICSS_EMAC_FW_MULTICAST_FILTER_OVERRIDE_STATUS    ( ICSS_EMAC_FW_MULTICAST_FILTER_CTRL_OFFSET + ICSS_EMAC_FW_MULTICAST_FILTER_CTRL_SIZE_BYTES )
/* Multicast drop statistics */
#define ICSS_EMAC_FW_MULTICAST_FILTER_DROP_CNT_OFFSET    ( ICSS_EMAC_FW_MULTICAST_FILTER_OVERRIDE_STATUS + ICSS_EMAC_FW_MULTICAST_FILTER_MASK_OVERRIDE_STATUS_SIZE_BYTES )
/* Multicast table */
#define ICSS_EMAC_FW_MULTICAST_FILTER_TABLE              ( ICSS_EMAC_FW_MULTICAST_FILTER_DROP_CNT_OFFSET + ICSS_EMAC_FW_MULTICAST_FILTER_DROP_CNT_SIZE_BYTES )

/* VLAN table Offsets */
#define ICSS_EMAC_FW_VLAN_FLTR_TBL_BASE_ADDR             ( 0x200 )
#define ICSS_EMAC_FW_VLAN_FILTER_CTRL_BITMAP_OFFSET      ( 0xEF  )
#define ICSS_EMAC_FW_VLAN_FILTER_DROP_CNT_OFFSET         ( ICSS_EMAC_FW_VLAN_FILTER_CTRL_BITMAP_OFFSET + ICSS_EMAC_FW_VLAN_FILTER_CTRL_SIZE_BYTES )

/* VLAN filter Control Bit maps */
/* one bit field, bit 0: | 0 : VLAN filter disabled (default), 1: VLAN filter enabled */
#define ICSS_EMAC_FW_VLAN_FILTER_CTRL_ENABLE_BIT                       ( 0 )
/* one bit field, bit 1: | 0 : untagged host rcv allowed (default), 1: untagged host rcv not allowed */
#define ICSS_EMAC_FW_VLAN_FILTER_UNTAG_HOST_RCV_ALLOW_CTRL_BIT         ( 1 )
/* one bit field, bit 1: | 0 : priotag host rcv allowed (default), 1: priotag host rcv not allowed */
#define ICSS_EMAC_FW_VLAN_FILTER_PRIOTAG_HOST_RCV_ALLOW_CTRL_BIT       ( 2 )
/* one bit field, bit 1: | 0 : skip sv vlan flow   :1 : take sv vlan flow  (not applicable for dual emac 
#define ICSS_EMAC_FW_VLAN_FILTER_SV_VLAN_FLOW_HOST_RCV_ALLOW_CTRL_BIT  ( 3 )
*/

/* VLAN IDs */
#define ICSS_EMAC_FW_VLAN_FILTER_PRIOTAG_VID                           ( 0 )
#define ICSS_EMAC_FW_VLAN_FILTER_VID_MIN                               ( 0x0000 )
#define ICSS_EMAC_FW_VLAN_FILTER_VID_MAX                               ( 0x0FFF )

/* VLAN Filtering Commands */
#define ICSS_EMAC_FW_VLAN_FILTER_ADD_VLAN_VID_CMD                      ( 0x00 )
#define ICSS_EMAC_FW_VLAN_FILTER_REMOVE_VLAN_VID_CMD                   ( 0x01 )
#define ICSS_EMAC_FW_BD_SIZE                         (4U)            /* one buffer descriptor is 4 bytes */
#define ICSS_EMAC_FW_BLOCK_SIZE                      (32U)           /* bytes derived from ICSS architecture */


/*Memory Usage of DRAM0/DRAM1 and Shared RAM */
#define EMAC_SPECIFIC_DRAM_START_OFFSET         ((uint32_t)0x1E98U)
#define SWITCH_SPECIFIC_SRAM_START_OFFSET       ((uint32_t)0x400U)
#define SWITCH_SPECIFIC_DRAM1_START_OFFSET      ((uint32_t)0x1D00U)


/* Queues on PHY PORT 1/2  */
//#define NUMBER_OF_QUEUES    4               /* different number of queues will have significant impact to memory map  */
/* 48 blocks per max packet  */
/* 2 full sized ETH packets: 96 blocks, 3 packets = 144, 4 packets = 192  */

/* Physical Port queue size. Same for both ports  */
#define ICSS_EMAC_FW_QUEUE_1_SIZE        (97U)           /* Network Management high  */
#define ICSS_EMAC_FW_QUEUE_2_SIZE        (97U)       /* Network Management low  */
#define ICSS_EMAC_FW_QUEUE_3_SIZE        (97U)       /* Protocol specific  */
#define ICSS_EMAC_FW_QUEUE_4_SIZE        (97U)       /* NRT (IP,ARP, ICMP, ?)    */

/* HOST PORT QUEUES can buffer up to 4 full sized frames per queue  */
#define ICSS_EMAC_FW_HOST_QUEUE_1_SIZE       (146U)      /* Protocol and/or VLAN priority 7 and 6  */
#define ICSS_EMAC_FW_HOST_QUEUE_2_SIZE       (194U)      /* Protocol mid  */
#define ICSS_EMAC_FW_HOST_QUEUE_3_SIZE       (194U)      /* Protocol low  */
#define ICSS_EMAC_FW_HOST_QUEUE_4_SIZE       (194U)      /* NRT (IP, ARP, ICMP ?)  */


#define ICSS_EMAC_NUMBER_OF_QUEUES  4

#define ICSS_EMAC_FW_COLLISION_QUEUE_SIZE    (48U)

#define ICSS_EMAC_FW_P0_BUFFER_DESC_OFFSET   (SWITCH_SPECIFIC_SRAM_START_OFFSET)





/***************************************************************************
*              General Purpose Statistics, these are present on both PRU0 and PRU1 DRAM
****************************************************************************/
/* Base statistics offset  */
#define ICSS_EMAC_FW_STATISTICS_OFFSET           (0x1F00U)
/*  Needs to be updated anytime a new statistic is added or existing statistics is no longer supported (removed).*/
#define ICSS_EMAC_FW_STAT_SIZE               (0x90U)

/* Offsets from ICSS_EMAC_FW_STATISTICS_OFFSET */
#define ICSS_EMAC_FW_STORM_PREVENTION_OFFSET         (ICSS_EMAC_FW_STATISTICS_OFFSET + ICSS_EMAC_FW_STAT_SIZE)         /*4 bytes */
#define ICSS_EMAC_FW_PHY_SPEED_OFFSET                (ICSS_EMAC_FW_STATISTICS_OFFSET + ICSS_EMAC_FW_STAT_SIZE + 4U)    /*4 bytes*/
#define ICSS_EMAC_FW_PORT_STATUS_OFFSET              (ICSS_EMAC_FW_STATISTICS_OFFSET + ICSS_EMAC_FW_STAT_SIZE + 8U)    /*1 byte */
#define ICSS_EMAC_FW_COLLISION_COUNTER               (ICSS_EMAC_FW_STATISTICS_OFFSET + ICSS_EMAC_FW_STAT_SIZE + 9U)    /*1 byte */
#define ICSS_EMAC_FW_RX_PKT_SIZE_OFFSET              (ICSS_EMAC_FW_STATISTICS_OFFSET + ICSS_EMAC_FW_STAT_SIZE + 10U)   /*4 bytes */
#define ICSS_EMAC_FW_PORT_CONTROL_ADDR               (ICSS_EMAC_FW_STATISTICS_OFFSET + ICSS_EMAC_FW_STAT_SIZE + 14U)   /*4 bytes   */
#define ICSS_EMAC_FW_PORT_MAC_ADDR                   (ICSS_EMAC_FW_STATISTICS_OFFSET + ICSS_EMAC_FW_STAT_SIZE + 18U)   /*6 bytes   */
#define ICSS_EMAC_FW_RX_INT_STATUS_OFFSET            (ICSS_EMAC_FW_STATISTICS_OFFSET + ICSS_EMAC_FW_STAT_SIZE + 24U)   /*1 byte */
#define ICSS_EMAC_FW_STORM_PREVENTION_OFFSET_MC      (ICSS_EMAC_FW_STATISTICS_OFFSET + ICSS_EMAC_FW_STAT_SIZE + 25U)   /*4 bytes, space allocated for future use for Storm prevention enhancements*/
#define ICSS_EMAC_FW_STORM_PREVENTION_OFFSET_UC      (ICSS_EMAC_FW_STATISTICS_OFFSET + ICSS_EMAC_FW_STAT_SIZE + 29U)   /*4 bytes, space allocated for future use for Storm prevention enhancements*/

/* DRAM1 Offsets for Switch*/
#define ICSS_EMAC_FW_P0_COL_QUEUE_DESC_OFFSET    (0x1E64U)      /* collision descriptor of port 0  */
#define ICSS_EMAC_FW_P0_QUEUE_DESC_OFFSET    (0x1E7CU)           /* 4 queue descriptors for port 0 (host receive)  */

#define ICSS_EMAC_FW_INTERFACE_MAC_ADDR_OFFSET              (0x1E58U)      /* Interface MAC Address  */
#define ICSS_EMAC_FW_COLLISION_STATUS_ADDR       (0x1E60U)      /* Collision Status Register, P0: bit 0 is pending flag, bit 1..2 inidicates which queue,  */
                                                       /* P1: bit 8 is pending flag, 9..10 is queue number  */
                                                       /* p2: bit 16 is pending flag, 17..18 is queue number, remaining bits are 0.  */

/*  DRAM Offsets for EMAC. Present on Both DRAM0 and DRAM1 */
/* EMAC Time Triggered Send Base Offset Base, the following offsets are calcuated during init time by the driver, no changes expected in FW:
    used by the driver, do not expect these offsets to change, same for all variants of Firmware 
 *       ICSS_EMAC_TTS_CYCLE_START_OFFSET           (ICSS_EMAC_FW_TTS_BASE_OFFSET)
 *       ICSS_EMAC_TTS_CYCLE_PERIOD_OFFSET          (ICSS_EMAC_TTS_CYCLE_START_OFFSET + 8U)
 *       ICSS_EMAC_TTS_CFG_TIME_OFFSET              (ICSS_EMAC_TTS_CYCLE_PERIOD_OFFSET + 4U)
 *       ICSS_EMAC_TTS_STATUS_OFFSET                (ICSS_EMAC_TTS_CFG_TIME_OFFSET + 4U)
 *       ICSS_EMAC_TTS_MISSED_CYCLE_CNT_OFFSET      (ICSS_EMAC_TTS_STATUS_OFFSET + 4U)
 *       ICSS_EMAC_TTS_PREV_TX_SOF                  (ICSS_EMAC_TTS_MISSED_CYCLE_CNT_OFFSET + 4U)  
 *       ICSS_EMAC_TTS_CYC_TX_SOF                   (ICSS_EMAC_TTS_PREV_TX_SOF + 8U)
*/
#define ICSS_EMAC_FW_TTS_BASE_OFFSET                  (EMAC_SPECIFIC_DRAM_START_OFFSET)



/* Used to calculate end of buffer bd offset, not part of Mmap configuration, local usage only */
#define ICSS_EMAC_FW_HOST_BD_SIZE            ((ICSS_EMAC_FW_HOST_QUEUE_1_SIZE + ICSS_EMAC_FW_HOST_QUEUE_2_SIZE + ICSS_EMAC_FW_HOST_QUEUE_3_SIZE + ICSS_EMAC_FW_HOST_QUEUE_4_SIZE) * ICSS_EMAC_FW_BD_SIZE)
#define ICSS_EMAC_FW_PORT_BD_SIZE            ((ICSS_EMAC_FW_QUEUE_1_SIZE + ICSS_EMAC_FW_QUEUE_2_SIZE + ICSS_EMAC_FW_QUEUE_3_SIZE + ICSS_EMAC_FW_QUEUE_4_SIZE) * 2U * ICSS_EMAC_FW_BD_SIZE)


#define ICSS_EMAC_FW_EOF_BUFFER_BD           (ICSS_EMAC_FW_P0_BUFFER_DESC_OFFSET + ICSS_EMAC_FW_HOST_BD_SIZE + ICSS_EMAC_FW_PORT_BD_SIZE)
#define ICSS_EMAC_FW_P0_COL_BD_OFFSET        (ICSS_EMAC_FW_EOF_BUFFER_BD)

/*  Shared RAM offsets for EMAC, Q2/Q3/Q4 derived from Q1 */
/* Host Port Rx Context OFFSET*/
#define ICSS_EMAC_FW_HOST_Q1_RX_CONTEXT_OFFSET               (ICSS_EMAC_FW_EOF_BUFFER_BD)


//EMAC Firmware Version Information
#define ICSS_EMAC_FW_RESERVED_FEATURE_OFFSET                (ICSS_EMAC_FW_FEATURE_OFFSET + 4U)
/* ICSS_EMAC_FIRMWARE_FEATURE_OFFSET bitmap
Bit 0 1: TTS support
Bit 1 0 : MAC mode 1 : Switch Mode
Bit 2 1: VLAN
Bit 3 : 1 : Storm prevention
Bit 4-Bit 7 : 0: No redundancy 1 : MRP 2 : DLR 3 : HSR 4 : PRP 5 : MRPD
Bit 8-Bit 10 : PTP (0: None 1: IEEE1588 2: PTCP 3: gPTP)
Bit 11 : Reserved
Bit 12-Bit 14 : Number of queues -1
Bit 15: Reserved
Bit 16: 1: PROFINET( CPM/PPM)
Bit 17: 1: DCP filer (PROFINET)
Bit 18: 1: ARP filter (PROFINET)
*/
#define ICSS_EMAC_FW_FEATURE_OFFSET                         (ICSS_EMAC_FW_RELEASE_2_OFFSET + 4U)
#define ICSS_EMAC_FW_RELEASE_2_OFFSET                       (ICSS_EMAC_FW_RELEASE_1_OFFSET + 4U)
#define ICSS_EMAC_FW_RELEASE_1_OFFSET                       (0x0)



/*DRAM1 Offsets for EMAC */
/*table offset for Port queue descriptors: 1 ports * 4 Queues * 2 byte offset = 8 bytes */
#define ICSS_EMAC_FW_PORT_QUEUE_DESC_OFFSET              (EMAC_SPECIFIC_DRAM_START_OFFSET + 32U + 8U)                /* 4 queue descriptors for port tx. 32 bytes  */
#define ICSS_EMAC_FW_EMAC_Q1_TX_CONTEXT_OFFSET           (ICSS_EMAC_FW_PORT_QUEUE_DESC_OFFSET + 32U)

/*DRAM1 Offsets for Switch */
/* TX/RX context offsets */
/* Port 1  */
#define ICSS_EMAC_FW_SWITCH_P1_Q1_TX_CONTEXT_OFFSET            (SWITCH_SPECIFIC_DRAM1_START_OFFSET)

/* Memory Usage of L3 OCMC RAM */
/* L3 64KB Memory - mainly buffer Pool
 * put collision buffer at end of L3 memory. Simplifies PRU coding to be on same memory as queue buffer  
    Note: Buffer queues for port 1 and port 2 are derived from base offset of Port 0*/
#define ICSS_EMAC_FW_HOST_P0_Q1_BUFFER_OFFSET         (0x0000U)


#define ICSS_EMAC_FW_P0_COL_BUFFER_OFFSET        (0xEE00U) /* 1536 byte collision buffer for port 0 send queue  */

#define ICSS_EMAC_FW_TX_QUEUES_BUFFER_OFFSET   (0U)

/* With dynamic configuration of queue size the offsets are variable. For PRU to find which queue and descriptor needs to be served there  */
/* is a look-up table with index of port and queue number.  */
/* table definition to access queue size, buffer descriptor and buffer  */

/* table offset for queue size: 3 ports * 4 Queues * 1 byte offset = 24 bytes  */
#define ICSS_EMAC_DEFAULT_FW_QUEUE_SIZE_ADDR                             (0x1E30U)
/* table offset for queue: 3 ports * 4 Queues * 2 byte offset = 24 bytes  */
#define ICSS_EMAC_DEFAULT_FW_QUEUE_OFFSET_ADDR                              (0x1E18U)
/* table offset for queue descriptors: 3 ports * 4 Queues * 2 byte offset = 24 bytes  */
#define ICSS_EMAC_DEFAULT_FW_QUEUE_DESCRIPTOR_OFFSET_ADDR                (0x1E00U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

ICSS_EMAC_FwStaticMmap icss_emacFwStaticCfg[2] = {
{
    ICSS_EMAC_FW_RELEASE_1_OFFSET ,
    ICSS_EMAC_FW_RELEASE_2_OFFSET ,
    ICSS_EMAC_FW_FEATURE_OFFSET,
    ICSS_EMAC_FW_RESERVED_FEATURE_OFFSET,
    ICSS_EMAC_FW_STATISTICS_OFFSET,
    ICSS_EMAC_FW_STAT_SIZE,
    ICSS_EMAC_FW_STORM_PREVENTION_OFFSET,
    ICSS_EMAC_FW_PHY_SPEED_OFFSET,
    ICSS_EMAC_FW_PORT_STATUS_OFFSET,
    ICSS_EMAC_FW_PORT_CONTROL_ADDR ,
    ICSS_EMAC_FW_PORT_MAC_ADDR,
    ICSS_EMAC_FW_RX_INT_STATUS_OFFSET,
    ICSS_EMAC_FW_STORM_PREVENTION_OFFSET_MC,
    ICSS_EMAC_FW_STORM_PREVENTION_OFFSET_UC,
    ICSS_EMAC_FW_P0_QUEUE_DESC_OFFSET ,
    ICSS_EMAC_FW_P0_COL_QUEUE_DESC_OFFSET,
    ICSS_EMAC_FW_TTS_BASE_OFFSET,
    ICSS_EMAC_FW_INTERFACE_MAC_ADDR_OFFSET,
    ICSS_EMAC_FW_COLLISION_STATUS_ADDR,
    0x0
},
{
    ICSS_EMAC_FW_RELEASE_1_OFFSET ,
    ICSS_EMAC_FW_RELEASE_2_OFFSET ,
    ICSS_EMAC_FW_FEATURE_OFFSET,
    ICSS_EMAC_FW_RESERVED_FEATURE_OFFSET,
    ICSS_EMAC_FW_STATISTICS_OFFSET,
    ICSS_EMAC_FW_STAT_SIZE,
    ICSS_EMAC_FW_STORM_PREVENTION_OFFSET,
    ICSS_EMAC_FW_PHY_SPEED_OFFSET,
    ICSS_EMAC_FW_PORT_STATUS_OFFSET,
    ICSS_EMAC_FW_PORT_CONTROL_ADDR ,
    ICSS_EMAC_FW_PORT_MAC_ADDR,
    ICSS_EMAC_FW_RX_INT_STATUS_OFFSET,
    ICSS_EMAC_FW_STORM_PREVENTION_OFFSET_MC,
    ICSS_EMAC_FW_STORM_PREVENTION_OFFSET_UC,
    ICSS_EMAC_FW_P0_QUEUE_DESC_OFFSET ,
    ICSS_EMAC_FW_P0_COL_QUEUE_DESC_OFFSET,
    ICSS_EMAC_FW_TTS_BASE_OFFSET,
    ICSS_EMAC_FW_INTERFACE_MAC_ADDR_OFFSET,
    ICSS_EMAC_FW_COLLISION_STATUS_ADDR,
    0x0
}
};

ICSS_EMAC_FwDynamicMmap icss_emacFwDynamicCfg = {
    ICSS_EMAC_DEFAULT_FW_QUEUE_SIZE_ADDR,
    ICSS_EMAC_DEFAULT_FW_QUEUE_OFFSET_ADDR,
    ICSS_EMAC_DEFAULT_FW_QUEUE_DESCRIPTOR_OFFSET_ADDR,
    {ICSS_EMAC_FW_QUEUE_1_SIZE, ICSS_EMAC_FW_QUEUE_2_SIZE, ICSS_EMAC_FW_QUEUE_3_SIZE, ICSS_EMAC_FW_QUEUE_4_SIZE,
     0x0, 0x0, 0x0, 0x0,
     0x0, 0x0, 0x0, 0x0,
     0x0, 0x0, 0x0, 0x0},
    {ICSS_EMAC_FW_HOST_QUEUE_1_SIZE, ICSS_EMAC_FW_HOST_QUEUE_2_SIZE, ICSS_EMAC_FW_HOST_QUEUE_3_SIZE, ICSS_EMAC_FW_HOST_QUEUE_4_SIZE,
     0x0, 0x0, 0x0, 0x0,
     0x0, 0x0, 0x0, 0x0,
     0x0, 0x0, 0x0, 0x0},
    ICSS_EMAC_FW_COLLISION_QUEUE_SIZE,
    ICSS_EMAC_FW_P0_BUFFER_DESC_OFFSET,
    ICSS_EMAC_FW_P0_COL_BD_OFFSET ,
    ICSS_EMAC_FW_HOST_P0_Q1_BUFFER_OFFSET,
    ICSS_EMAC_FW_TX_QUEUES_BUFFER_OFFSET,
    ICSS_EMAC_FW_P0_COL_BUFFER_OFFSET,
    ICSS_EMAC_FW_HOST_Q1_RX_CONTEXT_OFFSET,
    ICSS_EMAC_FW_SWITCH_P1_Q1_TX_CONTEXT_OFFSET,
    ICSS_EMAC_FW_PORT_QUEUE_DESC_OFFSET,
    ICSS_EMAC_FW_EMAC_Q1_TX_CONTEXT_OFFSET,
    ICSS_EMAC_NUMBER_OF_QUEUES
};


#ifdef __cplusplus
}
#endif

#endif /* #ifndef ICSS_EMAC_MMAP_H_ */
