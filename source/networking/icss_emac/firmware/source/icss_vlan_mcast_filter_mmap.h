/******************************************************************************
 *
 *
 * TEXAS INSTRUMENTS TEXT FILE LICENSE
 *
 *  Copyright (c) 2024 Texas Instruments Incorporated
 *
 * All rights reserved not granted herein.
 *
 * Limited License.
 *
 * Texas Instruments Incorporated grants a world-wide, royalty-free, non-exclusive
 * license under copyrights and patents it now or hereafter owns or controls to
 * make, have made, use, import, offer to sell and sell ("Utilize") this software
 * subject to the terms herein.  With respect to the foregoing patent license,
 * such license is granted  solely to the extent that any such patent is necessary
 * to Utilize the software alone.  The patent license shall not apply to any
 * combinations which include this software, other than combinations with devices
 * manufactured by or for TI ("TI Devices").  No hardware patent is licensed hereunder.
 *
 * Redistributions must preserve existing copyright notices and reproduce this license
 * (including the above copyright notice and the disclaimer and (if applicable) source
 * code license limitations below) in the documentation and/or other materials provided
 * with the distribution.
 *
 * Redistribution and use in binary form, without modification, are permitted provided
 * that the following conditions are met:
 *	No reverse engineering, decompilation, or disassembly of this software is
 *  permitted with respect to any software provided in binary form.
 *	Any redistribution and use are licensed by TI for use only with TI Devices.
 *	Nothing shall obligate TI to provide you with source code for the software
 *  licensed and provided to you in object code.
 *
 * If software source code is provided to you, modification and redistribution of the
 * source code are permitted provided that the following conditions are met:
 *	Any redistribution and use of the source code, including any resulting derivative
 *  works, are licensed by TI for use only with TI Devices.
 *	Any redistribution and use of any object code compiled from the source code
 *  and any resulting derivative works, are licensed by TI for use only with TI Devices.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of its suppliers
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * DISCLAIMER.
 *
 * THIS SOFTWARE IS PROVIDED BY TI AND TI’S LICENSORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL TI AND TI’S
 * LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * file : icss_vlan_mcast_filter_mmap.h
 *
 * brief: This file contains VLAN/Multicast filtering feature memory map
 *
 */
#ifndef ICSS_VLAN_MULTICAST_FILTER_MM_H
#define ICSS_VLAN_MULTICAST_FILTER_MM_H

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

#endif /* ICSS_MULTICAST_FILTER_MM_H */
