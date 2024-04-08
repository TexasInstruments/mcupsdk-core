/******************************************************************************
 *
 *
 * TEXAS INSTRUMENTS TEXT FILE LICENSE
 *
 *  Copyright (c) 2018-2019 Texas Instruments Incorporated
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
 * file : icss_stp_switch.h
 *
 * brief: Definitions and mapping of firmware-based Ethernet switch over PRU
 *        Differs from icss_switch insofar as the switching logic is all
 *        done within the PRU.
 *
 */

#ifndef ICSS_STP_SWITCH_H
#define ICSS_STP_SWITCH_H

/* In PRU1_DMEM */
#define ICSS_EMAC_FW_FDB__CFG__BASE (0x1C00)
#define ICSS_EMAC_FW_FDB__CFG__SIZE_OFFSET (0x0)

/* ========================================================================== */
/*                               FDB Addresses                                */
/* ========================================================================== */
/**
 * FDB Base
 */
#define ICSS_EMAC_FW_FDB__BASE_OFFSET	(0x2000)
#define ICSS_EMAC_FW_FDB__BASE_ADDR	(ICSS_SHARED \
					 + ICSS_EMAC_FW_FDB__BASE_OFFSET)
/**
 * FDB Index Table
 */
#define ICSS_EMAC_FW_FDB__IDX_TBL_OFFSET (0x0)
#define ICSS_EMAC_FW_FDB__IDX_TBL_ENTRY_SIZE (0x4)
#define ICSS_EMAC_FW_FDB__IDX_TBL_NUM_ENTRIES (256)
#define ICSS_EMAC_FW_FDB__IDX_TBL_SIZE (ICSS_EMAC_FW_FDB__IDX_TBL_ENTRY_SIZE \
					* ICSS_EMAC_FW_FDB__IDX_TBL_NUM_ENTRIES)
/* For indexing with an entry size of ENTRY_SIZE */
#define IDX_TBL_SHIFT_VAL (2)

/**
 * FDB MAC Table
 */
#define ICSS_EMAC_FW_FDB__MAC_TBL_OFFSET (ICSS_EMAC_FW_FDB__IDX_TBL_OFFSET \
					  + ICSS_EMAC_FW_FDB__IDX_TBL_SIZE)
#define ICSS_EMAC_FW_FDB__MAC_TBL_ENTRY_SIZE (0xA)
#define ICSS_EMAC_FW_FDB__MAC_TBL_NUM_ENTRIES (256)
#define ICSS_EMAC_FW_FDB__MAC_TBL_SIZE (ICSS_EMAC_FW_FDB__MAC_TBL_ENTRY_SIZE \
					* ICSS_EMAC_FW_FDB__MAC_TBL_NUM_ENTRIES)
/* For indexing with an entry size of ENTRY_SIZE */
#define MAC_TBL_SHIFT_VAL_1 (3) /*   1 << 3 */
#define MAC_TBL_SHIFT_VAL_2 (1) /* + 1 << 1 */
				/* == 0xA   */

/**
 * FDB STP Port State
 */
#define ICSS_EMAC_FW_FDB__STP_P1_STP_STATE_OFFSET (ICSS_EMAC_FW_FDB__MAC_TBL_OFFSET \
						   + ICSS_EMAC_FW_FDB__MAC_TBL_SIZE)
#define ICSS_EMAC_FW_FDB__STP_P1_STP_STATE_SIZE (0x1)

#define ICSS_EMAC_FW_FDB__STP_P2_STP_STATE_OFFSET (ICSS_EMAC_FW_FDB__STP_P1_STP_STATE_OFFSET \
						   + ICSS_EMAC_FW_FDB__STP_P1_STP_STATE_SIZE)
#define ICSS_EMAC_FW_FDB__STP_P2_STP_STATE_SIZE (0x1)

/**
 * FDB Flood Enable Flags
 */
#define ICSS_EMAC_FW_FDB__FLOOD_ENABLE_FLAGS_OFFSET (ICSS_EMAC_FW_FDB__STP_P2_STP_STATE_OFFSET \
						     + ICSS_EMAC_FW_FDB__STP_P2_STP_STATE_SIZE)
#define ICSS_EMAC_FW_FDB__FLOOD_ENABLE_FLAGS_SIZE (0x1)

/** 
 * FDB Arbitration
 */
#define ICSS_EMAC_FW_FDB__ARBITRATION_OFFSET (ICSS_EMAC_FW_FDB__FLOOD_ENABLE_FLAGS_OFFSET \
					      + ICSS_EMAC_FW_FDB__FLOOD_ENABLE_FLAGS_SIZE)
#define ICSS_EMAC_FW_FDB__HOST_ARBITRATION__LOCAL_OFFSET (0x0)
#define ICSS_EMAC_FW_FDB__PRU_ARBITRATION__LOCAL_OFFSET (0x1)
#define ICSS_EMAC_FW_FDB__HOST_ARBITRATION_SIZE (0x1)
#define ICSS_EMAC_FW_FDB__PRU_ARBITRATION_SIZE (0x1)
#define ICSS_EMAC_FW_FDB__ARBITRATION_SIZE (ICSS_EMAC_FW_FDB__HOST_ARBITRATION_SIZE \
p					    + ICSS_EMAC_FW_FDB__PRU_ARBITRATION_SIZE)

/**
 * FDB PRU Addresses
 */
#define ICSS_EMAC_FW_FDB__IDX_TBL_ADDR		(ICSS_EMAC_FW_FDB__BASE_ADDR \
						 + ICSS_EMAC_FW_FDB__IDX_TBL_OFFSET)
#define ICSS_EMAC_FW_FDB__MAC_TBL_ADDR		(ICSS_EMAC_FW_FDB__BASE_ADDR \
						 + ICSS_EMAC_FW_FDB__MAC_TBL_OFFSET)
#define ICSS_EMAC_FW_FDB__STP_P1_STP_STATE_ADDR	(ICSS_EMAC_FW_FDB__BASE_ADDR \
						 + ICSS_EMAC_FW_FDB__STP_P1_STP_STATE_OFFSET)
#define ICSS_EMAC_FW_FDB__STP_P2_STP_STATE_ADDR	(ICSS_EMAC_FW_FDB__BASE_ADDR \
						 + ICSS_EMAC_FW_FDB__STP_P2_STP_STATE_OFFSET)
#define ICSS_EMAC_FW_FDB__FLOOD_ENABLE_FLAGS_ADDR	(ICSS_EMAC_FW_FDB__BASE_ADDR \
						 + ICSS_EMAC_FW_FDB__FLOOD_ENABLE_FLAGS_OFFSET)
#define ICSS_EMAC_FW_FDB__ARBITRATION_ADDR	(ICSS_EMAC_FW_FDB__BASE_ADDR \
						 + ICSS_EMAC_FW_FDB__ARBITRATION_OFFSET)

/* ========================================================================== */
/*                                Structs                                     */
/* ========================================================================== */
/**
 * FDB MAC Table Entry
 */
#define FDB_MAC_INFO__MAC_ADDR__OFFSET (0)
#define FDB_MAC_INFO__MAC_ADDR__SIZE (6)
#define FDB_MAC_INFO__AGE__OFFSET (FDB_MAC_INFO__MAC_ADDR__OFFSET + FDB_MAC_INFO__MAC_ADDR__SIZE)
#define FDB_MAC_INFO__AGE__SIZE (2)
#define FDB_MAC_INFO__PORT_NO__OFFSET (FDB_MAC_INFO__AGE__OFFSET + FDB_MAC_INFO__AGE__SIZE)
#define FDB_MAC_INFO__PORT_NO__SIZE (1)
#define FDB_MAC_INFO__FLAGS__OFFSET (FDB_MAC_INFO__PORT_NO__OFFSET + FDB_MAC_INFO__PORT_NO__SIZE)
#define FDB_MAC_INFO__FLAGS__SIZE (1)
#define FDB_MAC_INFO__FLAGS__IS_STATIC_BIT (0)
#define FDB_MAC_INFO__FLAGS__ACTIVE_BIT (1)

/* ========================================================================== */
/*                            Constants etc.                                  */
/* ========================================================================== */
/** 
 * FDB Arbitration Locks
 */
#define ICSS_EMAC_FW_FDB__ARBITRATION__HOST_LOCK (0)
#define ICSS_EMAC_FW_FDB__ARBITRATION__PRU_0_LOCK (0)
#define ICSS_EMAC_FW_FDB__ARBITRATION__PRU_1_LOCK (1)

/**
 * STP Port State
 */
#define  STP_STATE_DISABLED   (0U)
#define  STP_STATE_LISTENING  (1U)
#define  STP_STATE_LEARNING   (2U)
#define  STP_STATE_FORWARDING (3U)		  
#define  STP_STATE_BLOCKING   (4U)

#define RSTP_STATE_DISCARDING  STP_STATE_LISTENING /* technically can be
						      Listening or Blocking */
#define RSTP_STATE_LEARNING    STP_STATE_LEARNING
#define RSTP_STATE_FORWARDING  STP_STATE_FORWARDING

/**
 * Buffer Descriptor Flags
 */
#define  FDB_LOOKUP_SUCCESS__BD_BIT   (6U)
#define  PKT_FLOODED__BD_BIT   (7U)

/**
 * Persistent Register R22 Bits
 */
#define FDB_LOOKUP_SUCCESS__R22_BIT  (6U) /* Preserve lookup success in R22 until write to BD */
#define PKT_FLOODED__R22_BIT  (3U) /* Preserve the flood status in R22 until write to BD */

/* STP STATE SHOULD ALWAYS START ON A BYTE BOUNDARY */
/* ~~ STP STATE uses R22 bits 0-2 ~~ */
#define STP_STATE__R22_BYTE R22.b0  /* STP state is in the LS 3 bits of R22.b0 */
#define STP_STATE__R22_MASK (0x7U)  /* .. so read 1 byte and mask 3 bits */                                     
#define STP_STATE__R22_INV_MASK (0xF8U) /* .. inverse mask bc pre-proc doesn't support '~' */
#endif /* ICSS_STP_SWITCH_H */
