/*
 * K3 System Firmware Resource Management Configuration Data
 * Auto generated from K3 Resource Partitioning tool
 *
 * Copyright (c) 2022, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 *  \file am62x/sciclient_defaultBoardcfg.c
 *
 *  \brief File containing the tisci_boardcfg default data structure to
 *      send TISCI_MSG_BOARD_CONFIG message.
 *
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/sciclient.h>
#include <drivers/sciclient/include/tisci/am64x_am243x/tisci_hosts.h>
#include <drivers/sciclient/include/tisci/am64x_am243x/tisci_boardcfg_constraints.h>
#include <drivers/sciclient/include/tisci/am64x_am243x/tisci_devices.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if defined (BUILD_MCU1_0)
const struct tisci_local_rm_boardcfg gBoardConfigLow_rm
__attribute__(( aligned(128), section(".boardcfg_data") )) =
	{
	.rm_boardcfg = {
		.rev = {
			 .tisci_boardcfg_abi_maj = TISCI_BOARDCFG_RM_ABI_MAJ_VALUE,
            .tisci_boardcfg_abi_min = TISCI_BOARDCFG_RM_ABI_MIN_VALUE,
		},
		.host_cfg = {
			.subhdr = {
				.magic = TISCI_BOARDCFG_RM_HOST_CFG_MAGIC_NUM,
				.size = (uint16_t) sizeof(struct tisci_boardcfg_rm_host_cfg),
			},
			.host_cfg_entries = {0},
		},
		.resasg = {
			.subhdr = {
				.magic = TISCI_BOARDCFG_RM_RESASG_MAGIC_NUM,
                .size = (uint16_t) sizeof(struct tisci_boardcfg_rm_resasg),
			},
			.resasg_entries_size = 78 * sizeof(struct tisci_boardcfg_rm_resasg_entry),
		},
	},
	.resasg_entries = {
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_CMP_EVENT_INTROUTER0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
			.start_resource = 0U,
			.num_resource = 42U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_MAIN_GPIOMUX_INTROUTER0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
			.start_resource = 0U,
			.num_resource = 36U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_WKUP_MCU_GPIOMUX_INTROUTER0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
			.start_resource = 0U,
			.num_resource = 13U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_TIMESYNC_EVENT_ROUTER0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
			.start_resource = 0U,
			.num_resource = 26U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_BCDMA_0, TISCI_RESASG_SUBTYPE_RA_ERROR_OES),
			.start_resource = 0U,
			.num_resource = 1U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_BCDMA_0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_TRIGGER),
			.start_resource = 50176U,
			.num_resource = 164U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_BCDMA_0, TISCI_RESASG_SUBTYPE_UDMAP_GLOBAL_CONFIG),
			.start_resource = 0U,
			.num_resource = 1U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_BCDMA_0, TISCI_RESASG_SUBTYPE_BCDMA_RING_BLOCK_COPY_CHAN),
			.start_resource = 0U,
			.num_resource = 32U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_BCDMA_0, TISCI_RESASG_SUBTYPE_BCDMA_RING_SPLIT_TR_RX_CHAN),
			.start_resource = 54U,
			.num_resource = 28U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_BCDMA_0, TISCI_RESASG_SUBTYPE_BCDMA_RING_SPLIT_TR_TX_CHAN),
			.start_resource = 32U,
			.num_resource = 22U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_BCDMA_0, TISCI_RESASG_SUBTYPE_BCDMA_BLOCK_COPY_CHAN),
			.start_resource = 0U,
			.num_resource = 32U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_BCDMA_0, TISCI_RESASG_SUBTYPE_BCDMA_SPLIT_TR_RX_CHAN),
			.start_resource = 0U,
			.num_resource = 28U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_BCDMA_0, TISCI_RESASG_SUBTYPE_BCDMA_SPLIT_TR_TX_CHAN),
			.start_resource = 0U,
			.num_resource = 22U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_VINT),
			.start_resource = 4U,
			.num_resource = 36U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_VINT),
			.start_resource = 44U,
			.num_resource = 92U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT),
			.start_resource = 11U,
			.num_resource = 1525U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_TIMERMGR_EVT_OES),
			.start_resource = 0U,
			.num_resource = 1024U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_PKTDMA_TX_CHAN_ERROR_OES),
			.start_resource = 4096U,
			.num_resource = 29U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_PKTDMA_TX_FLOW_COMPLETION_OES),
			.start_resource = 4608U,
			.num_resource = 99U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_PKTDMA_RX_CHAN_ERROR_OES),
			.start_resource = 5120U,
			.num_resource = 24U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_PKTDMA_RX_FLOW_COMPLETION_OES),
			.start_resource = 5632U,
			.num_resource = 51U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_PKTDMA_RX_FLOW_STARVATION_OES),
			.start_resource = 6144U,
			.num_resource = 51U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_PKTDMA_RX_FLOW_FIREWALL_OES),
			.start_resource = 6656U,
			.num_resource = 51U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_BCDMA_CHAN_ERROR_OES),
			.start_resource = 8192U,
			.num_resource = 32U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_BCDMA_CHAN_DATA_COMPLETION_OES),
			.start_resource = 8704U,
			.num_resource = 32U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_BCDMA_CHAN_RING_COMPLETION_OES),
			.start_resource = 9216U,
			.num_resource = 32U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_BCDMA_TX_CHAN_ERROR_OES),
			.start_resource = 9728U,
			.num_resource = 22U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_BCDMA_TX_CHAN_DATA_COMPLETION_OES),
			.start_resource = 10240U,
			.num_resource = 22U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_BCDMA_TX_CHAN_RING_COMPLETION_OES),
			.start_resource = 10752U,
			.num_resource = 22U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_BCDMA_RX_CHAN_ERROR_OES),
			.start_resource = 11264U,
			.num_resource = 28U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_BCDMA_RX_CHAN_DATA_COMPLETION_OES),
			.start_resource = 11776U,
			.num_resource = 28U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_BCDMA_RX_CHAN_RING_COMPLETION_OES),
			.start_resource = 12288U,
			.num_resource = 28U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_RA_ERROR_OES),
			.start_resource = 0U,
			.num_resource = 1U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_UDMAP_GLOBAL_CONFIG),
			.start_resource = 0U,
			.num_resource = 1U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_RING_UNMAPPED_TX_CHAN),
			.start_resource = 0U,
			.num_resource = 19U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_RING_CPSW_TX_CHAN),
			.start_resource = 19U,
			.num_resource = 64U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_RING_SAUL_TX_0_CHAN),
			.start_resource = 83U,
			.num_resource = 8U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_RING_SAUL_TX_1_CHAN),
			.start_resource = 91U,
			.num_resource = 8U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_RING_UNMAPPED_RX_CHAN),
			.start_resource = 99U,
			.num_resource = 19U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_RING_CPSW_RX_CHAN),
			.start_resource = 118U,
			.num_resource = 16U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_RING_SAUL_RX_0_CHAN),
			.start_resource = 134U,
			.num_resource = 8U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_RING_SAUL_RX_1_CHAN),
			.start_resource = 134U,
			.num_resource = 8U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_RING_SAUL_RX_2_CHAN),
			.start_resource = 142U,
			.num_resource = 8U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_RING_SAUL_RX_3_CHAN),
			.start_resource = 142U,
			.num_resource = 8U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_UNMAPPED_TX_CHAN),
			.start_resource = 0U,
			.num_resource = 19U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_CPSW_TX_CHAN),
			.start_resource = 19U,
			.num_resource = 8U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_SAUL_TX_0_CHAN),
			.start_resource = 27U,
			.num_resource = 1U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_SAUL_TX_1_CHAN),
			.start_resource = 28U,
			.num_resource = 1U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_UNMAPPED_RX_CHAN),
			.start_resource = 0U,
			.num_resource = 19U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_FLOW_UNMAPPED_RX_CHAN),
			.start_resource = 0U,
			.num_resource = 19U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_CPSW_RX_CHAN),
			.start_resource = 19U,
			.num_resource = 1U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_FLOW_CPSW_RX_CHAN),
			.start_resource = 19U,
			.num_resource = 16U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_SAUL_RX_0_CHAN),
			.start_resource = 20U,
			.num_resource = 1U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_FLOW_SAUL_RX_0_CHAN),
			.start_resource = 35U,
			.num_resource = 8U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_SAUL_RX_1_CHAN),
			.start_resource = 21U,
			.num_resource = 1U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_FLOW_SAUL_RX_1_CHAN),
			.start_resource = 35U,
			.num_resource = 8U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_SAUL_RX_2_CHAN),
			.start_resource = 22U,
			.num_resource = 1U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_FLOW_SAUL_RX_2_CHAN),
			.start_resource = 43U,
			.num_resource = 8U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_SAUL_RX_3_CHAN),
			.start_resource = 23U,
			.num_resource = 1U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_FLOW_SAUL_RX_3_CHAN),
			.start_resource = 43U,
			.num_resource = 8U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_RINGACC_0, TISCI_RESASG_SUBTYPE_RA_ERROR_OES),
			.start_resource = 0U,
			.num_resource = 1U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_DMASS0_RINGACC_0, TISCI_RESASG_SUBTYPE_RA_VIRTID),
			.start_resource = 0U,
			.num_resource = 4096U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_SA3_SS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_VINT),
			.start_resource = 0U,
			.num_resource = 8U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_SA3_SS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT),
			.start_resource = 0U,
			.num_resource = 100U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_SA3_SS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_TIMERMGR_EVT_OES),
			.start_resource = 0U,
			.num_resource = 1024U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_SA3_SS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_PKTDMA_TX_CHAN_ERROR_OES),
			.start_resource = 4096U,
			.num_resource = 2U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_SA3_SS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_PKTDMA_TX_FLOW_COMPLETION_OES),
			.start_resource = 4608U,
			.num_resource = 8U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_SA3_SS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_PKTDMA_RX_CHAN_ERROR_OES),
			.start_resource = 5120U,
			.num_resource = 4U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_SA3_SS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_PKTDMA_RX_FLOW_COMPLETION_OES),
			.start_resource = 5632U,
			.num_resource = 16U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_SA3_SS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_PKTDMA_RX_FLOW_STARVATION_OES),
			.start_resource = 6144U,
			.num_resource = 16U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_SA3_SS0_INTAGGR_0, TISCI_RESASG_SUBTYPE_IA_PKTDMA_RX_FLOW_FIREWALL_OES),
			.start_resource = 6656U,
			.num_resource = 16U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_SA3_SS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_RA_ERROR_OES),
			.start_resource = 0U,
			.num_resource = 1U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_SA3_SS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_UDMAP_GLOBAL_CONFIG),
			.start_resource = 0U,
			.num_resource = 1U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_SA3_SS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_RING_SAUL_RX_0_CHAN),
			.start_resource = 8U,
			.num_resource = 8U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_SA3_SS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_FLOW_SAUL_RX_0_CHAN),
			.start_resource = 0U,
			.num_resource = 8U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_SA3_SS0_PKTDMA_0, TISCI_RESASG_SUBTYPE_PKTDMA_FLOW_SAUL_RX_1_CHAN),
			.start_resource = 8U,
			.num_resource = 8U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_SA3_SS0_RINGACC_0, TISCI_RESASG_SUBTYPE_RA_ERROR_OES),
			.start_resource = 0U,
			.num_resource = 1U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
		{
			.type = TISCI_RESASG_UTYPE(TISCI_DEV_SA3_SS0_RINGACC_0, TISCI_RESASG_SUBTYPE_RA_VIRTID),
			.start_resource = 0U,
			.num_resource = 4096U,
			.host_id = TISCI_HOST_ID_MAIN_0_R5_1,
		},
	}
};

#endif
