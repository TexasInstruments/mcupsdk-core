/*
 * Copyright (c) 2024 Texas Instruments Incorporated
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
 *
 * K3 System Firmware Resource Management Configuration Data
 * Auto generated from K3 Resource Partitioning tool
 */
/**
 *  \file sciclient_defaultBoardcfg.c
 *
 *  \brief File containing the boardcfg default data structure to
 *      send TISCI_MSG_BOARD_CONFIG message.
 *
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/sciclient/include/tisci/am65x/tisci_hosts.h>
#include <drivers/sciclient/include/tisci/am65x/tisci_boardcfg_constraints.h>
#include <drivers/sciclient/sciclient_default_boardcfg/am65x/sciclient_defaultBoardcfg.h>
#include <drivers/sciclient/include/tisci/am65x_sr2/tisci_devices.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* \brief Structure to hold the RM board configuration */
struct tisci_local_rm_boardcfg {
    struct tisci_boardcfg_rm      rm_boardcfg;
    /**< Board configuration parameter */
    struct tisci_boardcfg_rm_resasg_entry resasg_entries[TISCI_RESASG_ENTRIES_MAX];
    /**< Resource assignment entries */
};

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
            .host_cfg_entries = {
                {
                    .host_id = TISCI_HOST_ID_R5_0,
                    .allowed_atype = 0b101010,
                    .allowed_qos   = 0xAAAA,
                    .allowed_orderid = 0xAAAAAAAA,
                    .allowed_priority = 0xAAAA,
                    .allowed_sched_priority = 0xAA
                },
                {
                    .host_id = TISCI_HOST_ID_R5_2,
                    .allowed_atype = 0b101010,
                    .allowed_qos   = 0xAAAA,
                    .allowed_orderid = 0xAAAAAAAA,
                    .allowed_priority = 0xAAAA,
                    .allowed_sched_priority = 0xAA
                },
                {
                    .host_id = TISCI_HOST_ID_A53_2,
                    .allowed_atype = 0b101010,
                    .allowed_qos   = 0xAAAA,
                    .allowed_orderid = 0xAAAAAAAA,
                    .allowed_priority = 0xAAAA,
                    .allowed_sched_priority = 0xAA
                },
                {
                    .host_id = TISCI_HOST_ID_A53_3,
                    .allowed_atype = 0b101010,
                    .allowed_qos   = 0xAAAA,
                    .allowed_orderid = 0xAAAAAAAA,
                    .allowed_priority = 0xAAAA,
                    .allowed_sched_priority = 0xAA
                },
            },
        },
        .resasg = {
            .subhdr = {
                .magic = TISCI_BOARDCFG_RM_RESASG_MAGIC_NUM,
                .size = (uint16_t) sizeof(struct tisci_boardcfg_rm_resasg),
            },
            .resasg_entries_size = 260 * sizeof(struct tisci_boardcfg_rm_resasg_entry),
        },
    },
    .resasg_entries = {
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_CMPEVENT_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_CMPEVENT_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 12,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_CMPEVENT_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 16,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_CMPEVENT_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 16,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_CMPEVENT_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 24,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 32,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MAIN2MCU_LVL_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 32,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MAIN2MCU_LVL_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 32,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MAIN2MCU_LVL_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 32,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 24,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MAIN2MCU_PLS_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 24,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MAIN2MCU_PLS_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 24,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MAIN2MCU_PLS_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 24,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 20,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_GPIOMUX_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_GPIOMUX_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 20,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_GPIOMUX_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 24,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_GPIOMUX_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 24,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_GPIOMUX_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 28,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 16,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_TIMESYNC_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 16,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_TIMESYNC_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 16,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_TIMESYNC_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 16,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_TIMESYNC_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 32,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_WKUP_GPIOMUX_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_WKUP_GPIOMUX_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_WKUP_GPIOMUX_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 8,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_WKUP_GPIOMUX_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 8,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_WKUP_GPIOMUX_INTRTR0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 12,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 80,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMASS_INTA0, TISCI_RESASG_SUBTYPE_IA_VINT),
            .start_resource = 16,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 30,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMASS_INTA0, TISCI_RESASG_SUBTYPE_IA_VINT),
            .start_resource = 96,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 50,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMASS_INTA0, TISCI_RESASG_SUBTYPE_IA_VINT),
            .start_resource = 126,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 50,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMASS_INTA0, TISCI_RESASG_SUBTYPE_IA_VINT),
            .start_resource = 126,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 50,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMASS_INTA0, TISCI_RESASG_SUBTYPE_IA_VINT),
            .start_resource = 176,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 30,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMASS_INTA0, TISCI_RESASG_SUBTYPE_IA_VINT),
            .start_resource = 226,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 1024,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMASS_INTA0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT),
            .start_resource = 16,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 512,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMASS_INTA0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT),
            .start_resource = 1040,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 512,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMASS_INTA0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT),
            .start_resource = 1552,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 512,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMASS_INTA0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT),
            .start_resource = 1552,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 512,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMASS_INTA0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT),
            .start_resource = 2064,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 2032,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMASS_INTA0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT),
            .start_resource = 2576,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 64,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_MODSS_INTA0, TISCI_RESASG_SUBTYPE_IA_VINT),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 1024,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_MODSS_INTA0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT),
            .start_resource = 20480,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 64,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_MODSS_INTA1, TISCI_RESASG_SUBTYPE_IA_VINT),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 1024,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_MODSS_INTA1, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT),
            .start_resource = 22528,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 64,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 16,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 40,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 80,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 120,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 120,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 124,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 24,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 128,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_PROXY0, TISCI_RESASG_SUBTYPE_PROXY_PROXIES),
            .start_resource = 1,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_PROXY0, TISCI_RESASG_SUBTYPE_PROXY_PROXIES),
            .start_resource = 13,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 16,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_PROXY0, TISCI_RESASG_SUBTYPE_PROXY_PROXIES),
            .start_resource = 17,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 16,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_PROXY0, TISCI_RESASG_SUBTYPE_PROXY_PROXIES),
            .start_resource = 17,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 16,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_PROXY0, TISCI_RESASG_SUBTYPE_PROXY_PROXIES),
            .start_resource = 33,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 15,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_PROXY0, TISCI_RESASG_SUBTYPE_PROXY_PROXIES),
            .start_resource = 49,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 1,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_ERROR_OES),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 100,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_GP),
            .start_resource = 304,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 50,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_GP),
            .start_resource = 404,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 256,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_GP),
            .start_resource = 454,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 256,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_GP),
            .start_resource = 454,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 32,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_GP),
            .start_resource = 710,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 26,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_GP),
            .start_resource = 742,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 160,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 172,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 172,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 172,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 176,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 52,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 178,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 230,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 32,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 238,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 32,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 238,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 14,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 270,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 18,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 284,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 8,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 20,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 20,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 20,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 24,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 38,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 26,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 64,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 32,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 72,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 32,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 72,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 14,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 104,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 118,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_EXT),
            .start_resource = 120,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_EXT),
            .start_resource = 124,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_EXT),
            .start_resource = 128,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_EXT),
            .start_resource = 128,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_EXT),
            .start_resource = 140,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX_H),
            .start_resource = 154,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX_H),
            .start_resource = 154,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX_H),
            .start_resource = 154,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX_H),
            .start_resource = 154,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX_H),
            .start_resource = 154,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX_H),
            .start_resource = 156,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX_H),
            .start_resource = 156,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX_H),
            .start_resource = 158,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_H),
            .start_resource = 1,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_H),
            .start_resource = 1,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_H),
            .start_resource = 1,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_H),
            .start_resource = 1,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 3,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_H),
            .start_resource = 1,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_H),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_H),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_H),
            .start_resource = 6,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 1,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_VIRTID),
            .start_resource = 2,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 1,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_VIRTID),
            .start_resource = 3,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_MONITORS),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_MONITORS),
            .start_resource = 8,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_MONITORS),
            .start_resource = 8,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_MONITORS),
            .start_resource = 16,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_MONITORS),
            .start_resource = 24,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 64,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_FLOW_COMMON),
            .start_resource = 150,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_FLOW_COMMON),
            .start_resource = 214,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 64,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_FLOW_COMMON),
            .start_resource = 222,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 64,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_FLOW_COMMON),
            .start_resource = 222,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_FLOW_COMMON),
            .start_resource = 286,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 6,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_FLOW_COMMON),
            .start_resource = 294,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 1,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_INVALID_FLOW_OES),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 1024,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_TRIGGER),
            .start_resource = 49152,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 1,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_GLOBAL_CONFIG),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 8,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 20,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 20,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 20,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 24,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 52,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 26,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 78,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 32,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 86,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 32,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 86,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 14,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 118,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 18,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 132,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_HCHAN),
            .start_resource = 2,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_HCHAN),
            .start_resource = 2,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_HCHAN),
            .start_resource = 2,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_HCHAN),
            .start_resource = 2,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_HCHAN),
            .start_resource = 2,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_HCHAN),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_HCHAN),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_HCHAN),
            .start_resource = 6,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 8,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 20,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 20,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 20,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 24,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 38,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 26,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 64,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 32,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 72,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 32,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 72,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 14,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 104,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 118,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_ECHAN),
            .start_resource = 120,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_ECHAN),
            .start_resource = 124,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_ECHAN),
            .start_resource = 128,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_ECHAN),
            .start_resource = 128,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_ECHAN),
            .start_resource = 140,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_HCHAN),
            .start_resource = 1,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_HCHAN),
            .start_resource = 1,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_HCHAN),
            .start_resource = 1,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_HCHAN),
            .start_resource = 1,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 3,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_HCHAN),
            .start_resource = 1,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_HCHAN),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_HCHAN),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_HCHAN),
            .start_resource = 6,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 80,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0, TISCI_RESASG_SUBTYPE_IA_VINT),
            .start_resource = 8,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 30,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0, TISCI_RESASG_SUBTYPE_IA_VINT),
            .start_resource = 88,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 50,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0, TISCI_RESASG_SUBTYPE_IA_VINT),
            .start_resource = 118,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 50,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0, TISCI_RESASG_SUBTYPE_IA_VINT),
            .start_resource = 118,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 50,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0, TISCI_RESASG_SUBTYPE_IA_VINT),
            .start_resource = 168,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 38,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0, TISCI_RESASG_SUBTYPE_IA_VINT),
            .start_resource = 218,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 512,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT),
            .start_resource = 16392,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 128,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT),
            .start_resource = 16904,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 256,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT),
            .start_resource = 17032,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 256,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT),
            .start_resource = 17032,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 256,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT),
            .start_resource = 17288,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 376,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_SEVT),
            .start_resource = 17544,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 28,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 28,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 28,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_INTR_ROUTER_0, TISCI_RESASG_SUBTYPE_IR_OUTPUT),
            .start_resource = 36,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_PROXY0, TISCI_RESASG_SUBTYPE_PROXY_PROXIES),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_PROXY0, TISCI_RESASG_SUBTYPE_PROXY_PROXIES),
            .start_resource = 12,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 24,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_PROXY0, TISCI_RESASG_SUBTYPE_PROXY_PROXIES),
            .start_resource = 16,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 24,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_PROXY0, TISCI_RESASG_SUBTYPE_PROXY_PROXIES),
            .start_resource = 16,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 24,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_PROXY0, TISCI_RESASG_SUBTYPE_PROXY_PROXIES),
            .start_resource = 40,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 16,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_FLOW_COMMON),
            .start_resource = 48,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_FLOW_COMMON),
            .start_resource = 64,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 16,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_FLOW_COMMON),
            .start_resource = 68,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 16,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_FLOW_COMMON),
            .start_resource = 68,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_FLOW_COMMON),
            .start_resource = 84,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_FLOW_COMMON),
            .start_resource = 92,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 1,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_INVALID_FLOW_OES),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 256,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_GLOBAL_EVENT_TRIGGER),
            .start_resource = 56320,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 1,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_GLOBAL_CONFIG),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 2,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 8,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 10,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 22,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 10,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 26,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 10,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 26,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_CHAN),
            .start_resource = 36,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_HCHAN),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_HCHAN),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_HCHAN),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_RX_HCHAN),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 2,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 8,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 10,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 22,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 10,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 26,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 10,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 26,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_CHAN),
            .start_resource = 36,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_HCHAN),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_HCHAN),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_HCHAN),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_UDMAP0, TISCI_RESASG_SUBTYPE_UDMAP_TX_HCHAN),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 1,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_ERROR_OES),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_ALL,
        },
        {
            .num_resource = 32,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_GP),
            .start_resource = 96,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_GP),
            .start_resource = 128,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 60,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_GP),
            .start_resource = 136,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 60,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_GP),
            .start_resource = 136,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 60,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_GP),
            .start_resource = 196,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 50,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 52,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 52,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 52,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 56,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 58,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 70,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 10,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 74,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 10,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 74,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX),
            .start_resource = 84,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 2,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 4,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 8,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 10,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 4,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 22,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 10,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 26,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 10,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 26,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 12,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX),
            .start_resource = 36,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX_H),
            .start_resource = 48,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX_H),
            .start_resource = 48,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX_H),
            .start_resource = 48,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_RX_H),
            .start_resource = 48,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_H),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_H),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 0,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_H),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 2,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_UDMAP_TX_H),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 1,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_VIRTID),
            .start_resource = 2,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 1,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_VIRTID),
            .start_resource = 3,
            .host_id = TISCI_HOST_ID_A53_3,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_MONITORS),
            .start_resource = 0,
            .host_id = TISCI_HOST_ID_A53_2,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_MONITORS),
            .start_resource = 8,
            .host_id = TISCI_HOST_ID_R5_0,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_MONITORS),
            .start_resource = 8,
            .host_id = TISCI_HOST_ID_R5_1,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_MONITORS),
            .start_resource = 16,
            .host_id = TISCI_HOST_ID_R5_2,
        },
        {
            .num_resource = 8,
            .type = TISCI_RESASG_UTYPE (TISCI_DEV_MCU_NAVSS0_RINGACC0, TISCI_RESASG_SUBTYPE_RA_MONITORS),
            .start_resource = 24,
            .host_id = TISCI_HOST_ID_ALL,
        },
    }
};

