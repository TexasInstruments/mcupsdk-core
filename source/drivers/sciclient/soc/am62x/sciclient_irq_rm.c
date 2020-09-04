/*
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
 *  \file am62x/sciclient_irq_rm.c
 *
 *  \brief File containing the AM62x specific interrupt management data for
 *         RM.
 *
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/sciclient/sciclient_rm_priv.h>
#include <drivers/sciclient/soc/am62x/sciclient_irq_rm.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
 static struct Sciclient_rmIaUsedMapping rom_usage_DMASS0_INTAGGR_0[1U] = {
	{
		.event = 30U,
		.cleared = false,
	},
};
 uint8_t vint_usage_count_DMSS_AM62_0_INTAGGR_0[184U]= {0};
 static struct Sciclient_rmIaUsedMapping rom_usage_SA3_SS0_INTAGGR_0[4U] = {
	{
		.event = 20U,
		.cleared = false,
	},
	{
		.event = 21U,
		.cleared = false,
	},
	{
		.event = 22U,
		.cleared = false,
	},
	{
		.event = 23U,
		.cleared = false,
	},
};
 uint8_t vint_usage_count_SA3_SS0_INTAGGR_0[8U]= {0};

struct Sciclient_rmIaInst gRmIaInstances[SCICLIENT_RM_IA_NUM_INST] =
{
    {
        .dev_id             = TISCI_DEV_DMASS0_INTAGGR_0,
        .imap               = 0x48100000,
        .sevt_offset        = 0u,
        .n_sevt             = 1536u,
        .n_vint             = 184,
        .vint_usage_count   = &vint_usage_count_DMSS_AM62_0_INTAGGR_0[0],
        .v0_b0_evt          = 0,
        .rom_usage = &rom_usage_DMASS0_INTAGGR_0[0U],
		.n_rom_usage = 1,
    },
     {
        .dev_id             = TISCI_DEV_SA3_SS0_INTAGGR_0,
        .imap               = 0x44809000,
        .sevt_offset        = 0u,
        .n_sevt             = 100u,
        .n_vint             = 8,
        .vint_usage_count   = &vint_usage_count_SA3_SS0_INTAGGR_0[0],
        .v0_b0_evt          = 0,
        .rom_usage = &rom_usage_SA3_SS0_INTAGGR_0[0U],
		.n_rom_usage = 4,
    },
};

struct Sciclient_rmIrInst gRmIrInstances[SCICLIENT_RM_IR_NUM_INST] =
{
    {
        .dev_id         = TISCI_DEV_CMP_EVENT_INTROUTER0,
        .cfg            = 0xa30000,
        .n_inp          = 32u,
        .n_outp         = 42u,
        .inp0_mapping   = 0,
        .rom_usage      = NULL,
        .n_rom_usage    = 0U,
    },
    {
        .dev_id         = TISCI_DEV_MAIN_GPIOMUX_INTROUTER0,
        .cfg            = 0xa00000,
        .n_inp          = 200u,
        .n_outp         = 36u,
        .inp0_mapping   = 0,
        .rom_usage      = NULL,
        .n_rom_usage    = 0U,
    },
    {
        .dev_id         = TISCI_DEV_WKUP_MCU_GPIOMUX_INTROUTER0,
        .cfg            = 0x4210000,
        .n_inp          = 32u,
        .n_outp         = 13u,
        .inp0_mapping   = 0,
        .rom_usage      = NULL,
        .n_rom_usage    = 0U,
    },
    {
        .dev_id         = TISCI_DEV_TIMESYNC_EVENT_ROUTER0,
        .cfg            = 0xa40000,
        .n_inp          = 20u,
        .n_outp         = 26u,
        .inp0_mapping   = 0,
        .rom_usage      = NULL,
        .n_rom_usage    = 0U,
    },
};

