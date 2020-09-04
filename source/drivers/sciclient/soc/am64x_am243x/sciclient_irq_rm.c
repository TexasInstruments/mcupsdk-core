/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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
 *  \file am64x_am243x/sciclient_irq_rm.c
 *
 *  \brief File containing the AM64x specific interrupt management data for
 *         RM.
 *
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/sciclient/sciclient_rm_priv.h>
#include <drivers/sciclient/soc/am64x_am243x/sciclient_irq_rm.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint8_t vint_usage_count_DMASS0_INTAGGR_0[184] = {0};
static struct Sciclient_rmIaUsedMapping rom_usage_DMASS0_INTAGGR_0[5U] = {
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
    {
        .event = 30U,
        .cleared = false,
    },
};

struct Sciclient_rmIaInst gRmIaInstances[SCICLIENT_RM_IA_NUM_INST] =
{
    {
        .dev_id             = TISCI_DEV_DMASS0_INTAGGR_0,
        .imap               = 0x48100000,
        .sevt_offset        = 0u,
        .n_sevt             = 1536u,
        .n_vint             = 184,
        .vint_usage_count   = &vint_usage_count_DMASS0_INTAGGR_0[0],
        .v0_b0_evt          = SCICLIENT_RM_IA_GENERIC_EVT_RESETVAL,
        .rom_usage          = &rom_usage_DMASS0_INTAGGR_0[0U],
        .n_rom_usage        = 5,
    },
};

struct Sciclient_rmIrInst gRmIrInstances[SCICLIENT_RM_IR_NUM_INST] =
{
    {
        .dev_id         = TISCI_DEV_CMP_EVENT_INTROUTER0,
        .cfg            = 0xa30000,
        .n_inp          = 87u,
        .n_outp         = 43u,
        .inp0_mapping   = SCICLIENT_RM_IR_MAPPING_FREE,
        .rom_usage      = NULL,
        .n_rom_usage    = 0U,
    },
    {
        .dev_id         = TISCI_DEV_MAIN_GPIOMUX_INTROUTER0,
        .cfg            = 0xa00000,
        .n_inp          = 200u,
        .n_outp         = 54u,
        .inp0_mapping   = SCICLIENT_RM_IR_MAPPING_FREE,
        .rom_usage      = NULL,
        .n_rom_usage    = 0U,
    },
    {
        .dev_id         = TISCI_DEV_MCU_MCU_GPIOMUX_INTROUTER0,
        .cfg            = 0x4210000,
        .n_inp          = 32u,
        .n_outp         = 12u,
        .inp0_mapping   = SCICLIENT_RM_IR_MAPPING_FREE,
        .rom_usage      = NULL,
        .n_rom_usage    = 0U,
    },
    {
        .dev_id         = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
        .cfg            = 0xa40000,
        .n_inp          = 43u,
        .n_outp         = 41u,
        .inp0_mapping   = SCICLIENT_RM_IR_MAPPING_FREE,
        .rom_usage      = NULL,
        .n_rom_usage    = 0U,
    },
};

/* IRQ Tree definition */

/* Start of CMP_EVENT_INTROUTER0 interface definition */
const struct Sciclient_rmIrqIf CMP_EVENT_INTROUTER0_outp_0_15_to_GICSS0_spi_48_63 = {
    .lbase = 0,
    .len = 16,
    .rid = TISCI_DEV_GICSS0,
    .rbase = 48,
};
const struct Sciclient_rmIrqIf CMP_EVENT_INTROUTER0_outp_16_23_to_R5FSS0_CORE0_intr_48_55 = {
    .lbase = 16,
    .len = 8,
    .rid = TISCI_DEV_R5FSS0_CORE0,
    .rbase = 48,
};
const struct Sciclient_rmIrqIf CMP_EVENT_INTROUTER0_outp_16_23_to_R5FSS0_CORE1_intr_48_55 = {
    .lbase = 16,
    .len = 8,
    .rid = TISCI_DEV_R5FSS0_CORE1,
    .rbase = 48,
};
const struct Sciclient_rmIrqIf CMP_EVENT_INTROUTER0_outp_24_31_to_R5FSS1_CORE0_intr_48_55 = {
    .lbase = 24,
    .len = 8,
    .rid = TISCI_DEV_R5FSS1_CORE0,
    .rbase = 48,
};
const struct Sciclient_rmIrqIf CMP_EVENT_INTROUTER0_outp_24_31_to_R5FSS1_CORE1_intr_48_55 = {
    .lbase = 24,
    .len = 8,
    .rid = TISCI_DEV_R5FSS1_CORE1,
    .rbase = 48,
};
const struct Sciclient_rmIrqIf CMP_EVENT_INTROUTER0_outp_32_39_to_DMASS0_INTAGGR_0_intaggr_levi_pend_0_7 = {
    .lbase = 32,
    .len = 8,
    .rid = TISCI_DEV_DMASS0_INTAGGR_0,
    .rbase = 0,
};
const struct Sciclient_rmIrqIf * const tisci_if_CMP_EVENT_INTROUTER0[] = {
    &CMP_EVENT_INTROUTER0_outp_0_15_to_GICSS0_spi_48_63,
    &CMP_EVENT_INTROUTER0_outp_16_23_to_R5FSS0_CORE0_intr_48_55,
    &CMP_EVENT_INTROUTER0_outp_16_23_to_R5FSS0_CORE1_intr_48_55,
    &CMP_EVENT_INTROUTER0_outp_24_31_to_R5FSS1_CORE0_intr_48_55,
    &CMP_EVENT_INTROUTER0_outp_24_31_to_R5FSS1_CORE1_intr_48_55,
    &CMP_EVENT_INTROUTER0_outp_32_39_to_DMASS0_INTAGGR_0_intaggr_levi_pend_0_7,
};
static const struct Sciclient_rmIrqNode tisci_irq_CMP_EVENT_INTROUTER0 = {
    .id = TISCI_DEV_CMP_EVENT_INTROUTER0,
    .n_if = 6,
    .p_if = &tisci_if_CMP_EVENT_INTROUTER0[0],
};

/* Start of MAIN_GPIOMUX_INTROUTER0 interface definition */
const struct Sciclient_rmIrqIf MAIN_GPIOMUX_INTROUTER0_outp_0_15_to_GICSS0_spi_32_47 = {
    .lbase = 0,
    .len = 16,
    .rid = TISCI_DEV_GICSS0,
    .rbase = 32,
};
const struct Sciclient_rmIrqIf MAIN_GPIOMUX_INTROUTER0_outp_0_15_to_R5FSS0_CORE0_intr_32_47 = {
    .lbase = 0,
    .len = 16,
    .rid = TISCI_DEV_R5FSS0_CORE0,
    .rbase = 32,
};
const struct Sciclient_rmIrqIf MAIN_GPIOMUX_INTROUTER0_outp_0_15_to_R5FSS0_CORE1_intr_32_47 = {
    .lbase = 0,
    .len = 16,
    .rid = TISCI_DEV_R5FSS0_CORE1,
    .rbase = 32,
};
const struct Sciclient_rmIrqIf MAIN_GPIOMUX_INTROUTER0_outp_0_15_to_R5FSS1_CORE0_intr_32_47 = {
    .lbase = 0,
    .len = 16,
    .rid = TISCI_DEV_R5FSS1_CORE0,
    .rbase = 32,
};
const struct Sciclient_rmIrqIf MAIN_GPIOMUX_INTROUTER0_outp_0_15_to_R5FSS1_CORE1_intr_32_47 = {
    .lbase = 0,
    .len = 16,
    .rid = TISCI_DEV_R5FSS1_CORE1,
    .rbase = 32,
};
const struct Sciclient_rmIrqIf MAIN_GPIOMUX_INTROUTER0_outp_30_37_to_DMASS0_INTAGGR_0_intaggr_levi_pend_16_23 = {
    .lbase = 30,
    .len = 8,
    .rid = TISCI_DEV_DMASS0_INTAGGR_0,
    .rbase = 16,
};
const struct Sciclient_rmIrqIf MAIN_GPIOMUX_INTROUTER0_outp_16_17_to_DMASS0_INTAGGR_0_intaggr_levi_pend_24_25 = {
    .lbase = 16,
    .len = 2,
    .rid = TISCI_DEV_DMASS0_INTAGGR_0,
    .rbase = 24,
};
const struct Sciclient_rmIrqIf MAIN_GPIOMUX_INTROUTER0_outp_18_23_to_PRU_ICSSG0_pr1_iep0_cap_intr_req_4_9 = {
    .lbase = 18,
    .len = 6,
    .rid = TISCI_DEV_PRU_ICSSG0,
    .rbase = 4,
};
const struct Sciclient_rmIrqIf MAIN_GPIOMUX_INTROUTER0_outp_24_29_to_PRU_ICSSG0_pr1_iep1_cap_intr_req_10_15 = {
    .lbase = 24,
    .len = 6,
    .rid = TISCI_DEV_PRU_ICSSG0,
    .rbase = 10,
};
const struct Sciclient_rmIrqIf MAIN_GPIOMUX_INTROUTER0_outp_38_45_to_PRU_ICSSG0_pr1_slv_intr_46_53 = {
    .lbase = 38,
    .len = 8,
    .rid = TISCI_DEV_PRU_ICSSG0,
    .rbase = 46,
};
const struct Sciclient_rmIrqIf MAIN_GPIOMUX_INTROUTER0_outp_18_23_to_PRU_ICSSG1_pr1_iep0_cap_intr_req_4_9 = {
    .lbase = 18,
    .len = 6,
    .rid = TISCI_DEV_PRU_ICSSG1,
    .rbase = 4,
};
const struct Sciclient_rmIrqIf MAIN_GPIOMUX_INTROUTER0_outp_24_29_to_PRU_ICSSG1_pr1_iep1_cap_intr_req_10_15 = {
    .lbase = 24,
    .len = 6,
    .rid = TISCI_DEV_PRU_ICSSG1,
    .rbase = 10,
};
const struct Sciclient_rmIrqIf MAIN_GPIOMUX_INTROUTER0_outp_46_53_to_PRU_ICSSG1_pr1_slv_intr_46_53 = {
    .lbase = 46,
    .len = 8,
    .rid = TISCI_DEV_PRU_ICSSG1,
    .rbase = 46,
};
const struct Sciclient_rmIrqIf * const tisci_if_MAIN_GPIOMUX_INTROUTER0[] = {
    &MAIN_GPIOMUX_INTROUTER0_outp_0_15_to_GICSS0_spi_32_47,
    &MAIN_GPIOMUX_INTROUTER0_outp_0_15_to_R5FSS0_CORE0_intr_32_47,
    &MAIN_GPIOMUX_INTROUTER0_outp_0_15_to_R5FSS0_CORE1_intr_32_47,
    &MAIN_GPIOMUX_INTROUTER0_outp_0_15_to_R5FSS1_CORE0_intr_32_47,
    &MAIN_GPIOMUX_INTROUTER0_outp_0_15_to_R5FSS1_CORE1_intr_32_47,
    &MAIN_GPIOMUX_INTROUTER0_outp_30_37_to_DMASS0_INTAGGR_0_intaggr_levi_pend_16_23,
    &MAIN_GPIOMUX_INTROUTER0_outp_16_17_to_DMASS0_INTAGGR_0_intaggr_levi_pend_24_25,
    &MAIN_GPIOMUX_INTROUTER0_outp_18_23_to_PRU_ICSSG0_pr1_iep0_cap_intr_req_4_9,
    &MAIN_GPIOMUX_INTROUTER0_outp_24_29_to_PRU_ICSSG0_pr1_iep1_cap_intr_req_10_15,
    &MAIN_GPIOMUX_INTROUTER0_outp_38_45_to_PRU_ICSSG0_pr1_slv_intr_46_53,
    &MAIN_GPIOMUX_INTROUTER0_outp_18_23_to_PRU_ICSSG1_pr1_iep0_cap_intr_req_4_9,
    &MAIN_GPIOMUX_INTROUTER0_outp_24_29_to_PRU_ICSSG1_pr1_iep1_cap_intr_req_10_15,
    &MAIN_GPIOMUX_INTROUTER0_outp_46_53_to_PRU_ICSSG1_pr1_slv_intr_46_53,
};
static const struct Sciclient_rmIrqNode tisci_irq_MAIN_GPIOMUX_INTROUTER0 = {
    .id = TISCI_DEV_MAIN_GPIOMUX_INTROUTER0,
    .n_if = 13,
    .p_if = &tisci_if_MAIN_GPIOMUX_INTROUTER0[0],
};

/* Start of MCU_MCU_GPIOMUX_INTROUTER0 interface definition */
const struct Sciclient_rmIrqIf MCU_MCU_GPIOMUX_INTROUTER0_outp_0_3_to_GICSS0_spi_104_107 = {
    .lbase = 0,
    .len = 4,
    .rid = TISCI_DEV_GICSS0,
    .rbase = 104,
};
const struct Sciclient_rmIrqIf MCU_MCU_GPIOMUX_INTROUTER0_outp_0_3_to_R5FSS0_CORE0_intr_104_107 = {
    .lbase = 0,
    .len = 4,
    .rid = TISCI_DEV_R5FSS0_CORE0,
    .rbase = 104,
};
const struct Sciclient_rmIrqIf MCU_MCU_GPIOMUX_INTROUTER0_outp_0_3_to_R5FSS0_CORE1_intr_104_107 = {
    .lbase = 0,
    .len = 4,
    .rid = TISCI_DEV_R5FSS0_CORE1,
    .rbase = 104,
};
const struct Sciclient_rmIrqIf MCU_MCU_GPIOMUX_INTROUTER0_outp_0_3_to_R5FSS1_CORE0_intr_104_107 = {
    .lbase = 0,
    .len = 4,
    .rid = TISCI_DEV_R5FSS1_CORE0,
    .rbase = 104,
};
const struct Sciclient_rmIrqIf MCU_MCU_GPIOMUX_INTROUTER0_outp_0_3_to_R5FSS1_CORE1_intr_104_107 = {
    .lbase = 0,
    .len = 4,
    .rid = TISCI_DEV_R5FSS1_CORE1,
    .rbase = 104,
};
const struct Sciclient_rmIrqIf MCU_MCU_GPIOMUX_INTROUTER0_outp_4_7_to_MCU_M4FSS0_CORE0_nvic_0_3 = {
    .lbase = 4,
    .len = 4,
    .rid = TISCI_DEV_MCU_M4FSS0_CORE0,
    .rbase = 0,
};
const struct Sciclient_rmIrqIf MCU_MCU_GPIOMUX_INTROUTER0_outp_8_11_to_MCU_ESM0_esm_pls_event0_88_91 = {
    .lbase = 8,
    .len = 4,
    .rid = TISCI_DEV_MCU_ESM0,
    .rbase = 88,
};
const struct Sciclient_rmIrqIf MCU_MCU_GPIOMUX_INTROUTER0_outp_8_11_to_MCU_ESM0_esm_pls_event1_92_95 = {
    .lbase = 8,
    .len = 4,
    .rid = TISCI_DEV_MCU_ESM0,
    .rbase = 92,
};
const struct Sciclient_rmIrqIf MCU_MCU_GPIOMUX_INTROUTER0_outp_8_11_to_MCU_ESM0_esm_pls_event2_96_99 = {
    .lbase = 8,
    .len = 4,
    .rid = TISCI_DEV_MCU_ESM0,
    .rbase = 96,
};
const struct Sciclient_rmIrqIf * const tisci_if_MCU_MCU_GPIOMUX_INTROUTER0[] = {
    &MCU_MCU_GPIOMUX_INTROUTER0_outp_0_3_to_GICSS0_spi_104_107,
    &MCU_MCU_GPIOMUX_INTROUTER0_outp_0_3_to_R5FSS0_CORE0_intr_104_107,
    &MCU_MCU_GPIOMUX_INTROUTER0_outp_0_3_to_R5FSS0_CORE1_intr_104_107,
    &MCU_MCU_GPIOMUX_INTROUTER0_outp_0_3_to_R5FSS1_CORE0_intr_104_107,
    &MCU_MCU_GPIOMUX_INTROUTER0_outp_0_3_to_R5FSS1_CORE1_intr_104_107,
    &MCU_MCU_GPIOMUX_INTROUTER0_outp_4_7_to_MCU_M4FSS0_CORE0_nvic_0_3,
    &MCU_MCU_GPIOMUX_INTROUTER0_outp_8_11_to_MCU_ESM0_esm_pls_event0_88_91,
    &MCU_MCU_GPIOMUX_INTROUTER0_outp_8_11_to_MCU_ESM0_esm_pls_event1_92_95,
    &MCU_MCU_GPIOMUX_INTROUTER0_outp_8_11_to_MCU_ESM0_esm_pls_event2_96_99,
};
static const struct Sciclient_rmIrqNode tisci_irq_MCU_MCU_GPIOMUX_INTROUTER0 = {
    .id = TISCI_DEV_MCU_MCU_GPIOMUX_INTROUTER0,
    .n_if = 9,
    .p_if = &tisci_if_MCU_MCU_GPIOMUX_INTROUTER0[0],
};

/* Start of TIMESYNC_EVENT_INTROUTER0 interface definition */
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_0_7_to_DMASS0_INTAGGR_0_intaggr_levi_pend_8_15 = {
    .lbase = 0,
    .len = 8,
    .rid = TISCI_DEV_DMASS0_INTAGGR_0,
    .rbase = 8,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_8_8_to_PRU_ICSSG0_pr1_edc0_latch0_in_0_0 = {
    .lbase = 8,
    .len = 1,
    .rid = TISCI_DEV_PRU_ICSSG0,
    .rbase = 0,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_9_9_to_PRU_ICSSG0_pr1_edc0_latch1_in_1_1 = {
    .lbase = 9,
    .len = 1,
    .rid = TISCI_DEV_PRU_ICSSG0,
    .rbase = 1,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_10_10_to_PRU_ICSSG0_pr1_edc1_latch0_in_2_2 = {
    .lbase = 10,
    .len = 1,
    .rid = TISCI_DEV_PRU_ICSSG0,
    .rbase = 2,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_11_11_to_PRU_ICSSG0_pr1_edc1_latch1_in_3_3 = {
    .lbase = 11,
    .len = 1,
    .rid = TISCI_DEV_PRU_ICSSG0,
    .rbase = 3,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_12_12_to_PRU_ICSSG1_pr1_edc0_latch0_in_0_0 = {
    .lbase = 12,
    .len = 1,
    .rid = TISCI_DEV_PRU_ICSSG1,
    .rbase = 0,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_13_13_to_PRU_ICSSG1_pr1_edc0_latch1_in_1_1 = {
    .lbase = 13,
    .len = 1,
    .rid = TISCI_DEV_PRU_ICSSG1,
    .rbase = 1,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_14_14_to_PRU_ICSSG1_pr1_edc1_latch0_in_2_2 = {
    .lbase = 14,
    .len = 1,
    .rid = TISCI_DEV_PRU_ICSSG1,
    .rbase = 2,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_15_15_to_PRU_ICSSG1_pr1_edc1_latch1_in_3_3 = {
    .lbase = 15,
    .len = 1,
    .rid = TISCI_DEV_PRU_ICSSG1,
    .rbase = 3,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_16_16_to_CPTS0_cpts_hw1_push_0_0 = {
    .lbase = 16,
    .len = 1,
    .rid = TISCI_DEV_CPTS0,
    .rbase = 0,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_17_17_to_CPTS0_cpts_hw2_push_1_1 = {
    .lbase = 17,
    .len = 1,
    .rid = TISCI_DEV_CPTS0,
    .rbase = 1,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_18_18_to_CPTS0_cpts_hw3_push_2_2 = {
    .lbase = 18,
    .len = 1,
    .rid = TISCI_DEV_CPTS0,
    .rbase = 2,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_19_19_to_CPTS0_cpts_hw4_push_3_3 = {
    .lbase = 19,
    .len = 1,
    .rid = TISCI_DEV_CPTS0,
    .rbase = 3,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_20_20_to_CPTS0_cpts_hw5_push_4_4 = {
    .lbase = 20,
    .len = 1,
    .rid = TISCI_DEV_CPTS0,
    .rbase = 4,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_21_21_to_CPTS0_cpts_hw6_push_5_5 = {
    .lbase = 21,
    .len = 1,
    .rid = TISCI_DEV_CPTS0,
    .rbase = 5,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_22_22_to_CPTS0_cpts_hw7_push_6_6 = {
    .lbase = 22,
    .len = 1,
    .rid = TISCI_DEV_CPTS0,
    .rbase = 6,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_23_23_to_CPTS0_cpts_hw8_push_7_7 = {
    .lbase = 23,
    .len = 1,
    .rid = TISCI_DEV_CPTS0,
    .rbase = 7,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_29_29_to_PCIE0_pcie_cpts_hw2_push_0_0 = {
    .lbase = 29,
    .len = 1,
    .rid = TISCI_DEV_PCIE0,
    .rbase = 0,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_30_30_to_CPSW0_cpts_hw1_push_0_0 = {
    .lbase = 30,
    .len = 1,
    .rid = TISCI_DEV_CPSW0,
    .rbase = 0,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_31_31_to_CPSW0_cpts_hw2_push_1_1 = {
    .lbase = 31,
    .len = 1,
    .rid = TISCI_DEV_CPSW0,
    .rbase = 1,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_32_32_to_CPSW0_cpts_hw3_push_2_2 = {
    .lbase = 32,
    .len = 1,
    .rid = TISCI_DEV_CPSW0,
    .rbase = 2,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_33_33_to_CPSW0_cpts_hw4_push_3_3 = {
    .lbase = 33,
    .len = 1,
    .rid = TISCI_DEV_CPSW0,
    .rbase = 3,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_34_34_to_CPSW0_cpts_hw5_push_4_4 = {
    .lbase = 34,
    .len = 1,
    .rid = TISCI_DEV_CPSW0,
    .rbase = 4,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_35_35_to_CPSW0_cpts_hw6_push_5_5 = {
    .lbase = 35,
    .len = 1,
    .rid = TISCI_DEV_CPSW0,
    .rbase = 5,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_36_36_to_CPSW0_cpts_hw7_push_6_6 = {
    .lbase = 36,
    .len = 1,
    .rid = TISCI_DEV_CPSW0,
    .rbase = 6,
};
const struct Sciclient_rmIrqIf TIMESYNC_EVENT_INTROUTER0_outl_37_37_to_CPSW0_cpts_hw8_push_7_7 = {
    .lbase = 37,
    .len = 1,
    .rid = TISCI_DEV_CPSW0,
    .rbase = 7,
};
const struct Sciclient_rmIrqIf * const tisci_if_TIMESYNC_EVENT_INTROUTER0[] = {
    &TIMESYNC_EVENT_INTROUTER0_outl_0_7_to_DMASS0_INTAGGR_0_intaggr_levi_pend_8_15,
    &TIMESYNC_EVENT_INTROUTER0_outl_8_8_to_PRU_ICSSG0_pr1_edc0_latch0_in_0_0,
    &TIMESYNC_EVENT_INTROUTER0_outl_9_9_to_PRU_ICSSG0_pr1_edc0_latch1_in_1_1,
    &TIMESYNC_EVENT_INTROUTER0_outl_10_10_to_PRU_ICSSG0_pr1_edc1_latch0_in_2_2,
    &TIMESYNC_EVENT_INTROUTER0_outl_11_11_to_PRU_ICSSG0_pr1_edc1_latch1_in_3_3,
    &TIMESYNC_EVENT_INTROUTER0_outl_12_12_to_PRU_ICSSG1_pr1_edc0_latch0_in_0_0,
    &TIMESYNC_EVENT_INTROUTER0_outl_13_13_to_PRU_ICSSG1_pr1_edc0_latch1_in_1_1,
    &TIMESYNC_EVENT_INTROUTER0_outl_14_14_to_PRU_ICSSG1_pr1_edc1_latch0_in_2_2,
    &TIMESYNC_EVENT_INTROUTER0_outl_15_15_to_PRU_ICSSG1_pr1_edc1_latch1_in_3_3,
    &TIMESYNC_EVENT_INTROUTER0_outl_16_16_to_CPTS0_cpts_hw1_push_0_0,
    &TIMESYNC_EVENT_INTROUTER0_outl_17_17_to_CPTS0_cpts_hw2_push_1_1,
    &TIMESYNC_EVENT_INTROUTER0_outl_18_18_to_CPTS0_cpts_hw3_push_2_2,
    &TIMESYNC_EVENT_INTROUTER0_outl_19_19_to_CPTS0_cpts_hw4_push_3_3,
    &TIMESYNC_EVENT_INTROUTER0_outl_20_20_to_CPTS0_cpts_hw5_push_4_4,
    &TIMESYNC_EVENT_INTROUTER0_outl_21_21_to_CPTS0_cpts_hw6_push_5_5,
    &TIMESYNC_EVENT_INTROUTER0_outl_22_22_to_CPTS0_cpts_hw7_push_6_6,
    &TIMESYNC_EVENT_INTROUTER0_outl_23_23_to_CPTS0_cpts_hw8_push_7_7,
    &TIMESYNC_EVENT_INTROUTER0_outl_29_29_to_PCIE0_pcie_cpts_hw2_push_0_0,
    &TIMESYNC_EVENT_INTROUTER0_outl_30_30_to_CPSW0_cpts_hw1_push_0_0,
    &TIMESYNC_EVENT_INTROUTER0_outl_31_31_to_CPSW0_cpts_hw2_push_1_1,
    &TIMESYNC_EVENT_INTROUTER0_outl_32_32_to_CPSW0_cpts_hw3_push_2_2,
    &TIMESYNC_EVENT_INTROUTER0_outl_33_33_to_CPSW0_cpts_hw4_push_3_3,
    &TIMESYNC_EVENT_INTROUTER0_outl_34_34_to_CPSW0_cpts_hw5_push_4_4,
    &TIMESYNC_EVENT_INTROUTER0_outl_35_35_to_CPSW0_cpts_hw6_push_5_5,
    &TIMESYNC_EVENT_INTROUTER0_outl_36_36_to_CPSW0_cpts_hw7_push_6_6,
    &TIMESYNC_EVENT_INTROUTER0_outl_37_37_to_CPSW0_cpts_hw8_push_7_7,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMESYNC_EVENT_INTROUTER0 = {
    .id = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .n_if = 26,
    .p_if = &tisci_if_TIMESYNC_EVENT_INTROUTER0[0],
};

/* Start of CPSW0 interface definition */
const struct Sciclient_rmIrqIf CPSW0_cpts_comp_0_0_to_CMP_EVENT_INTROUTER0_in_80_80 = {
    .lbase = 0,
    .len = 1,
    .rid = TISCI_DEV_CMP_EVENT_INTROUTER0,
    .rbase = 80,
};
const struct Sciclient_rmIrqIf CPSW0_cpts_genf0_1_1_to_TIMESYNC_EVENT_INTROUTER0_in_21_21 = {
    .lbase = 1,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 21,
};
const struct Sciclient_rmIrqIf CPSW0_cpts_genf1_2_2_to_TIMESYNC_EVENT_INTROUTER0_in_22_22 = {
    .lbase = 2,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 22,
};
const struct Sciclient_rmIrqIf CPSW0_cpts_sync_3_3_to_TIMESYNC_EVENT_INTROUTER0_in_34_34 = {
    .lbase = 3,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 34,
};
const struct Sciclient_rmIrqIf * const tisci_if_CPSW0[] = {
    &CPSW0_cpts_comp_0_0_to_CMP_EVENT_INTROUTER0_in_80_80,
    &CPSW0_cpts_genf0_1_1_to_TIMESYNC_EVENT_INTROUTER0_in_21_21,
    &CPSW0_cpts_genf1_2_2_to_TIMESYNC_EVENT_INTROUTER0_in_22_22,
    &CPSW0_cpts_sync_3_3_to_TIMESYNC_EVENT_INTROUTER0_in_34_34,
};
static const struct Sciclient_rmIrqNode tisci_irq_CPSW0 = {
    .id = TISCI_DEV_CPSW0,
    .n_if = 4,
    .p_if = &tisci_if_CPSW0[0],
};

/* Start of DMASS0_INTAGGR_0 interface definition */
const struct Sciclient_rmIrqIf DMASS0_INTAGGR_0_intaggr_vintr_pend_0_39_to_GICSS0_spi_64_103 = {
    .lbase = 0,
    .len = 40,
    .rid = TISCI_DEV_GICSS0,
    .rbase = 64,
};
const struct Sciclient_rmIrqIf DMASS0_INTAGGR_0_intaggr_vintr_pend_72_79_to_R5FSS0_CORE0_intr_8_15 = {
    .lbase = 72,
    .len = 8,
    .rid = TISCI_DEV_R5FSS0_CORE0,
    .rbase = 8,
};
const struct Sciclient_rmIrqIf DMASS0_INTAGGR_0_intaggr_vintr_pend_40_71_to_R5FSS0_CORE0_intr_64_95 = {
    .lbase = 40,
    .len = 32,
    .rid = TISCI_DEV_R5FSS0_CORE0,
    .rbase = 64,
};
const struct Sciclient_rmIrqIf DMASS0_INTAGGR_0_intaggr_vintr_pend_80_87_to_R5FSS0_CORE1_intr_8_15 = {
    .lbase = 80,
    .len = 8,
    .rid = TISCI_DEV_R5FSS0_CORE1,
    .rbase = 8,
};
const struct Sciclient_rmIrqIf DMASS0_INTAGGR_0_intaggr_vintr_pend_40_71_to_R5FSS0_CORE1_intr_64_95 = {
    .lbase = 40,
    .len = 32,
    .rid = TISCI_DEV_R5FSS0_CORE1,
    .rbase = 64,
};
const struct Sciclient_rmIrqIf DMASS0_INTAGGR_0_intaggr_vintr_pend_120_127_to_R5FSS1_CORE0_intr_8_15 = {
    .lbase = 120,
    .len = 8,
    .rid = TISCI_DEV_R5FSS1_CORE0,
    .rbase = 8,
};
const struct Sciclient_rmIrqIf DMASS0_INTAGGR_0_intaggr_vintr_pend_88_119_to_R5FSS1_CORE0_intr_64_95 = {
    .lbase = 88,
    .len = 32,
    .rid = TISCI_DEV_R5FSS1_CORE0,
    .rbase = 64,
};
const struct Sciclient_rmIrqIf DMASS0_INTAGGR_0_intaggr_vintr_pend_128_135_to_R5FSS1_CORE1_intr_8_15 = {
    .lbase = 128,
    .len = 8,
    .rid = TISCI_DEV_R5FSS1_CORE1,
    .rbase = 8,
};
const struct Sciclient_rmIrqIf DMASS0_INTAGGR_0_intaggr_vintr_pend_88_119_to_R5FSS1_CORE1_intr_64_95 = {
    .lbase = 88,
    .len = 32,
    .rid = TISCI_DEV_R5FSS1_CORE1,
    .rbase = 64,
};
const struct Sciclient_rmIrqIf DMASS0_INTAGGR_0_intaggr_vintr_pend_152_159_to_PRU_ICSSG0_pr1_slv_intr_16_23 = {
    .lbase = 152,
    .len = 8,
    .rid = TISCI_DEV_PRU_ICSSG0,
    .rbase = 16,
};
const struct Sciclient_rmIrqIf DMASS0_INTAGGR_0_intaggr_vintr_pend_160_167_to_PRU_ICSSG1_pr1_slv_intr_16_23 = {
    .lbase = 160,
    .len = 8,
    .rid = TISCI_DEV_PRU_ICSSG1,
    .rbase = 16,
};
const struct Sciclient_rmIrqIf DMASS0_INTAGGR_0_intaggr_vintr_pend_168_183_to_MCU_M4FSS0_CORE0_nvic_32_47 = {
    .lbase = 168,
    .len = 16,
    .rid = TISCI_DEV_MCU_M4FSS0_CORE0,
    .rbase = 32,
};
const struct Sciclient_rmIrqIf * const tisci_if_DMASS0_INTAGGR_0[] = {
    &DMASS0_INTAGGR_0_intaggr_vintr_pend_0_39_to_GICSS0_spi_64_103,
    &DMASS0_INTAGGR_0_intaggr_vintr_pend_72_79_to_R5FSS0_CORE0_intr_8_15,
    &DMASS0_INTAGGR_0_intaggr_vintr_pend_40_71_to_R5FSS0_CORE0_intr_64_95,
    &DMASS0_INTAGGR_0_intaggr_vintr_pend_80_87_to_R5FSS0_CORE1_intr_8_15,
    &DMASS0_INTAGGR_0_intaggr_vintr_pend_40_71_to_R5FSS0_CORE1_intr_64_95,
    &DMASS0_INTAGGR_0_intaggr_vintr_pend_120_127_to_R5FSS1_CORE0_intr_8_15,
    &DMASS0_INTAGGR_0_intaggr_vintr_pend_88_119_to_R5FSS1_CORE0_intr_64_95,
    &DMASS0_INTAGGR_0_intaggr_vintr_pend_128_135_to_R5FSS1_CORE1_intr_8_15,
    &DMASS0_INTAGGR_0_intaggr_vintr_pend_88_119_to_R5FSS1_CORE1_intr_64_95,
    &DMASS0_INTAGGR_0_intaggr_vintr_pend_152_159_to_PRU_ICSSG0_pr1_slv_intr_16_23,
    &DMASS0_INTAGGR_0_intaggr_vintr_pend_160_167_to_PRU_ICSSG1_pr1_slv_intr_16_23,
    &DMASS0_INTAGGR_0_intaggr_vintr_pend_168_183_to_MCU_M4FSS0_CORE0_nvic_32_47,
};
static const struct Sciclient_rmIrqNode tisci_irq_DMASS0_INTAGGR_0 = {
    .id = TISCI_DEV_DMASS0_INTAGGR_0,
    .n_if = 12,
    .p_if = &tisci_if_DMASS0_INTAGGR_0[0],
};

/* Start of TIMER0 interface definition */
const struct Sciclient_rmIrqIf TIMER0_timer_pwm_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_0_0 = {
    .lbase = 0,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 0,
};
const struct Sciclient_rmIrqIf * const tisci_if_TIMER0[] = {
    &TIMER0_timer_pwm_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_0_0,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMER0 = {
    .id = TISCI_DEV_TIMER0,
    .n_if = 1,
    .p_if = &tisci_if_TIMER0[0],
};

/* Start of TIMER1 interface definition */
const struct Sciclient_rmIrqIf TIMER1_timer_pwm_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_1_1 = {
    .lbase = 0,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 1,
};
const struct Sciclient_rmIrqIf * const tisci_if_TIMER1[] = {
    &TIMER1_timer_pwm_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_1_1,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMER1 = {
    .id = TISCI_DEV_TIMER1,
    .n_if = 1,
    .p_if = &tisci_if_TIMER1[0],
};

/* Start of TIMER2 interface definition */
const struct Sciclient_rmIrqIf TIMER2_timer_pwm_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_2_2 = {
    .lbase = 0,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 2,
};
const struct Sciclient_rmIrqIf * const tisci_if_TIMER2[] = {
    &TIMER2_timer_pwm_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_2_2,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMER2 = {
    .id = TISCI_DEV_TIMER2,
    .n_if = 1,
    .p_if = &tisci_if_TIMER2[0],
};

/* Start of TIMER3 interface definition */
const struct Sciclient_rmIrqIf TIMER3_timer_pwm_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_3_3 = {
    .lbase = 0,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 3,
};
const struct Sciclient_rmIrqIf * const tisci_if_TIMER3[] = {
    &TIMER3_timer_pwm_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_3_3,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMER3 = {
    .id = TISCI_DEV_TIMER3,
    .n_if = 1,
    .p_if = &tisci_if_TIMER3[0],
};

/* Start of GTC0 interface definition */
const struct Sciclient_rmIrqIf GTC0_gtc_push_event_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_36_36 = {
    .lbase = 0,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 36,
};
const struct Sciclient_rmIrqIf * const tisci_if_GTC0[] = {
    &GTC0_gtc_push_event_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_36_36,
};
static const struct Sciclient_rmIrqNode tisci_irq_GTC0 = {
    .id = TISCI_DEV_GTC0,
    .n_if = 1,
    .p_if = &tisci_if_GTC0[0],
};

/* Start of GPIO0 interface definition */
const struct Sciclient_rmIrqIf GPIO0_gpio_0_89_to_MAIN_GPIOMUX_INTROUTER0_in_0_89 = {
    .lbase = 0,
    .len = 90,
    .rid = TISCI_DEV_MAIN_GPIOMUX_INTROUTER0,
    .rbase = 0,
};
const struct Sciclient_rmIrqIf GPIO0_gpio_bank_90_98_to_MAIN_GPIOMUX_INTROUTER0_in_190_198 = {
    .lbase = 90,
    .len = 9,
    .rid = TISCI_DEV_MAIN_GPIOMUX_INTROUTER0,
    .rbase = 190,
};
const struct Sciclient_rmIrqIf * const tisci_if_GPIO0[] = {
    &GPIO0_gpio_0_89_to_MAIN_GPIOMUX_INTROUTER0_in_0_89,
    &GPIO0_gpio_bank_90_98_to_MAIN_GPIOMUX_INTROUTER0_in_190_198,
};
static const struct Sciclient_rmIrqNode tisci_irq_GPIO0 = {
    .id = TISCI_DEV_GPIO0,
    .n_if = 2,
    .p_if = &tisci_if_GPIO0[0],
};

/* Start of GPIO1 interface definition */
const struct Sciclient_rmIrqIf GPIO1_gpio_0_89_to_MAIN_GPIOMUX_INTROUTER0_in_90_179 = {
    .lbase = 0,
    .len = 90,
    .rid = TISCI_DEV_MAIN_GPIOMUX_INTROUTER0,
    .rbase = 90,
};
const struct Sciclient_rmIrqIf GPIO1_gpio_bank_90_98_to_MAIN_GPIOMUX_INTROUTER0_in_180_188 = {
    .lbase = 90,
    .len = 9,
    .rid = TISCI_DEV_MAIN_GPIOMUX_INTROUTER0,
    .rbase = 180,
};
const struct Sciclient_rmIrqIf * const tisci_if_GPIO1[] = {
    &GPIO1_gpio_0_89_to_MAIN_GPIOMUX_INTROUTER0_in_90_179,
    &GPIO1_gpio_bank_90_98_to_MAIN_GPIOMUX_INTROUTER0_in_180_188,
};
static const struct Sciclient_rmIrqNode tisci_irq_GPIO1 = {
    .id = TISCI_DEV_GPIO1,
    .n_if = 2,
    .p_if = &tisci_if_GPIO1[0],
};

/* Start of MCU_GPIO0 interface definition */
const struct Sciclient_rmIrqIf MCU_GPIO0_gpio_0_29_to_MCU_MCU_GPIOMUX_INTROUTER0_in_0_29 = {
    .lbase = 0,
    .len = 30,
    .rid = TISCI_DEV_MCU_MCU_GPIOMUX_INTROUTER0,
    .rbase = 0,
};
const struct Sciclient_rmIrqIf MCU_GPIO0_gpio_bank_30_31_to_MCU_MCU_GPIOMUX_INTROUTER0_in_30_31 = {
    .lbase = 30,
    .len = 2,
    .rid = TISCI_DEV_MCU_MCU_GPIOMUX_INTROUTER0,
    .rbase = 30,
};
const struct Sciclient_rmIrqIf * const tisci_if_MCU_GPIO0[] = {
    &MCU_GPIO0_gpio_0_29_to_MCU_MCU_GPIOMUX_INTROUTER0_in_0_29,
    &MCU_GPIO0_gpio_bank_30_31_to_MCU_MCU_GPIOMUX_INTROUTER0_in_30_31,
};
static const struct Sciclient_rmIrqNode tisci_irq_MCU_GPIO0 = {
    .id = TISCI_DEV_MCU_GPIO0,
    .n_if = 2,
    .p_if = &tisci_if_MCU_GPIO0[0],
};

/* Start of GPMC0 interface definition */
const struct Sciclient_rmIrqIf GPMC0_gpmc_sdmareq_0_0_to_DMASS0_INTAGGR_0_intaggr_levi_pend_29_29 = {
    .lbase = 0,
    .len = 1,
    .rid = TISCI_DEV_DMASS0_INTAGGR_0,
    .rbase = 29,
};
const struct Sciclient_rmIrqIf * const tisci_if_GPMC0[] = {
    &GPMC0_gpmc_sdmareq_0_0_to_DMASS0_INTAGGR_0_intaggr_levi_pend_29_29,
};
static const struct Sciclient_rmIrqNode tisci_irq_GPMC0 = {
    .id = TISCI_DEV_GPMC0,
    .n_if = 1,
    .p_if = &tisci_if_GPMC0[0],
};

/* Start of PRU_ICSSG0 interface definition */
const struct Sciclient_rmIrqIf PRU_ICSSG0_pr1_edc0_sync0_out_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_25_25 = {
    .lbase = 0,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 25,
};
const struct Sciclient_rmIrqIf PRU_ICSSG0_pr1_edc0_sync1_out_1_1_to_TIMESYNC_EVENT_INTROUTER0_in_26_26 = {
    .lbase = 1,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 26,
};
const struct Sciclient_rmIrqIf PRU_ICSSG0_pr1_edc1_sync0_out_2_2_to_TIMESYNC_EVENT_INTROUTER0_in_27_27 = {
    .lbase = 2,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 27,
};
const struct Sciclient_rmIrqIf PRU_ICSSG0_pr1_edc1_sync1_out_3_3_to_TIMESYNC_EVENT_INTROUTER0_in_28_28 = {
    .lbase = 3,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 28,
};
const struct Sciclient_rmIrqIf PRU_ICSSG0_pr1_host_intr_req_4_11_to_CMP_EVENT_INTROUTER0_in_0_7 = {
    .lbase = 4,
    .len = 8,
    .rid = TISCI_DEV_CMP_EVENT_INTROUTER0,
    .rbase = 0,
};
const struct Sciclient_rmIrqIf PRU_ICSSG0_pr1_iep0_cmp_intr_req_12_27_to_CMP_EVENT_INTROUTER0_in_16_31 = {
    .lbase = 12,
    .len = 16,
    .rid = TISCI_DEV_CMP_EVENT_INTROUTER0,
    .rbase = 16,
};
const struct Sciclient_rmIrqIf PRU_ICSSG0_pr1_iep1_cmp_intr_req_28_43_to_CMP_EVENT_INTROUTER0_in_32_47 = {
    .lbase = 28,
    .len = 16,
    .rid = TISCI_DEV_CMP_EVENT_INTROUTER0,
    .rbase = 32,
};
const struct Sciclient_rmIrqIf * const tisci_if_PRU_ICSSG0[] = {
    &PRU_ICSSG0_pr1_edc0_sync0_out_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_25_25,
    &PRU_ICSSG0_pr1_edc0_sync1_out_1_1_to_TIMESYNC_EVENT_INTROUTER0_in_26_26,
    &PRU_ICSSG0_pr1_edc1_sync0_out_2_2_to_TIMESYNC_EVENT_INTROUTER0_in_27_27,
    &PRU_ICSSG0_pr1_edc1_sync1_out_3_3_to_TIMESYNC_EVENT_INTROUTER0_in_28_28,
    &PRU_ICSSG0_pr1_host_intr_req_4_11_to_CMP_EVENT_INTROUTER0_in_0_7,
    &PRU_ICSSG0_pr1_iep0_cmp_intr_req_12_27_to_CMP_EVENT_INTROUTER0_in_16_31,
    &PRU_ICSSG0_pr1_iep1_cmp_intr_req_28_43_to_CMP_EVENT_INTROUTER0_in_32_47,
};
static const struct Sciclient_rmIrqNode tisci_irq_PRU_ICSSG0 = {
    .id = TISCI_DEV_PRU_ICSSG0,
    .n_if = 7,
    .p_if = &tisci_if_PRU_ICSSG0[0],
};

/* Start of PRU_ICSSG1 interface definition */
const struct Sciclient_rmIrqIf PRU_ICSSG1_pr1_edc0_sync0_out_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_29_29 = {
    .lbase = 0,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 29,
};
const struct Sciclient_rmIrqIf PRU_ICSSG1_pr1_edc0_sync1_out_1_1_to_TIMESYNC_EVENT_INTROUTER0_in_30_30 = {
    .lbase = 1,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 30,
};
const struct Sciclient_rmIrqIf PRU_ICSSG1_pr1_edc1_sync0_out_2_2_to_TIMESYNC_EVENT_INTROUTER0_in_31_31 = {
    .lbase = 2,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 31,
};
const struct Sciclient_rmIrqIf PRU_ICSSG1_pr1_edc1_sync1_out_3_3_to_TIMESYNC_EVENT_INTROUTER0_in_32_32 = {
    .lbase = 3,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 32,
};
const struct Sciclient_rmIrqIf PRU_ICSSG1_pr1_host_intr_req_4_11_to_CMP_EVENT_INTROUTER0_in_8_15 = {
    .lbase = 4,
    .len = 8,
    .rid = TISCI_DEV_CMP_EVENT_INTROUTER0,
    .rbase = 8,
};
const struct Sciclient_rmIrqIf PRU_ICSSG1_pr1_iep0_cmp_intr_req_12_27_to_CMP_EVENT_INTROUTER0_in_48_63 = {
    .lbase = 12,
    .len = 16,
    .rid = TISCI_DEV_CMP_EVENT_INTROUTER0,
    .rbase = 48,
};
const struct Sciclient_rmIrqIf PRU_ICSSG1_pr1_iep1_cmp_intr_req_28_43_to_CMP_EVENT_INTROUTER0_in_64_79 = {
    .lbase = 28,
    .len = 16,
    .rid = TISCI_DEV_CMP_EVENT_INTROUTER0,
    .rbase = 64,
};
const struct Sciclient_rmIrqIf * const tisci_if_PRU_ICSSG1[] = {
    &PRU_ICSSG1_pr1_edc0_sync0_out_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_29_29,
    &PRU_ICSSG1_pr1_edc0_sync1_out_1_1_to_TIMESYNC_EVENT_INTROUTER0_in_30_30,
    &PRU_ICSSG1_pr1_edc1_sync0_out_2_2_to_TIMESYNC_EVENT_INTROUTER0_in_31_31,
    &PRU_ICSSG1_pr1_edc1_sync1_out_3_3_to_TIMESYNC_EVENT_INTROUTER0_in_32_32,
    &PRU_ICSSG1_pr1_host_intr_req_4_11_to_CMP_EVENT_INTROUTER0_in_8_15,
    &PRU_ICSSG1_pr1_iep0_cmp_intr_req_12_27_to_CMP_EVENT_INTROUTER0_in_48_63,
    &PRU_ICSSG1_pr1_iep1_cmp_intr_req_28_43_to_CMP_EVENT_INTROUTER0_in_64_79,
};
static const struct Sciclient_rmIrqNode tisci_irq_PRU_ICSSG1 = {
    .id = TISCI_DEV_PRU_ICSSG1,
    .n_if = 7,
    .p_if = &tisci_if_PRU_ICSSG1[0],
};

/* Start of CPTS0 interface definition */
const struct Sciclient_rmIrqIf CPTS0_cpts_comp_0_0_to_CMP_EVENT_INTROUTER0_in_82_82 = {
    .lbase = 0,
    .len = 1,
    .rid = TISCI_DEV_CMP_EVENT_INTROUTER0,
    .rbase = 82,
};
const struct Sciclient_rmIrqIf CPTS0_cpts_genf0_1_1_to_TIMESYNC_EVENT_INTROUTER0_in_16_16 = {
    .lbase = 1,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 16,
};
const struct Sciclient_rmIrqIf CPTS0_cpts_genf1_2_2_to_TIMESYNC_EVENT_INTROUTER0_in_17_17 = {
    .lbase = 2,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 17,
};
const struct Sciclient_rmIrqIf CPTS0_cpts_genf2_3_3_to_TIMESYNC_EVENT_INTROUTER0_in_18_18 = {
    .lbase = 3,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 18,
};
const struct Sciclient_rmIrqIf CPTS0_cpts_genf3_4_4_to_TIMESYNC_EVENT_INTROUTER0_in_19_19 = {
    .lbase = 4,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 19,
};
const struct Sciclient_rmIrqIf CPTS0_cpts_genf4_5_5_to_TIMESYNC_EVENT_INTROUTER0_in_20_20 = {
    .lbase = 5,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 20,
};
const struct Sciclient_rmIrqIf CPTS0_cpts_genf5_6_6_to_TIMESYNC_EVENT_INTROUTER0_in_24_24 = {
    .lbase = 6,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 24,
};
const struct Sciclient_rmIrqIf CPTS0_cpts_sync_7_7_to_TIMESYNC_EVENT_INTROUTER0_in_35_35 = {
    .lbase = 7,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 35,
};
const struct Sciclient_rmIrqIf * const tisci_if_CPTS0[] = {
    &CPTS0_cpts_comp_0_0_to_CMP_EVENT_INTROUTER0_in_82_82,
    &CPTS0_cpts_genf0_1_1_to_TIMESYNC_EVENT_INTROUTER0_in_16_16,
    &CPTS0_cpts_genf1_2_2_to_TIMESYNC_EVENT_INTROUTER0_in_17_17,
    &CPTS0_cpts_genf2_3_3_to_TIMESYNC_EVENT_INTROUTER0_in_18_18,
    &CPTS0_cpts_genf3_4_4_to_TIMESYNC_EVENT_INTROUTER0_in_19_19,
    &CPTS0_cpts_genf4_5_5_to_TIMESYNC_EVENT_INTROUTER0_in_20_20,
    &CPTS0_cpts_genf5_6_6_to_TIMESYNC_EVENT_INTROUTER0_in_24_24,
    &CPTS0_cpts_sync_7_7_to_TIMESYNC_EVENT_INTROUTER0_in_35_35,
};
static const struct Sciclient_rmIrqNode tisci_irq_CPTS0 = {
    .id = TISCI_DEV_CPTS0,
    .n_if = 8,
    .p_if = &tisci_if_CPTS0[0],
};

/* Start of EPWM0 interface definition */
const struct Sciclient_rmIrqIf EPWM0_epwm_synco_o_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_39_39 = {
    .lbase = 0,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 39,
};
const struct Sciclient_rmIrqIf * const tisci_if_EPWM0[] = {
    &EPWM0_epwm_synco_o_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_39_39,
};
static const struct Sciclient_rmIrqNode tisci_irq_EPWM0 = {
    .id = TISCI_DEV_EPWM0,
    .n_if = 1,
    .p_if = &tisci_if_EPWM0[0],
};

/* Start of EPWM3 interface definition */
const struct Sciclient_rmIrqIf EPWM3_epwm_synco_o_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_40_40 = {
    .lbase = 0,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 40,
};
const struct Sciclient_rmIrqIf * const tisci_if_EPWM3[] = {
    &EPWM3_epwm_synco_o_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_40_40,
};
static const struct Sciclient_rmIrqNode tisci_irq_EPWM3 = {
    .id = TISCI_DEV_EPWM3,
    .n_if = 1,
    .p_if = &tisci_if_EPWM3[0],
};

/* Start of EPWM6 interface definition */
const struct Sciclient_rmIrqIf EPWM6_epwm_synco_o_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_41_41 = {
    .lbase = 0,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 41,
};
const struct Sciclient_rmIrqIf * const tisci_if_EPWM6[] = {
    &EPWM6_epwm_synco_o_0_0_to_TIMESYNC_EVENT_INTROUTER0_in_41_41,
};
static const struct Sciclient_rmIrqNode tisci_irq_EPWM6 = {
    .id = TISCI_DEV_EPWM6,
    .n_if = 1,
    .p_if = &tisci_if_EPWM6[0],
};

/* Start of PCIE0 interface definition */
const struct Sciclient_rmIrqIf PCIE0_pcie_cpts_comp_0_0_to_CMP_EVENT_INTROUTER0_in_81_81 = {
    .lbase = 0,
    .len = 1,
    .rid = TISCI_DEV_CMP_EVENT_INTROUTER0,
    .rbase = 81,
};
const struct Sciclient_rmIrqIf PCIE0_pcie_cpts_genf0_1_1_to_TIMESYNC_EVENT_INTROUTER0_in_23_23 = {
    .lbase = 1,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 23,
};
const struct Sciclient_rmIrqIf PCIE0_pcie_cpts_sync_3_3_to_TIMESYNC_EVENT_INTROUTER0_in_33_33 = {
    .lbase = 3,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 33,
};
const struct Sciclient_rmIrqIf PCIE0_pcie_cpts_hw1_push_2_2_to_TIMESYNC_EVENT_INTROUTER0_in_37_37 = {
    .lbase = 2,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 37,
};
const struct Sciclient_rmIrqIf PCIE0_pcie_ptm_valid_pulse_4_4_to_TIMESYNC_EVENT_INTROUTER0_in_38_38 = {
    .lbase = 4,
    .len = 1,
    .rid = TISCI_DEV_TIMESYNC_EVENT_INTROUTER0,
    .rbase = 38,
};
const struct Sciclient_rmIrqIf * const tisci_if_PCIE0[] = {
    &PCIE0_pcie_cpts_comp_0_0_to_CMP_EVENT_INTROUTER0_in_81_81,
    &PCIE0_pcie_cpts_genf0_1_1_to_TIMESYNC_EVENT_INTROUTER0_in_23_23,
    &PCIE0_pcie_cpts_sync_3_3_to_TIMESYNC_EVENT_INTROUTER0_in_33_33,
    &PCIE0_pcie_cpts_hw1_push_2_2_to_TIMESYNC_EVENT_INTROUTER0_in_37_37,
    &PCIE0_pcie_ptm_valid_pulse_4_4_to_TIMESYNC_EVENT_INTROUTER0_in_38_38,
};
static const struct Sciclient_rmIrqNode tisci_irq_PCIE0 = {
    .id = TISCI_DEV_PCIE0,
    .n_if = 5,
    .p_if = &tisci_if_PCIE0[0],
};


const struct Sciclient_rmIrqNode * const gRmIrqTree[] = {
    &tisci_irq_CMP_EVENT_INTROUTER0,
    &tisci_irq_MAIN_GPIOMUX_INTROUTER0,
    &tisci_irq_MCU_MCU_GPIOMUX_INTROUTER0,
    &tisci_irq_TIMESYNC_EVENT_INTROUTER0,
    &tisci_irq_CPSW0,
    &tisci_irq_DMASS0_INTAGGR_0,
    &tisci_irq_TIMER0,
    &tisci_irq_TIMER1,
    &tisci_irq_TIMER2,
    &tisci_irq_TIMER3,
    &tisci_irq_GTC0,
    &tisci_irq_GPIO0,
    &tisci_irq_GPIO1,
    &tisci_irq_MCU_GPIO0,
    &tisci_irq_GPMC0,
    &tisci_irq_PRU_ICSSG0,
    &tisci_irq_PRU_ICSSG1,
    &tisci_irq_CPTS0,
    &tisci_irq_EPWM0,
    &tisci_irq_EPWM3,
    &tisci_irq_EPWM6,
    &tisci_irq_PCIE0,
};

const uint32_t gRmIrqTreeCount = sizeof(gRmIrqTree)/sizeof(gRmIrqTree[0]);
