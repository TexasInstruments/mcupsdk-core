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
 */
/**
 *  \file sciclient_irq_rm.c
 *
 *  \brief File containing the AM65xx specific interrupt management data for
 *         RM.
 *
 */
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/sciclient/sciclient_rm_priv.h>
#include <drivers/sciclient/soc/am65x/sciclient_irq_rm.h>

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint8_t vint_usage_count_NAVSS0_UDMASS_INTA0[256] = {0};
uint8_t vint_usage_count_NAVSS0_MODSS_INTA0[64] = {0};
uint8_t vint_usage_count_NAVSS0_MODSS_INTA1[64] = {0};
uint8_t vint_usage_count_MCU_NAVSS0_INTR_AGGR_0[256] = {0};
static struct Sciclient_rmIaUsedMapping rom_usage_MCU_NAVSS0_INTR_AGGR_0[3u] = {
    {
        .event = 16404U,
        .cleared = false,
    },
    {
        .event = 16405U,
        .cleared = false,
    },
    {
        .event = 16414U,
        .cleared = false,
    }
};

struct Sciclient_rmIaInst gRmIaInstances[SCICLIENT_RM_IA_NUM_INST] =
{
    {
        .dev_id             = TISCI_DEV_NAVSS0_UDMASS_INTA0,
        .imap               = 0x30940000,
        .sevt_offset        = 0u,
        .n_sevt             = 4608u,
        .n_vint             = 256,
        .vint_usage_count   = &vint_usage_count_NAVSS0_UDMASS_INTA0[0],
        .v0_b0_evt          = SCICLIENT_RM_IA_GENERIC_EVT_RESETVAL,
        .rom_usage          = NULL,
        .n_rom_usage        = 0u,
    },
    {
        .dev_id             = TISCI_DEV_NAVSS0_MODSS_INTA0,
        .imap               = 0x30900000,
        .sevt_offset        = 20480u,
        .n_sevt             = 1024u,
        .n_vint             = 64,
        .vint_usage_count   = &vint_usage_count_NAVSS0_MODSS_INTA0[0],
        .v0_b0_evt          = SCICLIENT_RM_IA_GENERIC_EVT_RESETVAL,
        .rom_usage          = NULL,
        .n_rom_usage        = 0u,
    },
    {
        .dev_id             = TISCI_DEV_NAVSS0_MODSS_INTA1,
        .imap               = 0x30908000,
        .sevt_offset        = 22528u,
        .n_sevt             = 1024u,
        .n_vint             = 64,
        .vint_usage_count   = &vint_usage_count_NAVSS0_MODSS_INTA1[0],
        .v0_b0_evt          = SCICLIENT_RM_IA_GENERIC_EVT_RESETVAL,
        .rom_usage          = NULL,
        .n_rom_usage        = 0u,
    },
    {
        .dev_id             = TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0,
        .imap               = 0x28560000,
        .sevt_offset        = 16384u,
        .n_sevt             = 1536u,
        .n_vint             = 256,
        .vint_usage_count   = &vint_usage_count_MCU_NAVSS0_INTR_AGGR_0[0],
        .v0_b0_evt          = SCICLIENT_RM_IA_GENERIC_EVT_RESETVAL,
        .rom_usage          = &rom_usage_MCU_NAVSS0_INTR_AGGR_0[0U],
        .n_rom_usage        = 3U,
    }
};

static struct Sciclient_rmIrUsedMapping rom_usage_MAIN2MCU_LVL_INTRTR0[2U] = {
    {
        .inp_start = 64U,
        .outp_start = 0U,
        .length = 32U,
        .cleared = false,
    },
    {
        .inp_start = 28U,
        .outp_start = 32U,
        .length = 2U,
        .cleared = false,
    },
};
static struct Sciclient_rmIrUsedMapping rom_usage_mcu_navss0_intr_router_0[1U] = {
    {
        .inp_start = 1U,
        .outp_start = 0U,
        .length = 2U,
        .cleared = false,
    },
};

struct Sciclient_rmIrInst gRmIrInstances[SCICLIENT_RM_IR_NUM_INST] =
{
    {
        .dev_id         = TISCI_DEV_CMPEVENT_INTRTR0,
        .cfg            = 0xa30000,
        .n_inp          = 128u,
        .n_outp         = 32u,
        .inp0_mapping   = SCICLIENT_RM_IR_MAPPING_FREE,
        .rom_usage      = NULL,
        .n_rom_usage    = 0U,
    },
    {
        .dev_id         = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
        .cfg            = 0xa10000,
        .n_inp          = 192u,
        .n_outp         = 64u,
        .inp0_mapping   = SCICLIENT_RM_IR_MAPPING_FREE,
        .rom_usage      = &rom_usage_MAIN2MCU_LVL_INTRTR0[0U],
        .n_rom_usage    = 2,
    },
    {
        .dev_id         = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
        .cfg            = 0xa20000,
        .n_inp          = 79u,
        .n_outp         = 48u,
        .inp0_mapping   = SCICLIENT_RM_IR_MAPPING_FREE,
        .rom_usage      = NULL,
        .n_rom_usage    = 0U,
    },
    {
        .dev_id         = TISCI_DEV_GPIOMUX_INTRTR0,
        .cfg            = 0xa00000,
        .n_inp          = 208u,
        .n_outp         = 32u,
        .inp0_mapping   = SCICLIENT_RM_IR_MAPPING_FREE,
        .rom_usage      = NULL,
        .n_rom_usage    = 0U,
    },
    {
        .dev_id         = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
        .cfg            = 0x310e0000,
        .n_inp          = 440u,
        .n_outp         = 152u,
        .inp0_mapping   = SCICLIENT_RM_IR_MAPPING_FREE,
        .rom_usage      = NULL,
        .n_rom_usage    = 0U,
    },
    {
        .dev_id         = TISCI_DEV_MCU_NAVSS0_INTR_ROUTER_0,
        .cfg            = 0x28540000,
        .n_inp          = 261,
        .n_outp         = 64,
        .inp0_mapping   = SCICLIENT_RM_IR_MAPPING_FREE,
        .rom_usage      = &rom_usage_mcu_navss0_intr_router_0[0U],
        .n_rom_usage    = 1,
    },
    {
        .dev_id         = TISCI_DEV_TIMESYNC_INTRTR0,
        .cfg            = 0xa40000,
        .n_inp          = 48u,
        .n_outp         = 40u,
        .inp0_mapping   = SCICLIENT_RM_IR_MAPPING_FREE,
        .rom_usage      = NULL,
        .n_rom_usage    = 0U,
    },
    {
        .dev_id         = TISCI_DEV_WKUP_GPIOMUX_INTRTR0,
        .cfg            = 0x42200000,
        .n_inp          = 64u,
        .n_outp         = 16u,
        .inp0_mapping   = SCICLIENT_RM_IR_MAPPING_FREE,
        .rom_usage      = NULL,
        .n_rom_usage    = 0U,
    },
};

/* IRQ Tree definition */

/* Start of CAL0 interface definition */
const struct Sciclient_rmIrqIf cal_main_0_bus_int_cal_l_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_11_11 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 11,
};
const struct Sciclient_rmIrqIf *const tisci_if_CAL0[] = {
    &cal_main_0_bus_int_cal_l_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_11_11,
};
static const struct Sciclient_rmIrqNode tisci_irq_CAL0 = {
    .id = TISCI_DEV_CAL0,
    .n_if   = 1,
    .p_if   = &tisci_if_CAL0[0],
};

/* Start of CMPEVENT_INTRTR0 interface definition */
const struct Sciclient_rmIrqIf cmp_event_introuter_main_0_bus_outp_0_15_to_gic500ss_main_0_bus_spi_544_559 = {
    .lbase  = 0,
    .len    = 16,
    .rid    = TISCI_DEV_GIC0,
    .rbase  = 544,
};
const struct Sciclient_rmIrqIf cmp_event_introuter_main_0_bus_outp_24_31_to_pdma_main1_main_0_bus_levent_in_8_15 = {
    .lbase  = 24,
    .len    = 8,
    .rid    = TISCI_DEV_PDMA1,
    .rbase  = 8,
};
const struct Sciclient_rmIrqIf *const tisci_if_CMPEVENT_INTRTR0[] = {
    &cmp_event_introuter_main_0_bus_outp_0_15_to_gic500ss_main_0_bus_spi_544_559,
    &cmp_event_introuter_main_0_bus_outp_24_31_to_pdma_main1_main_0_bus_levent_in_8_15,
};
static const struct Sciclient_rmIrqNode tisci_irq_CMPEVENT_INTRTR0 = {
    .id = TISCI_DEV_CMPEVENT_INTRTR0,
    .n_if   = 2,
    .p_if   = &tisci_if_CMPEVENT_INTRTR0[0],
};

/* Start of MCU_CPSW0 interface definition */
const struct Sciclient_rmIrqIf cpsw_2guss_mcu_0_bus_cpts_comp_6_6_to_cmp_event_introuter_main_0_bus_in_7_7 = {
    .lbase  = 6,
    .len    = 1,
    .rid    = TISCI_DEV_CMPEVENT_INTRTR0,
    .rbase  = 7,
};
const struct Sciclient_rmIrqIf cpsw_2guss_mcu_0_bus_cpts_genf0_3_3_to_timesync_event_introuter_main_0_bus_in_12_12 = {
    .lbase  = 3,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 12,
};
const struct Sciclient_rmIrqIf cpsw_2guss_mcu_0_bus_cpts_genf1_4_4_to_timesync_event_introuter_main_0_bus_in_13_13 = {
    .lbase  = 4,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 13,
};
const struct Sciclient_rmIrqIf cpsw_2guss_mcu_0_bus_cpts_sync_5_5_to_timesync_event_introuter_main_0_bus_in_31_31 = {
    .lbase  = 5,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 31,
};
const struct Sciclient_rmIrqIf *const tisci_if_MCU_CPSW0[] = {
    &cpsw_2guss_mcu_0_bus_cpts_comp_6_6_to_cmp_event_introuter_main_0_bus_in_7_7,
    &cpsw_2guss_mcu_0_bus_cpts_genf0_3_3_to_timesync_event_introuter_main_0_bus_in_12_12,
    &cpsw_2guss_mcu_0_bus_cpts_genf1_4_4_to_timesync_event_introuter_main_0_bus_in_13_13,
    &cpsw_2guss_mcu_0_bus_cpts_sync_5_5_to_timesync_event_introuter_main_0_bus_in_31_31,
};
static const struct Sciclient_rmIrqNode tisci_irq_MCU_CPSW0 = {
    .id = TISCI_DEV_MCU_CPSW0,
    .n_if   = 4,
    .p_if   = &tisci_if_MCU_CPSW0[0],
};

/* Start of DCC0 interface definition */
const struct Sciclient_rmIrqIf dcc_main_0_bus_intr_done_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_120_120 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 120,
};
const struct Sciclient_rmIrqIf *const tisci_if_DCC0[] = {
    &dcc_main_0_bus_intr_done_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_120_120,
};
static const struct Sciclient_rmIrqNode tisci_irq_DCC0 = {
    .id = TISCI_DEV_DCC0,
    .n_if   = 1,
    .p_if   = &tisci_if_DCC0[0],
};

/* Start of DCC1 interface definition */
const struct Sciclient_rmIrqIf dcc_main_1_bus_intr_done_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_121_121 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 121,
};
const struct Sciclient_rmIrqIf *const tisci_if_DCC1[] = {
    &dcc_main_1_bus_intr_done_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_121_121,
};
static const struct Sciclient_rmIrqNode tisci_irq_DCC1 = {
    .id = TISCI_DEV_DCC1,
    .n_if   = 1,
    .p_if   = &tisci_if_DCC1[0],
};

/* Start of DCC2 interface definition */
const struct Sciclient_rmIrqIf dcc_main_2_bus_intr_done_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_122_122 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 122,
};
const struct Sciclient_rmIrqIf *const tisci_if_DCC2[] = {
    &dcc_main_2_bus_intr_done_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_122_122,
};
static const struct Sciclient_rmIrqNode tisci_irq_DCC2 = {
    .id = TISCI_DEV_DCC2,
    .n_if   = 1,
    .p_if   = &tisci_if_DCC2[0],
};

/* Start of DCC3 interface definition */
const struct Sciclient_rmIrqIf dcc_main_3_bus_intr_done_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_123_123 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 123,
};
const struct Sciclient_rmIrqIf *const tisci_if_DCC3[] = {
    &dcc_main_3_bus_intr_done_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_123_123,
};
static const struct Sciclient_rmIrqNode tisci_irq_DCC3 = {
    .id = TISCI_DEV_DCC3,
    .n_if   = 1,
    .p_if   = &tisci_if_DCC3[0],
};

/* Start of DCC4 interface definition */
const struct Sciclient_rmIrqIf dcc_main_4_bus_intr_done_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_124_124 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 124,
};
const struct Sciclient_rmIrqIf *const tisci_if_DCC4[] = {
    &dcc_main_4_bus_intr_done_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_124_124,
};
static const struct Sciclient_rmIrqNode tisci_irq_DCC4 = {
    .id = TISCI_DEV_DCC4,
    .n_if   = 1,
    .p_if   = &tisci_if_DCC4[0],
};

/* Start of DCC5 interface definition */
const struct Sciclient_rmIrqIf dcc_main_5_bus_intr_done_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_125_125 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 125,
};
const struct Sciclient_rmIrqIf *const tisci_if_DCC5[] = {
    &dcc_main_5_bus_intr_done_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_125_125,
};
static const struct Sciclient_rmIrqNode tisci_irq_DCC5 = {
    .id = TISCI_DEV_DCC5,
    .n_if   = 1,
    .p_if   = &tisci_if_DCC5[0],
};

/* Start of DCC6 interface definition */
const struct Sciclient_rmIrqIf dcc_main_6_bus_intr_done_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_126_126 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 126,
};
const struct Sciclient_rmIrqIf *const tisci_if_DCC6[] = {
    &dcc_main_6_bus_intr_done_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_126_126,
};
static const struct Sciclient_rmIrqNode tisci_irq_DCC6 = {
    .id = TISCI_DEV_DCC6,
    .n_if   = 1,
    .p_if   = &tisci_if_DCC6[0],
};

/* Start of DCC7 interface definition */
const struct Sciclient_rmIrqIf dcc_main_7_bus_intr_done_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_127_127 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 127,
};
const struct Sciclient_rmIrqIf *const tisci_if_DCC7[] = {
    &dcc_main_7_bus_intr_done_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_127_127,
};
static const struct Sciclient_rmIrqNode tisci_irq_DCC7 = {
    .id = TISCI_DEV_DCC7,
    .n_if   = 1,
    .p_if   = &tisci_if_DCC7[0],
};

/* Start of DDRSS0 interface definition */
const struct Sciclient_rmIrqIf ddr39ss_gs80_main_0_bus_ddrss_v2h_other_err_lvl_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_10_10 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 10,
};
const struct Sciclient_rmIrqIf *const tisci_if_DDRSS0[] = {
    &ddr39ss_gs80_main_0_bus_ddrss_v2h_other_err_lvl_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_10_10,
};
static const struct Sciclient_rmIrqNode tisci_irq_DDRSS0 = {
    .id = TISCI_DEV_DDRSS0,
    .n_if   = 1,
    .p_if   = &tisci_if_DDRSS0[0],
};

/* Start of TIMER0 interface definition */
const struct Sciclient_rmIrqIf dmtimer_dmc1ms_main_0_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_108_108 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 108,
};
const struct Sciclient_rmIrqIf *const tisci_if_TIMER0[] = {
    &dmtimer_dmc1ms_main_0_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_108_108,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMER0 = {
    .id = TISCI_DEV_TIMER0,
    .n_if   = 1,
    .p_if   = &tisci_if_TIMER0[0],
};

/* Start of TIMER1 interface definition */
const struct Sciclient_rmIrqIf dmtimer_dmc1ms_main_1_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_109_109 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 109,
};
const struct Sciclient_rmIrqIf *const tisci_if_TIMER1[] = {
    &dmtimer_dmc1ms_main_1_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_109_109,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMER1 = {
    .id = TISCI_DEV_TIMER1,
    .n_if   = 1,
    .p_if   = &tisci_if_TIMER1[0],
};

/* Start of TIMER10 interface definition */
const struct Sciclient_rmIrqIf dmtimer_dmc1ms_main_10_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_118_118 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 118,
};
const struct Sciclient_rmIrqIf *const tisci_if_TIMER10[] = {
    &dmtimer_dmc1ms_main_10_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_118_118,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMER10 = {
    .id = TISCI_DEV_TIMER10,
    .n_if   = 1,
    .p_if   = &tisci_if_TIMER10[0],
};

/* Start of TIMER11 interface definition */
const struct Sciclient_rmIrqIf dmtimer_dmc1ms_main_11_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_119_119 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 119,
};
const struct Sciclient_rmIrqIf *const tisci_if_TIMER11[] = {
    &dmtimer_dmc1ms_main_11_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_119_119,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMER11 = {
    .id = TISCI_DEV_TIMER11,
    .n_if   = 1,
    .p_if   = &tisci_if_TIMER11[0],
};

/* Start of TIMER2 interface definition */
const struct Sciclient_rmIrqIf dmtimer_dmc1ms_main_2_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_110_110 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 110,
};
const struct Sciclient_rmIrqIf *const tisci_if_TIMER2[] = {
    &dmtimer_dmc1ms_main_2_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_110_110,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMER2 = {
    .id = TISCI_DEV_TIMER2,
    .n_if   = 1,
    .p_if   = &tisci_if_TIMER2[0],
};

/* Start of TIMER3 interface definition */
const struct Sciclient_rmIrqIf dmtimer_dmc1ms_main_3_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_111_111 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 111,
};
const struct Sciclient_rmIrqIf *const tisci_if_TIMER3[] = {
    &dmtimer_dmc1ms_main_3_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_111_111,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMER3 = {
    .id = TISCI_DEV_TIMER3,
    .n_if   = 1,
    .p_if   = &tisci_if_TIMER3[0],
};

/* Start of TIMER4 interface definition */
const struct Sciclient_rmIrqIf dmtimer_dmc1ms_main_4_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_112_112 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 112,
};
const struct Sciclient_rmIrqIf *const tisci_if_TIMER4[] = {
    &dmtimer_dmc1ms_main_4_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_112_112,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMER4 = {
    .id = TISCI_DEV_TIMER4,
    .n_if   = 1,
    .p_if   = &tisci_if_TIMER4[0],
};

/* Start of TIMER5 interface definition */
const struct Sciclient_rmIrqIf dmtimer_dmc1ms_main_5_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_113_113 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 113,
};
const struct Sciclient_rmIrqIf *const tisci_if_TIMER5[] = {
    &dmtimer_dmc1ms_main_5_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_113_113,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMER5 = {
    .id = TISCI_DEV_TIMER5,
    .n_if   = 1,
    .p_if   = &tisci_if_TIMER5[0],
};

/* Start of TIMER6 interface definition */
const struct Sciclient_rmIrqIf dmtimer_dmc1ms_main_6_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_114_114 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 114,
};
const struct Sciclient_rmIrqIf *const tisci_if_TIMER6[] = {
    &dmtimer_dmc1ms_main_6_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_114_114,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMER6 = {
    .id = TISCI_DEV_TIMER6,
    .n_if   = 1,
    .p_if   = &tisci_if_TIMER6[0],
};

/* Start of TIMER7 interface definition */
const struct Sciclient_rmIrqIf dmtimer_dmc1ms_main_7_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_115_115 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 115,
};
const struct Sciclient_rmIrqIf *const tisci_if_TIMER7[] = {
    &dmtimer_dmc1ms_main_7_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_115_115,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMER7 = {
    .id = TISCI_DEV_TIMER7,
    .n_if   = 1,
    .p_if   = &tisci_if_TIMER7[0],
};

/* Start of TIMER8 interface definition */
const struct Sciclient_rmIrqIf dmtimer_dmc1ms_main_8_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_116_116 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 116,
};
const struct Sciclient_rmIrqIf *const tisci_if_TIMER8[] = {
    &dmtimer_dmc1ms_main_8_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_116_116,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMER8 = {
    .id = TISCI_DEV_TIMER8,
    .n_if   = 1,
    .p_if   = &tisci_if_TIMER8[0],
};

/* Start of TIMER9 interface definition */
const struct Sciclient_rmIrqIf dmtimer_dmc1ms_main_9_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_117_117 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 117,
};
const struct Sciclient_rmIrqIf *const tisci_if_TIMER9[] = {
    &dmtimer_dmc1ms_main_9_bus_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_117_117,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMER9 = {
    .id = TISCI_DEV_TIMER9,
    .n_if   = 1,
    .p_if   = &tisci_if_TIMER9[0],
};

/* Start of ECAP0 interface definition */
const struct Sciclient_rmIrqIf ecap_main_0_bus_ecap_int_0_0_to_main2mcu_pls_introuter_main_0_bus_in_17_17 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 17,
};
const struct Sciclient_rmIrqIf *const tisci_if_ECAP0[] = {
    &ecap_main_0_bus_ecap_int_0_0_to_main2mcu_pls_introuter_main_0_bus_in_17_17,
};
static const struct Sciclient_rmIrqNode tisci_irq_ECAP0 = {
    .id = TISCI_DEV_ECAP0,
    .n_if   = 1,
    .p_if   = &tisci_if_ECAP0[0],
};

/* Start of EHRPWM0 interface definition */
const struct Sciclient_rmIrqIf ehrpwm_main_0_bus_epwm_etint_2_2_to_main2mcu_pls_introuter_main_0_bus_in_2_2 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 2,
};
const struct Sciclient_rmIrqIf ehrpwm_main_0_bus_epwm_tripzint_0_0_to_main2mcu_pls_introuter_main_0_bus_in_8_8 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 8,
};
const struct Sciclient_rmIrqIf *const tisci_if_EHRPWM0[] = {
    &ehrpwm_main_0_bus_epwm_etint_2_2_to_main2mcu_pls_introuter_main_0_bus_in_2_2,
    &ehrpwm_main_0_bus_epwm_tripzint_0_0_to_main2mcu_pls_introuter_main_0_bus_in_8_8,
};
static const struct Sciclient_rmIrqNode tisci_irq_EHRPWM0 = {
    .id = TISCI_DEV_EHRPWM0,
    .n_if   = 2,
    .p_if   = &tisci_if_EHRPWM0[0],
};

/* Start of EHRPWM1 interface definition */
const struct Sciclient_rmIrqIf ehrpwm_main_1_bus_epwm_etint_2_2_to_main2mcu_pls_introuter_main_0_bus_in_3_3 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 3,
};
const struct Sciclient_rmIrqIf ehrpwm_main_1_bus_epwm_tripzint_0_0_to_main2mcu_pls_introuter_main_0_bus_in_9_9 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 9,
};
const struct Sciclient_rmIrqIf *const tisci_if_EHRPWM1[] = {
    &ehrpwm_main_1_bus_epwm_etint_2_2_to_main2mcu_pls_introuter_main_0_bus_in_3_3,
    &ehrpwm_main_1_bus_epwm_tripzint_0_0_to_main2mcu_pls_introuter_main_0_bus_in_9_9,
};
static const struct Sciclient_rmIrqNode tisci_irq_EHRPWM1 = {
    .id = TISCI_DEV_EHRPWM1,
    .n_if   = 2,
    .p_if   = &tisci_if_EHRPWM1[0],
};

/* Start of EHRPWM2 interface definition */
const struct Sciclient_rmIrqIf ehrpwm_main_2_bus_epwm_etint_2_2_to_main2mcu_pls_introuter_main_0_bus_in_4_4 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 4,
};
const struct Sciclient_rmIrqIf ehrpwm_main_2_bus_epwm_tripzint_0_0_to_main2mcu_pls_introuter_main_0_bus_in_10_10 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 10,
};
const struct Sciclient_rmIrqIf *const tisci_if_EHRPWM2[] = {
    &ehrpwm_main_2_bus_epwm_etint_2_2_to_main2mcu_pls_introuter_main_0_bus_in_4_4,
    &ehrpwm_main_2_bus_epwm_tripzint_0_0_to_main2mcu_pls_introuter_main_0_bus_in_10_10,
};
static const struct Sciclient_rmIrqNode tisci_irq_EHRPWM2 = {
    .id = TISCI_DEV_EHRPWM2,
    .n_if   = 2,
    .p_if   = &tisci_if_EHRPWM2[0],
};

/* Start of EHRPWM3 interface definition */
const struct Sciclient_rmIrqIf ehrpwm_main_3_bus_epwm_etint_2_2_to_main2mcu_pls_introuter_main_0_bus_in_5_5 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 5,
};
const struct Sciclient_rmIrqIf ehrpwm_main_3_bus_epwm_tripzint_0_0_to_main2mcu_pls_introuter_main_0_bus_in_11_11 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 11,
};
const struct Sciclient_rmIrqIf *const tisci_if_EHRPWM3[] = {
    &ehrpwm_main_3_bus_epwm_etint_2_2_to_main2mcu_pls_introuter_main_0_bus_in_5_5,
    &ehrpwm_main_3_bus_epwm_tripzint_0_0_to_main2mcu_pls_introuter_main_0_bus_in_11_11,
};
static const struct Sciclient_rmIrqNode tisci_irq_EHRPWM3 = {
    .id = TISCI_DEV_EHRPWM3,
    .n_if   = 2,
    .p_if   = &tisci_if_EHRPWM3[0],
};

/* Start of EHRPWM4 interface definition */
const struct Sciclient_rmIrqIf ehrpwm_main_4_bus_epwm_etint_2_2_to_main2mcu_pls_introuter_main_0_bus_in_6_6 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 6,
};
const struct Sciclient_rmIrqIf ehrpwm_main_4_bus_epwm_tripzint_0_0_to_main2mcu_pls_introuter_main_0_bus_in_12_12 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 12,
};
const struct Sciclient_rmIrqIf *const tisci_if_EHRPWM4[] = {
    &ehrpwm_main_4_bus_epwm_etint_2_2_to_main2mcu_pls_introuter_main_0_bus_in_6_6,
    &ehrpwm_main_4_bus_epwm_tripzint_0_0_to_main2mcu_pls_introuter_main_0_bus_in_12_12,
};
static const struct Sciclient_rmIrqNode tisci_irq_EHRPWM4 = {
    .id = TISCI_DEV_EHRPWM4,
    .n_if   = 2,
    .p_if   = &tisci_if_EHRPWM4[0],
};

/* Start of EHRPWM5 interface definition */
const struct Sciclient_rmIrqIf ehrpwm_main_5_bus_epwm_etint_2_2_to_main2mcu_pls_introuter_main_0_bus_in_7_7 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 7,
};
const struct Sciclient_rmIrqIf ehrpwm_main_5_bus_epwm_tripzint_0_0_to_main2mcu_pls_introuter_main_0_bus_in_13_13 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 13,
};
const struct Sciclient_rmIrqIf *const tisci_if_EHRPWM5[] = {
    &ehrpwm_main_5_bus_epwm_etint_2_2_to_main2mcu_pls_introuter_main_0_bus_in_7_7,
    &ehrpwm_main_5_bus_epwm_tripzint_0_0_to_main2mcu_pls_introuter_main_0_bus_in_13_13,
};
static const struct Sciclient_rmIrqNode tisci_irq_EHRPWM5 = {
    .id = TISCI_DEV_EHRPWM5,
    .n_if   = 2,
    .p_if   = &tisci_if_EHRPWM5[0],
};

/* Start of ELM0 interface definition */
const struct Sciclient_rmIrqIf elm_main_0_bus_elm_porocpsinterrupt_lvl_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_7_7 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 7,
};
const struct Sciclient_rmIrqIf *const tisci_if_ELM0[] = {
    &elm_main_0_bus_elm_porocpsinterrupt_lvl_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_7_7,
};
static const struct Sciclient_rmIrqNode tisci_irq_ELM0 = {
    .id = TISCI_DEV_ELM0,
    .n_if   = 1,
    .p_if   = &tisci_if_ELM0[0],
};

/* Start of MMCSD0 interface definition */
const struct Sciclient_rmIrqIf emmc2sd3ss_gs80_main_0_bus_emmcsdss_intr_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_29_29 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 29,
};
const struct Sciclient_rmIrqIf *const tisci_if_MMCSD0[] = {
    &emmc2sd3ss_gs80_main_0_bus_emmcsdss_intr_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_29_29,
};
static const struct Sciclient_rmIrqNode tisci_irq_MMCSD0 = {
    .id = TISCI_DEV_MMCSD0,
    .n_if   = 1,
    .p_if   = &tisci_if_MMCSD0[0],
};

/* Start of MMCSD1 interface definition */
const struct Sciclient_rmIrqIf emmc4sd3ss_gs80_main_0_bus_emmcsdss_intr_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_28_28 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 28,
};
const struct Sciclient_rmIrqIf *const tisci_if_MMCSD1[] = {
    &emmc4sd3ss_gs80_main_0_bus_emmcsdss_intr_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_28_28,
};
static const struct Sciclient_rmIrqNode tisci_irq_MMCSD1 = {
    .id = TISCI_DEV_MMCSD1,
    .n_if   = 1,
    .p_if   = &tisci_if_MMCSD1[0],
};

/* Start of EQEP0 interface definition */
const struct Sciclient_rmIrqIf eqep_main_0_bus_eqep_int_0_0_to_main2mcu_pls_introuter_main_0_bus_in_14_14 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 14,
};
const struct Sciclient_rmIrqIf *const tisci_if_EQEP0[] = {
    &eqep_main_0_bus_eqep_int_0_0_to_main2mcu_pls_introuter_main_0_bus_in_14_14,
};
static const struct Sciclient_rmIrqNode tisci_irq_EQEP0 = {
    .id = TISCI_DEV_EQEP0,
    .n_if   = 1,
    .p_if   = &tisci_if_EQEP0[0],
};

/* Start of EQEP1 interface definition */
const struct Sciclient_rmIrqIf eqep_main_1_bus_eqep_int_0_0_to_main2mcu_pls_introuter_main_0_bus_in_15_15 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 15,
};
const struct Sciclient_rmIrqIf *const tisci_if_EQEP1[] = {
    &eqep_main_1_bus_eqep_int_0_0_to_main2mcu_pls_introuter_main_0_bus_in_15_15,
};
static const struct Sciclient_rmIrqNode tisci_irq_EQEP1 = {
    .id = TISCI_DEV_EQEP1,
    .n_if   = 1,
    .p_if   = &tisci_if_EQEP1[0],
};

/* Start of EQEP2 interface definition */
const struct Sciclient_rmIrqIf eqep_main_2_bus_eqep_int_0_0_to_main2mcu_pls_introuter_main_0_bus_in_16_16 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 16,
};
const struct Sciclient_rmIrqIf *const tisci_if_EQEP2[] = {
    &eqep_main_2_bus_eqep_int_0_0_to_main2mcu_pls_introuter_main_0_bus_in_16_16,
};
static const struct Sciclient_rmIrqNode tisci_irq_EQEP2 = {
    .id = TISCI_DEV_EQEP2,
    .n_if   = 1,
    .p_if   = &tisci_if_EQEP2[0],
};

/* Start of GPIO0 interface definition */
const struct Sciclient_rmIrqIf gpio_144_main_0_bus_gpio_0_95_to_main_gpiomux_introuter_main_0_bus_in_0_95 = {
    .lbase  = 0,
    .len    = 96,
    .rid    = TISCI_DEV_GPIOMUX_INTRTR0,
    .rbase  = 0,
};
const struct Sciclient_rmIrqIf gpio_144_main_0_bus_gpio_bank_256_261_to_main_gpiomux_introuter_main_0_bus_in_192_197 = {
    .lbase  = 256,
    .len    = 6,
    .rid    = TISCI_DEV_GPIOMUX_INTRTR0,
    .rbase  = 192,
};
const struct Sciclient_rmIrqIf *const tisci_if_GPIO0[] = {
    &gpio_144_main_0_bus_gpio_0_95_to_main_gpiomux_introuter_main_0_bus_in_0_95,
    &gpio_144_main_0_bus_gpio_bank_256_261_to_main_gpiomux_introuter_main_0_bus_in_192_197,
};
static const struct Sciclient_rmIrqNode tisci_irq_GPIO0 = {
    .id = TISCI_DEV_GPIO0,
    .n_if   = 2,
    .p_if   = &tisci_if_GPIO0[0],
};

/* Start of GPIO1 interface definition */
const struct Sciclient_rmIrqIf gpio_144_main_1_bus_gpio_0_89_to_main_gpiomux_introuter_main_0_bus_in_96_185 = {
    .lbase  = 0,
    .len    = 90,
    .rid    = TISCI_DEV_GPIOMUX_INTRTR0,
    .rbase  = 96,
};
const struct Sciclient_rmIrqIf gpio_144_main_1_bus_gpio_bank_256_261_to_main_gpiomux_introuter_main_0_bus_in_200_205 = {
    .lbase  = 256,
    .len    = 6,
    .rid    = TISCI_DEV_GPIOMUX_INTRTR0,
    .rbase  = 200,
};
const struct Sciclient_rmIrqIf *const tisci_if_GPIO1[] = {
    &gpio_144_main_1_bus_gpio_0_89_to_main_gpiomux_introuter_main_0_bus_in_96_185,
    &gpio_144_main_1_bus_gpio_bank_256_261_to_main_gpiomux_introuter_main_0_bus_in_200_205,
};
static const struct Sciclient_rmIrqNode tisci_irq_GPIO1 = {
    .id = TISCI_DEV_GPIO1,
    .n_if   = 2,
    .p_if   = &tisci_if_GPIO1[0],
};

/* Start of WKUP_GPIO0 interface definition */
const struct Sciclient_rmIrqIf gpio_144_wkup_0_bus_gpio_0_55_to_wkup_gpiomux_introuter_wkup_0_bus_in_0_55 = {
    .lbase  = 0,
    .len    = 56,
    .rid    = TISCI_DEV_WKUP_GPIOMUX_INTRTR0,
    .rbase  = 0,
};
const struct Sciclient_rmIrqIf gpio_144_wkup_0_bus_gpio_bank_128_131_to_wkup_gpiomux_introuter_wkup_0_bus_in_60_63 = {
    .lbase  = 128,
    .len    = 4,
    .rid    = TISCI_DEV_WKUP_GPIOMUX_INTRTR0,
    .rbase  = 60,
};
const struct Sciclient_rmIrqIf *const tisci_if_WKUP_GPIO0[] = {
    &gpio_144_wkup_0_bus_gpio_0_55_to_wkup_gpiomux_introuter_wkup_0_bus_in_0_55,
    &gpio_144_wkup_0_bus_gpio_bank_128_131_to_wkup_gpiomux_introuter_wkup_0_bus_in_60_63,
};
static const struct Sciclient_rmIrqNode tisci_irq_WKUP_GPIO0 = {
    .id = TISCI_DEV_WKUP_GPIO0,
    .n_if   = 2,
    .p_if   = &tisci_if_WKUP_GPIO0[0],
};

/* Start of GPMC0 interface definition */
const struct Sciclient_rmIrqIf gpmc_main_0_bus_gpmc_sinterrupt_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_8_8 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 8,
};
const struct Sciclient_rmIrqIf *const tisci_if_GPMC0[] = {
    &gpmc_main_0_bus_gpmc_sinterrupt_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_8_8,
};
static const struct Sciclient_rmIrqNode tisci_irq_GPMC0 = {
    .id = TISCI_DEV_GPMC0,
    .n_if   = 1,
    .p_if   = &tisci_if_GPMC0[0],
};

/* Start of PRU_ICSSG0 interface definition */
const struct Sciclient_rmIrqIf icss_g_main_0_bus_pr1_rx_sof_intr_req_284_285_to_main2mcu_pls_introuter_main_0_bus_in_20_21 = {
    .lbase  = 284,
    .len    = 2,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 20,
};
const struct Sciclient_rmIrqIf icss_g_main_0_bus_pr1_tx_sof_intr_req_302_303_to_main2mcu_pls_introuter_main_0_bus_in_22_23 = {
    .lbase  = 302,
    .len    = 2,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 22,
};
const struct Sciclient_rmIrqIf icss_g_main_0_bus_pr1_host_intr_req_286_293_to_cmp_event_introuter_main_0_bus_in_8_15 = {
    .lbase  = 286,
    .len    = 8,
    .rid    = TISCI_DEV_CMPEVENT_INTRTR0,
    .rbase  = 8,
};
const struct Sciclient_rmIrqIf icss_g_main_0_bus_pr1_iep0_cmp_intr_req_268_283_to_cmp_event_introuter_main_0_bus_in_32_47 = {
    .lbase  = 268,
    .len    = 16,
    .rid    = TISCI_DEV_CMPEVENT_INTRTR0,
    .rbase  = 32,
};
const struct Sciclient_rmIrqIf icss_g_main_0_bus_pr1_iep1_cmp_intr_req_256_261_to_cmp_event_introuter_main_0_bus_in_48_53 = {
    .lbase  = 256,
    .len    = 6,
    .rid    = TISCI_DEV_CMPEVENT_INTRTR0,
    .rbase  = 48,
};
const struct Sciclient_rmIrqIf icss_g_main_0_bus_pr1_iep1_cmp_intr_req_6_15_to_cmp_event_introuter_main_0_bus_in_54_63 = {
    .lbase  = 6,
    .len    = 10,
    .rid    = TISCI_DEV_CMPEVENT_INTRTR0,
    .rbase  = 54,
};
const struct Sciclient_rmIrqIf icss_g_main_0_bus_pr1_edc0_sync0_out_304_304_to_timesync_event_introuter_main_0_bus_in_16_16 = {
    .lbase  = 304,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 16,
};
const struct Sciclient_rmIrqIf icss_g_main_0_bus_pr1_edc0_sync1_out_305_305_to_timesync_event_introuter_main_0_bus_in_17_17 = {
    .lbase  = 305,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 17,
};
const struct Sciclient_rmIrqIf icss_g_main_0_bus_pr1_edc1_sync0_out_306_306_to_timesync_event_introuter_main_0_bus_in_18_18 = {
    .lbase  = 306,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 18,
};
const struct Sciclient_rmIrqIf icss_g_main_0_bus_pr1_edc1_sync1_out_307_307_to_timesync_event_introuter_main_0_bus_in_19_19 = {
    .lbase  = 307,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 19,
};
const struct Sciclient_rmIrqIf icss_g_main_0_bus_pr1_host_intr_pend_294_301_to_main2mcu_lvl_introuter_main_0_bus_in_32_39 = {
    .lbase  = 294,
    .len    = 8,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 32,
};
const struct Sciclient_rmIrqIf *const tisci_if_PRU_ICSSG0[] = {
    &icss_g_main_0_bus_pr1_rx_sof_intr_req_284_285_to_main2mcu_pls_introuter_main_0_bus_in_20_21,
    &icss_g_main_0_bus_pr1_tx_sof_intr_req_302_303_to_main2mcu_pls_introuter_main_0_bus_in_22_23,
    &icss_g_main_0_bus_pr1_host_intr_req_286_293_to_cmp_event_introuter_main_0_bus_in_8_15,
    &icss_g_main_0_bus_pr1_iep0_cmp_intr_req_268_283_to_cmp_event_introuter_main_0_bus_in_32_47,
    &icss_g_main_0_bus_pr1_iep1_cmp_intr_req_256_261_to_cmp_event_introuter_main_0_bus_in_48_53,
    &icss_g_main_0_bus_pr1_iep1_cmp_intr_req_6_15_to_cmp_event_introuter_main_0_bus_in_54_63,
    &icss_g_main_0_bus_pr1_edc0_sync0_out_304_304_to_timesync_event_introuter_main_0_bus_in_16_16,
    &icss_g_main_0_bus_pr1_edc0_sync1_out_305_305_to_timesync_event_introuter_main_0_bus_in_17_17,
    &icss_g_main_0_bus_pr1_edc1_sync0_out_306_306_to_timesync_event_introuter_main_0_bus_in_18_18,
    &icss_g_main_0_bus_pr1_edc1_sync1_out_307_307_to_timesync_event_introuter_main_0_bus_in_19_19,
    &icss_g_main_0_bus_pr1_host_intr_pend_294_301_to_main2mcu_lvl_introuter_main_0_bus_in_32_39,
};
static const struct Sciclient_rmIrqNode tisci_irq_PRU_ICSSG0 = {
    .id = TISCI_DEV_PRU_ICSSG0,
    .n_if   = 11,
    .p_if   = &tisci_if_PRU_ICSSG0[0],
};

/* Start of PRU_ICSSG1 interface definition */
const struct Sciclient_rmIrqIf icss_g_main_1_bus_pr1_rx_sof_intr_req_284_285_to_main2mcu_pls_introuter_main_0_bus_in_24_25 = {
    .lbase  = 284,
    .len    = 2,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 24,
};
const struct Sciclient_rmIrqIf icss_g_main_1_bus_pr1_tx_sof_intr_req_302_303_to_main2mcu_pls_introuter_main_0_bus_in_26_27 = {
    .lbase  = 302,
    .len    = 2,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 26,
};
const struct Sciclient_rmIrqIf icss_g_main_1_bus_pr1_host_intr_req_286_293_to_cmp_event_introuter_main_0_bus_in_16_23 = {
    .lbase  = 286,
    .len    = 8,
    .rid    = TISCI_DEV_CMPEVENT_INTRTR0,
    .rbase  = 16,
};
const struct Sciclient_rmIrqIf icss_g_main_1_bus_pr1_iep0_cmp_intr_req_268_283_to_cmp_event_introuter_main_0_bus_in_64_79 = {
    .lbase  = 268,
    .len    = 16,
    .rid    = TISCI_DEV_CMPEVENT_INTRTR0,
    .rbase  = 64,
};
const struct Sciclient_rmIrqIf icss_g_main_1_bus_pr1_iep1_cmp_intr_req_256_261_to_cmp_event_introuter_main_0_bus_in_80_85 = {
    .lbase  = 256,
    .len    = 6,
    .rid    = TISCI_DEV_CMPEVENT_INTRTR0,
    .rbase  = 80,
};
const struct Sciclient_rmIrqIf icss_g_main_1_bus_pr1_iep1_cmp_intr_req_6_15_to_cmp_event_introuter_main_0_bus_in_86_95 = {
    .lbase  = 6,
    .len    = 10,
    .rid    = TISCI_DEV_CMPEVENT_INTRTR0,
    .rbase  = 86,
};
const struct Sciclient_rmIrqIf icss_g_main_1_bus_pr1_edc0_sync0_out_304_304_to_timesync_event_introuter_main_0_bus_in_20_20 = {
    .lbase  = 304,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 20,
};
const struct Sciclient_rmIrqIf icss_g_main_1_bus_pr1_edc0_sync1_out_305_305_to_timesync_event_introuter_main_0_bus_in_21_21 = {
    .lbase  = 305,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 21,
};
const struct Sciclient_rmIrqIf icss_g_main_1_bus_pr1_edc1_sync0_out_306_306_to_timesync_event_introuter_main_0_bus_in_22_22 = {
    .lbase  = 306,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 22,
};
const struct Sciclient_rmIrqIf icss_g_main_1_bus_pr1_edc1_sync1_out_307_307_to_timesync_event_introuter_main_0_bus_in_23_23 = {
    .lbase  = 307,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 23,
};
const struct Sciclient_rmIrqIf icss_g_main_1_bus_pr1_host_intr_pend_294_301_to_main2mcu_lvl_introuter_main_0_bus_in_40_47 = {
    .lbase  = 294,
    .len    = 8,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 40,
};
const struct Sciclient_rmIrqIf *const tisci_if_PRU_ICSSG1[] = {
    &icss_g_main_1_bus_pr1_rx_sof_intr_req_284_285_to_main2mcu_pls_introuter_main_0_bus_in_24_25,
    &icss_g_main_1_bus_pr1_tx_sof_intr_req_302_303_to_main2mcu_pls_introuter_main_0_bus_in_26_27,
    &icss_g_main_1_bus_pr1_host_intr_req_286_293_to_cmp_event_introuter_main_0_bus_in_16_23,
    &icss_g_main_1_bus_pr1_iep0_cmp_intr_req_268_283_to_cmp_event_introuter_main_0_bus_in_64_79,
    &icss_g_main_1_bus_pr1_iep1_cmp_intr_req_256_261_to_cmp_event_introuter_main_0_bus_in_80_85,
    &icss_g_main_1_bus_pr1_iep1_cmp_intr_req_6_15_to_cmp_event_introuter_main_0_bus_in_86_95,
    &icss_g_main_1_bus_pr1_edc0_sync0_out_304_304_to_timesync_event_introuter_main_0_bus_in_20_20,
    &icss_g_main_1_bus_pr1_edc0_sync1_out_305_305_to_timesync_event_introuter_main_0_bus_in_21_21,
    &icss_g_main_1_bus_pr1_edc1_sync0_out_306_306_to_timesync_event_introuter_main_0_bus_in_22_22,
    &icss_g_main_1_bus_pr1_edc1_sync1_out_307_307_to_timesync_event_introuter_main_0_bus_in_23_23,
    &icss_g_main_1_bus_pr1_host_intr_pend_294_301_to_main2mcu_lvl_introuter_main_0_bus_in_40_47,
};
static const struct Sciclient_rmIrqNode tisci_irq_PRU_ICSSG1 = {
    .id = TISCI_DEV_PRU_ICSSG1,
    .n_if   = 11,
    .p_if   = &tisci_if_PRU_ICSSG1[0],
};

/* Start of PRU_ICSSG2 interface definition */
const struct Sciclient_rmIrqIf icss_g_main_2_bus_pr1_rx_sof_intr_req_284_285_to_main2mcu_pls_introuter_main_0_bus_in_28_29 = {
    .lbase  = 284,
    .len    = 2,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 28,
};
const struct Sciclient_rmIrqIf icss_g_main_2_bus_pr1_tx_sof_intr_req_302_303_to_main2mcu_pls_introuter_main_0_bus_in_30_31 = {
    .lbase  = 302,
    .len    = 2,
    .rid    = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .rbase  = 30,
};
const struct Sciclient_rmIrqIf icss_g_main_2_bus_pr1_host_intr_req_286_293_to_cmp_event_introuter_main_0_bus_in_24_31 = {
    .lbase  = 286,
    .len    = 8,
    .rid    = TISCI_DEV_CMPEVENT_INTRTR0,
    .rbase  = 24,
};
const struct Sciclient_rmIrqIf icss_g_main_2_bus_pr1_iep0_cmp_intr_req_268_283_to_cmp_event_introuter_main_0_bus_in_96_111 = {
    .lbase  = 268,
    .len    = 16,
    .rid    = TISCI_DEV_CMPEVENT_INTRTR0,
    .rbase  = 96,
};
const struct Sciclient_rmIrqIf icss_g_main_2_bus_pr1_iep1_cmp_intr_req_256_261_to_cmp_event_introuter_main_0_bus_in_112_117 = {
    .lbase  = 256,
    .len    = 6,
    .rid    = TISCI_DEV_CMPEVENT_INTRTR0,
    .rbase  = 112,
};
const struct Sciclient_rmIrqIf icss_g_main_2_bus_pr1_iep1_cmp_intr_req_6_15_to_cmp_event_introuter_main_0_bus_in_118_127 = {
    .lbase  = 6,
    .len    = 10,
    .rid    = TISCI_DEV_CMPEVENT_INTRTR0,
    .rbase  = 118,
};
const struct Sciclient_rmIrqIf icss_g_main_2_bus_pr1_edc0_sync0_out_304_304_to_timesync_event_introuter_main_0_bus_in_24_24 = {
    .lbase  = 304,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 24,
};
const struct Sciclient_rmIrqIf icss_g_main_2_bus_pr1_edc0_sync1_out_305_305_to_timesync_event_introuter_main_0_bus_in_25_25 = {
    .lbase  = 305,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 25,
};
const struct Sciclient_rmIrqIf icss_g_main_2_bus_pr1_edc1_sync0_out_306_306_to_timesync_event_introuter_main_0_bus_in_26_26 = {
    .lbase  = 306,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 26,
};
const struct Sciclient_rmIrqIf icss_g_main_2_bus_pr1_edc1_sync1_out_307_307_to_timesync_event_introuter_main_0_bus_in_27_27 = {
    .lbase  = 307,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 27,
};
const struct Sciclient_rmIrqIf icss_g_main_2_bus_pr1_host_intr_pend_294_301_to_main2mcu_lvl_introuter_main_0_bus_in_48_55 = {
    .lbase  = 294,
    .len    = 8,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 48,
};
const struct Sciclient_rmIrqIf *const tisci_if_PRU_ICSSG2[] = {
    &icss_g_main_2_bus_pr1_rx_sof_intr_req_284_285_to_main2mcu_pls_introuter_main_0_bus_in_28_29,
    &icss_g_main_2_bus_pr1_tx_sof_intr_req_302_303_to_main2mcu_pls_introuter_main_0_bus_in_30_31,
    &icss_g_main_2_bus_pr1_host_intr_req_286_293_to_cmp_event_introuter_main_0_bus_in_24_31,
    &icss_g_main_2_bus_pr1_iep0_cmp_intr_req_268_283_to_cmp_event_introuter_main_0_bus_in_96_111,
    &icss_g_main_2_bus_pr1_iep1_cmp_intr_req_256_261_to_cmp_event_introuter_main_0_bus_in_112_117,
    &icss_g_main_2_bus_pr1_iep1_cmp_intr_req_6_15_to_cmp_event_introuter_main_0_bus_in_118_127,
    &icss_g_main_2_bus_pr1_edc0_sync0_out_304_304_to_timesync_event_introuter_main_0_bus_in_24_24,
    &icss_g_main_2_bus_pr1_edc0_sync1_out_305_305_to_timesync_event_introuter_main_0_bus_in_25_25,
    &icss_g_main_2_bus_pr1_edc1_sync0_out_306_306_to_timesync_event_introuter_main_0_bus_in_26_26,
    &icss_g_main_2_bus_pr1_edc1_sync1_out_307_307_to_timesync_event_introuter_main_0_bus_in_27_27,
    &icss_g_main_2_bus_pr1_host_intr_pend_294_301_to_main2mcu_lvl_introuter_main_0_bus_in_48_55,
};
static const struct Sciclient_rmIrqNode tisci_irq_PRU_ICSSG2 = {
    .id = TISCI_DEV_PRU_ICSSG2,
    .n_if   = 11,
    .p_if   = &tisci_if_PRU_ICSSG2[0],
};

/* Start of GPU0 interface definition */
const struct Sciclient_rmIrqIf k3_boltv2_main_0_bus_gpu_irq_3_3_to_main2mcu_lvl_introuter_main_0_bus_in_56_56 = {
    .lbase  = 3,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 56,
};
const struct Sciclient_rmIrqIf k3_boltv2_main_0_bus_exp_intr_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_57_57 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 57,
};
const struct Sciclient_rmIrqIf k3_boltv2_main_0_bus_init_err_4_4_to_main2mcu_lvl_introuter_main_0_bus_in_58_58 = {
    .lbase  = 4,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 58,
};
const struct Sciclient_rmIrqIf k3_boltv2_main_0_bus_target_err_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_59_59 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 59,
};
const struct Sciclient_rmIrqIf *const tisci_if_GPU0[] = {
    &k3_boltv2_main_0_bus_gpu_irq_3_3_to_main2mcu_lvl_introuter_main_0_bus_in_56_56,
    &k3_boltv2_main_0_bus_exp_intr_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_57_57,
    &k3_boltv2_main_0_bus_init_err_4_4_to_main2mcu_lvl_introuter_main_0_bus_in_58_58,
    &k3_boltv2_main_0_bus_target_err_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_59_59,
};
static const struct Sciclient_rmIrqNode tisci_irq_GPU0 = {
    .id = TISCI_DEV_GPU0,
    .n_if   = 4,
    .p_if   = &tisci_if_GPU0[0],
};

/* Start of CCDEBUGSS0 interface definition */
const struct Sciclient_rmIrqIf k3_cc_debug_cell_main_0_bus_aqcmpintr_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_13_13 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 13,
};
const struct Sciclient_rmIrqIf *const tisci_if_CCDEBUGSS0[] = {
    &k3_cc_debug_cell_main_0_bus_aqcmpintr_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_13_13,
};
static const struct Sciclient_rmIrqNode tisci_irq_CCDEBUGSS0 = {
    .id = TISCI_DEV_CCDEBUGSS0,
    .n_if   = 1,
    .p_if   = &tisci_if_CCDEBUGSS0[0],
};

/* Start of DSS0 interface definition */
const struct Sciclient_rmIrqIf k3_dss_ul_main_0_bus_dispc_intr_req_0_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_2_2 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 2,
};
const struct Sciclient_rmIrqIf k3_dss_ul_main_0_bus_dispc_intr_req_1_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_3_3 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 3,
};
const struct Sciclient_rmIrqIf *const tisci_if_DSS0[] = {
    &k3_dss_ul_main_0_bus_dispc_intr_req_0_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_2_2,
    &k3_dss_ul_main_0_bus_dispc_intr_req_1_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_3_3,
};
static const struct Sciclient_rmIrqNode tisci_irq_DSS0 = {
    .id = TISCI_DEV_DSS0,
    .n_if   = 2,
    .p_if   = &tisci_if_DSS0[0],
};

/* Start of DEBUGSS0 interface definition */
const struct Sciclient_rmIrqIf k3_main_debug_cell_main_0_bus_aqcmpintr_level_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_14_14 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 14,
};
const struct Sciclient_rmIrqIf k3_main_debug_cell_main_0_bus_ctm_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_15_15 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 15,
};
const struct Sciclient_rmIrqIf *const tisci_if_DEBUGSS0[] = {
    &k3_main_debug_cell_main_0_bus_aqcmpintr_level_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_14_14,
    &k3_main_debug_cell_main_0_bus_ctm_level_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_15_15,
};
static const struct Sciclient_rmIrqNode tisci_irq_DEBUGSS0 = {
    .id = TISCI_DEV_DEBUGSS0,
    .n_if   = 2,
    .p_if   = &tisci_if_DEBUGSS0[0],
};

/* Start of CBASS0 interface definition */
const struct Sciclient_rmIrqIf m4_main_cbass_main_0_bus_LPSC_per_common_err_intr_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_172_172 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 172,
};
const struct Sciclient_rmIrqIf *const tisci_if_CBASS0[] = {
    &m4_main_cbass_main_0_bus_LPSC_per_common_err_intr_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_172_172,
};
static const struct Sciclient_rmIrqNode tisci_irq_CBASS0 = {
    .id = TISCI_DEV_CBASS0,
    .n_if   = 1,
    .p_if   = &tisci_if_CBASS0[0],
};

/* Start of CBASS_DEBUG0 interface definition */
const struct Sciclient_rmIrqIf m4_main_dbg_cbass_main_0_bus_LPSC_main_debug_err_intr_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_173_173 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 173,
};
const struct Sciclient_rmIrqIf *const tisci_if_CBASS_DEBUG0[] = {
    &m4_main_dbg_cbass_main_0_bus_LPSC_main_debug_err_intr_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_173_173,
};
static const struct Sciclient_rmIrqNode tisci_irq_CBASS_DEBUG0 = {
    .id = TISCI_DEV_CBASS_DEBUG0,
    .n_if   = 1,
    .p_if   = &tisci_if_CBASS_DEBUG0[0],
};

/* Start of CBASS_FW0 interface definition */
const struct Sciclient_rmIrqIf m4_main_fw_cbass_main_0_bus_LPSC_main_infra_err_intr_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_174_174 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 174,
};
const struct Sciclient_rmIrqIf *const tisci_if_CBASS_FW0[] = {
    &m4_main_fw_cbass_main_0_bus_LPSC_main_infra_err_intr_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_174_174,
};
static const struct Sciclient_rmIrqNode tisci_irq_CBASS_FW0 = {
    .id = TISCI_DEV_CBASS_FW0,
    .n_if   = 1,
    .p_if   = &tisci_if_CBASS_FW0[0],
};

/* Start of CBASS_INFRA0 interface definition */
const struct Sciclient_rmIrqIf m4_main_infra_cbass_main_0_bus_LPSC_main_infra_err_intr_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_175_175 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 175,
};
const struct Sciclient_rmIrqIf *const tisci_if_CBASS_INFRA0[] = {
    &m4_main_infra_cbass_main_0_bus_LPSC_main_infra_err_intr_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_175_175,
};
static const struct Sciclient_rmIrqNode tisci_irq_CBASS_INFRA0 = {
    .id = TISCI_DEV_CBASS_INFRA0,
    .n_if   = 1,
    .p_if   = &tisci_if_CBASS_INFRA0[0],
};

/* Start of MAIN2MCU_LVL_INTRTR0 interface definition */
const struct Sciclient_rmIrqIf main2mcu_lvl_introuter_main_0_bus_outl_0_63_to_mcu_armss0_cpu0_bus_intr_160_223 = {
    .lbase  = 0,
    .len    = 64,
    .rid    = TISCI_DEV_MCU_ARMSS0_CPU0,
    .rbase  = 160,
};
const struct Sciclient_rmIrqIf main2mcu_lvl_introuter_main_0_bus_outl_0_63_to_mcu_armss0_cpu1_bus_intr_160_223 = {
    .lbase  = 0,
    .len    = 64,
    .rid    = TISCI_DEV_MCU_ARMSS0_CPU1,
    .rbase  = 160,
};
const struct Sciclient_rmIrqIf *const tisci_if_MAIN2MCU_LVL_INTRTR0[] = {
    &main2mcu_lvl_introuter_main_0_bus_outl_0_63_to_mcu_armss0_cpu0_bus_intr_160_223,
    &main2mcu_lvl_introuter_main_0_bus_outl_0_63_to_mcu_armss0_cpu1_bus_intr_160_223,
};
static const struct Sciclient_rmIrqNode tisci_irq_MAIN2MCU_LVL_INTRTR0 = {
    .id = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .n_if   = 2,
    .p_if   = &tisci_if_MAIN2MCU_LVL_INTRTR0[0],
};

/* Start of MAIN2MCU_PLS_INTRTR0 interface definition */
const struct Sciclient_rmIrqIf main2mcu_pls_introuter_main_0_bus_outp_0_47_to_mcu_armss0_cpu1_bus_intr_224_271 = {
    .lbase  = 0,
    .len    = 48,
    .rid    = TISCI_DEV_MCU_ARMSS0_CPU1,
    .rbase  = 224,
};
const struct Sciclient_rmIrqIf main2mcu_pls_introuter_main_0_bus_outp_0_47_to_mcu_armss0_cpu0_bus_intr_224_271 = {
    .lbase  = 0,
    .len    = 48,
    .rid    = TISCI_DEV_MCU_ARMSS0_CPU0,
    .rbase  = 224,
};
const struct Sciclient_rmIrqIf *const tisci_if_MAIN2MCU_PLS_INTRTR0[] = {
    &main2mcu_pls_introuter_main_0_bus_outp_0_47_to_mcu_armss0_cpu1_bus_intr_224_271,
    &main2mcu_pls_introuter_main_0_bus_outp_0_47_to_mcu_armss0_cpu0_bus_intr_224_271,
};
static const struct Sciclient_rmIrqNode tisci_irq_MAIN2MCU_PLS_INTRTR0 = {
    .id = TISCI_DEV_MAIN2MCU_PLS_INTRTR0,
    .n_if   = 2,
    .p_if   = &tisci_if_MAIN2MCU_PLS_INTRTR0[0],
};

/* Start of CTRL_MMR0 interface definition */
const struct Sciclient_rmIrqIf main_ctrl_mmr_main_0_bus_access_err_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_6_6 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 6,
};
const struct Sciclient_rmIrqIf *const tisci_if_CTRL_MMR0[] = {
    &main_ctrl_mmr_main_0_bus_access_err_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_6_6,
};
static const struct Sciclient_rmIrqNode tisci_irq_CTRL_MMR0 = {
    .id = TISCI_DEV_CTRL_MMR0,
    .n_if   = 1,
    .p_if   = &tisci_if_CTRL_MMR0[0],
};

/* Start of GPIOMUX_INTRTR0 interface definition */
const struct Sciclient_rmIrqIf main_gpiomux_introuter_main_0_bus_outp_20_25_to_icss_g_main_1_bus_pr1_iep0_cap_intr_req_262_267 = {
    .lbase  = 20,
    .len    = 6,
    .rid    = TISCI_DEV_PRU_ICSSG1,
    .rbase  = 262,
};
const struct Sciclient_rmIrqIf main_gpiomux_introuter_main_0_bus_outp_26_31_to_icss_g_main_1_bus_pr1_iep1_cap_intr_req_0_5 = {
    .lbase  = 26,
    .len    = 6,
    .rid    = TISCI_DEV_PRU_ICSSG1,
    .rbase  = 0,
};
const struct Sciclient_rmIrqIf main_gpiomux_introuter_main_0_bus_outp_24_31_to_icss_g_main_1_bus_pr1_slv_intr_88_95 = {
    .lbase  = 24,
    .len    = 8,
    .rid    = TISCI_DEV_PRU_ICSSG1,
    .rbase  = 88,
};
const struct Sciclient_rmIrqIf main_gpiomux_introuter_main_0_bus_outp_20_25_to_icss_g_main_0_bus_pr1_iep0_cap_intr_req_262_267 = {
    .lbase  = 20,
    .len    = 6,
    .rid    = TISCI_DEV_PRU_ICSSG0,
    .rbase  = 262,
};
const struct Sciclient_rmIrqIf main_gpiomux_introuter_main_0_bus_outp_26_31_to_icss_g_main_0_bus_pr1_iep1_cap_intr_req_0_5 = {
    .lbase  = 26,
    .len    = 6,
    .rid    = TISCI_DEV_PRU_ICSSG0,
    .rbase  = 0,
};
const struct Sciclient_rmIrqIf main_gpiomux_introuter_main_0_bus_outp_24_31_to_icss_g_main_0_bus_pr1_slv_intr_88_95 = {
    .lbase  = 24,
    .len    = 8,
    .rid    = TISCI_DEV_PRU_ICSSG0,
    .rbase  = 88,
};
const struct Sciclient_rmIrqIf main_gpiomux_introuter_main_0_bus_outp_0_31_to_gic500ss_main_0_bus_spi_392_423 = {
    .lbase  = 0,
    .len    = 32,
    .rid    = TISCI_DEV_GIC0,
    .rbase  = 392,
};
const struct Sciclient_rmIrqIf main_gpiomux_introuter_main_0_bus_outp_20_25_to_icss_g_main_2_bus_pr1_iep0_cap_intr_req_262_267 = {
    .lbase  = 20,
    .len    = 6,
    .rid    = TISCI_DEV_PRU_ICSSG2,
    .rbase  = 262,
};
const struct Sciclient_rmIrqIf main_gpiomux_introuter_main_0_bus_outp_26_31_to_icss_g_main_2_bus_pr1_iep1_cap_intr_req_0_5 = {
    .lbase  = 26,
    .len    = 6,
    .rid    = TISCI_DEV_PRU_ICSSG2,
    .rbase  = 0,
};
const struct Sciclient_rmIrqIf main_gpiomux_introuter_main_0_bus_outp_24_31_to_icss_g_main_2_bus_pr1_slv_intr_88_95 = {
    .lbase  = 24,
    .len    = 8,
    .rid    = TISCI_DEV_PRU_ICSSG2,
    .rbase  = 88,
};
const struct Sciclient_rmIrqIf main_gpiomux_introuter_main_0_bus_outp_0_7_to_esm_main_main_0_bus_esm_pls_event0_512_519 = {
    .lbase  = 0,
    .len    = 8,
    .rid    = TISCI_DEV_ESM0,
    .rbase  = 512,
};
const struct Sciclient_rmIrqIf main_gpiomux_introuter_main_0_bus_outp_0_7_to_esm_main_main_0_bus_esm_pls_event1_520_527 = {
    .lbase  = 0,
    .len    = 8,
    .rid    = TISCI_DEV_ESM0,
    .rbase  = 520,
};
const struct Sciclient_rmIrqIf main_gpiomux_introuter_main_0_bus_outp_0_7_to_esm_main_main_0_bus_esm_pls_event2_248_255 = {
    .lbase  = 0,
    .len    = 8,
    .rid    = TISCI_DEV_ESM0,
    .rbase  = 248,
};
const struct Sciclient_rmIrqIf *const tisci_if_GPIOMUX_INTRTR0[] = {
    &main_gpiomux_introuter_main_0_bus_outp_20_25_to_icss_g_main_1_bus_pr1_iep0_cap_intr_req_262_267,
    &main_gpiomux_introuter_main_0_bus_outp_26_31_to_icss_g_main_1_bus_pr1_iep1_cap_intr_req_0_5,
    &main_gpiomux_introuter_main_0_bus_outp_24_31_to_icss_g_main_1_bus_pr1_slv_intr_88_95,
    &main_gpiomux_introuter_main_0_bus_outp_20_25_to_icss_g_main_0_bus_pr1_iep0_cap_intr_req_262_267,
    &main_gpiomux_introuter_main_0_bus_outp_26_31_to_icss_g_main_0_bus_pr1_iep1_cap_intr_req_0_5,
    &main_gpiomux_introuter_main_0_bus_outp_24_31_to_icss_g_main_0_bus_pr1_slv_intr_88_95,
    &main_gpiomux_introuter_main_0_bus_outp_0_31_to_gic500ss_main_0_bus_spi_392_423,
    &main_gpiomux_introuter_main_0_bus_outp_20_25_to_icss_g_main_2_bus_pr1_iep0_cap_intr_req_262_267,
    &main_gpiomux_introuter_main_0_bus_outp_26_31_to_icss_g_main_2_bus_pr1_iep1_cap_intr_req_0_5,
    &main_gpiomux_introuter_main_0_bus_outp_24_31_to_icss_g_main_2_bus_pr1_slv_intr_88_95,
    &main_gpiomux_introuter_main_0_bus_outp_0_7_to_esm_main_main_0_bus_esm_pls_event0_512_519,
    &main_gpiomux_introuter_main_0_bus_outp_0_7_to_esm_main_main_0_bus_esm_pls_event1_520_527,
    &main_gpiomux_introuter_main_0_bus_outp_0_7_to_esm_main_main_0_bus_esm_pls_event2_248_255,
};
static const struct Sciclient_rmIrqNode tisci_irq_GPIOMUX_INTRTR0 = {
    .id = TISCI_DEV_GPIOMUX_INTRTR0,
    .n_if   = 13,
    .p_if   = &tisci_if_GPIOMUX_INTRTR0[0],
};

/* Start of MCASP0 interface definition */
const struct Sciclient_rmIrqIf mcasp_main_0_bus_xmit_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_16_16 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 16,
};
const struct Sciclient_rmIrqIf mcasp_main_0_bus_rec_intr_pend_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_17_17 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 17,
};
const struct Sciclient_rmIrqIf *const tisci_if_MCASP0[] = {
    &mcasp_main_0_bus_xmit_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_16_16,
    &mcasp_main_0_bus_rec_intr_pend_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_17_17,
};
static const struct Sciclient_rmIrqNode tisci_irq_MCASP0 = {
    .id = TISCI_DEV_MCASP0,
    .n_if   = 2,
    .p_if   = &tisci_if_MCASP0[0],
};

/* Start of MCASP1 interface definition */
const struct Sciclient_rmIrqIf mcasp_main_1_bus_xmit_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_18_18 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 18,
};
const struct Sciclient_rmIrqIf mcasp_main_1_bus_rec_intr_pend_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_19_19 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 19,
};
const struct Sciclient_rmIrqIf *const tisci_if_MCASP1[] = {
    &mcasp_main_1_bus_xmit_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_18_18,
    &mcasp_main_1_bus_rec_intr_pend_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_19_19,
};
static const struct Sciclient_rmIrqNode tisci_irq_MCASP1 = {
    .id = TISCI_DEV_MCASP1,
    .n_if   = 2,
    .p_if   = &tisci_if_MCASP1[0],
};

/* Start of MCASP2 interface definition */
const struct Sciclient_rmIrqIf mcasp_main_2_bus_xmit_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_20_20 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 20,
};
const struct Sciclient_rmIrqIf mcasp_main_2_bus_rec_intr_pend_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_21_21 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 21,
};
const struct Sciclient_rmIrqIf *const tisci_if_MCASP2[] = {
    &mcasp_main_2_bus_xmit_intr_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_20_20,
    &mcasp_main_2_bus_rec_intr_pend_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_21_21,
};
static const struct Sciclient_rmIrqNode tisci_irq_MCASP2 = {
    .id = TISCI_DEV_MCASP2,
    .n_if   = 2,
    .p_if   = &tisci_if_MCASP2[0],
};

/* Start of I2C0 interface definition */
const struct Sciclient_rmIrqIf mshsi2c_main_0_bus_pointrpend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_100_100 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 100,
};
const struct Sciclient_rmIrqIf *const tisci_if_I2C0[] = {
    &mshsi2c_main_0_bus_pointrpend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_100_100,
};
static const struct Sciclient_rmIrqNode tisci_irq_I2C0 = {
    .id = TISCI_DEV_I2C0,
    .n_if   = 1,
    .p_if   = &tisci_if_I2C0[0],
};

/* Start of I2C1 interface definition */
const struct Sciclient_rmIrqIf mshsi2c_main_1_bus_pointrpend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_101_101 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 101,
};
const struct Sciclient_rmIrqIf *const tisci_if_I2C1[] = {
    &mshsi2c_main_1_bus_pointrpend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_101_101,
};
static const struct Sciclient_rmIrqNode tisci_irq_I2C1 = {
    .id = TISCI_DEV_I2C1,
    .n_if   = 1,
    .p_if   = &tisci_if_I2C1[0],
};

/* Start of I2C2 interface definition */
const struct Sciclient_rmIrqIf mshsi2c_main_2_bus_pointrpend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_102_102 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 102,
};
const struct Sciclient_rmIrqIf *const tisci_if_I2C2[] = {
    &mshsi2c_main_2_bus_pointrpend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_102_102,
};
static const struct Sciclient_rmIrqNode tisci_irq_I2C2 = {
    .id = TISCI_DEV_I2C2,
    .n_if   = 1,
    .p_if   = &tisci_if_I2C2[0],
};

/* Start of I2C3 interface definition */
const struct Sciclient_rmIrqIf mshsi2c_main_3_bus_pointrpend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_103_103 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 103,
};
const struct Sciclient_rmIrqIf *const tisci_if_I2C3[] = {
    &mshsi2c_main_3_bus_pointrpend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_103_103,
};
static const struct Sciclient_rmIrqNode tisci_irq_I2C3 = {
    .id = TISCI_DEV_I2C3,
    .n_if   = 1,
    .p_if   = &tisci_if_I2C3[0],
};

/* Start of NAVSS0 interface definition */
const struct Sciclient_rmIrqIf navss256l_main_0_bus_cpts0_comp_9_9_to_cmp_event_introuter_main_0_bus_in_4_4 = {
    .lbase  = 9,
    .len    = 1,
    .rid    = TISCI_DEV_CMPEVENT_INTRTR0,
    .rbase  = 4,
};
const struct Sciclient_rmIrqIf navss256l_main_0_bus_cpts0_genf0_10_10_to_timesync_event_introuter_main_0_bus_in_4_4 = {
    .lbase  = 10,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 4,
};
const struct Sciclient_rmIrqIf navss256l_main_0_bus_cpts0_genf1_11_11_to_timesync_event_introuter_main_0_bus_in_5_5 = {
    .lbase  = 11,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 5,
};
const struct Sciclient_rmIrqIf navss256l_main_0_bus_cpts0_genf2_12_12_to_timesync_event_introuter_main_0_bus_in_6_6 = {
    .lbase  = 12,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 6,
};
const struct Sciclient_rmIrqIf navss256l_main_0_bus_cpts0_genf3_13_13_to_timesync_event_introuter_main_0_bus_in_7_7 = {
    .lbase  = 13,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 7,
};
const struct Sciclient_rmIrqIf navss256l_main_0_bus_cpts0_genf4_14_14_to_timesync_event_introuter_main_0_bus_in_8_8 = {
    .lbase  = 14,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 8,
};
const struct Sciclient_rmIrqIf navss256l_main_0_bus_cpts0_genf5_15_15_to_timesync_event_introuter_main_0_bus_in_9_9 = {
    .lbase  = 15,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 9,
};
const struct Sciclient_rmIrqIf navss256l_main_0_bus_cpts0_sync_16_16_to_timesync_event_introuter_main_0_bus_in_30_30 = {
    .lbase  = 16,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 30,
};
const struct Sciclient_rmIrqIf *const tisci_if_NAVSS0[] = {
    &navss256l_main_0_bus_cpts0_comp_9_9_to_cmp_event_introuter_main_0_bus_in_4_4,
    &navss256l_main_0_bus_cpts0_genf0_10_10_to_timesync_event_introuter_main_0_bus_in_4_4,
    &navss256l_main_0_bus_cpts0_genf1_11_11_to_timesync_event_introuter_main_0_bus_in_5_5,
    &navss256l_main_0_bus_cpts0_genf2_12_12_to_timesync_event_introuter_main_0_bus_in_6_6,
    &navss256l_main_0_bus_cpts0_genf3_13_13_to_timesync_event_introuter_main_0_bus_in_7_7,
    &navss256l_main_0_bus_cpts0_genf4_14_14_to_timesync_event_introuter_main_0_bus_in_8_8,
    &navss256l_main_0_bus_cpts0_genf5_15_15_to_timesync_event_introuter_main_0_bus_in_9_9,
    &navss256l_main_0_bus_cpts0_sync_16_16_to_timesync_event_introuter_main_0_bus_in_30_30,
};
static const struct Sciclient_rmIrqNode tisci_irq_NAVSS0 = {
    .id = TISCI_DEV_NAVSS0,
    .n_if   = 8,
    .p_if   = &tisci_if_NAVSS0[0],
};

/* Start of PCIE0 interface definition */
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie_cpts_comp_19_19_to_cmp_event_introuter_main_0_bus_in_5_5 = {
    .lbase  = 19,
    .len    = 1,
    .rid    = TISCI_DEV_CMPEVENT_INTRTR0,
    .rbase  = 5,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie_cpts_genf0_20_20_to_timesync_event_introuter_main_0_bus_in_10_10 = {
    .lbase  = 20,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 10,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie_cpts_hw1_push_17_17_to_timesync_event_introuter_main_0_bus_in_14_14 = {
    .lbase  = 17,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 14,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie_cpts_sync_21_21_to_timesync_event_introuter_main_0_bus_in_28_28 = {
    .lbase  = 21,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 28,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie0_pend_13_13_to_main2mcu_lvl_introuter_main_0_bus_in_64_64 = {
    .lbase  = 13,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 64,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie1_pend_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_65_65 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 65,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie2_pend_7_7_to_main2mcu_lvl_introuter_main_0_bus_in_66_66 = {
    .lbase  = 7,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 66,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie3_pend_4_4_to_main2mcu_lvl_introuter_main_0_bus_in_67_67 = {
    .lbase  = 4,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 67,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie4_pend_5_5_to_main2mcu_lvl_introuter_main_0_bus_in_68_68 = {
    .lbase  = 5,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 68,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie5_pend_3_3_to_main2mcu_lvl_introuter_main_0_bus_in_69_69 = {
    .lbase  = 3,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 69,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie6_pend_11_11_to_main2mcu_lvl_introuter_main_0_bus_in_70_70 = {
    .lbase  = 11,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 70,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie7_pend_8_8_to_main2mcu_lvl_introuter_main_0_bus_in_71_71 = {
    .lbase  = 8,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 71,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie8_pend_9_9_to_main2mcu_lvl_introuter_main_0_bus_in_72_72 = {
    .lbase  = 9,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 72,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie9_pend_16_16_to_main2mcu_lvl_introuter_main_0_bus_in_73_73 = {
    .lbase  = 16,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 73,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie10_pend_15_15_to_main2mcu_lvl_introuter_main_0_bus_in_74_74 = {
    .lbase  = 15,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 74,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie11_pend_14_14_to_main2mcu_lvl_introuter_main_0_bus_in_75_75 = {
    .lbase  = 14,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 75,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie12_pend_6_6_to_main2mcu_lvl_introuter_main_0_bus_in_76_76 = {
    .lbase  = 6,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 76,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie13_pend_10_10_to_main2mcu_lvl_introuter_main_0_bus_in_77_77 = {
    .lbase  = 10,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 77,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie14_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_78_78 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 78,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_0_bus_pcie_cpts_pend_12_12_to_main2mcu_lvl_introuter_main_0_bus_in_79_79 = {
    .lbase  = 12,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 79,
};
const struct Sciclient_rmIrqIf *const tisci_if_PCIE0[] = {
    &pcie_g3x2_main_0_bus_pcie_cpts_comp_19_19_to_cmp_event_introuter_main_0_bus_in_5_5,
    &pcie_g3x2_main_0_bus_pcie_cpts_genf0_20_20_to_timesync_event_introuter_main_0_bus_in_10_10,
    &pcie_g3x2_main_0_bus_pcie_cpts_hw1_push_17_17_to_timesync_event_introuter_main_0_bus_in_14_14,
    &pcie_g3x2_main_0_bus_pcie_cpts_sync_21_21_to_timesync_event_introuter_main_0_bus_in_28_28,
    &pcie_g3x2_main_0_bus_pcie0_pend_13_13_to_main2mcu_lvl_introuter_main_0_bus_in_64_64,
    &pcie_g3x2_main_0_bus_pcie1_pend_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_65_65,
    &pcie_g3x2_main_0_bus_pcie2_pend_7_7_to_main2mcu_lvl_introuter_main_0_bus_in_66_66,
    &pcie_g3x2_main_0_bus_pcie3_pend_4_4_to_main2mcu_lvl_introuter_main_0_bus_in_67_67,
    &pcie_g3x2_main_0_bus_pcie4_pend_5_5_to_main2mcu_lvl_introuter_main_0_bus_in_68_68,
    &pcie_g3x2_main_0_bus_pcie5_pend_3_3_to_main2mcu_lvl_introuter_main_0_bus_in_69_69,
    &pcie_g3x2_main_0_bus_pcie6_pend_11_11_to_main2mcu_lvl_introuter_main_0_bus_in_70_70,
    &pcie_g3x2_main_0_bus_pcie7_pend_8_8_to_main2mcu_lvl_introuter_main_0_bus_in_71_71,
    &pcie_g3x2_main_0_bus_pcie8_pend_9_9_to_main2mcu_lvl_introuter_main_0_bus_in_72_72,
    &pcie_g3x2_main_0_bus_pcie9_pend_16_16_to_main2mcu_lvl_introuter_main_0_bus_in_73_73,
    &pcie_g3x2_main_0_bus_pcie10_pend_15_15_to_main2mcu_lvl_introuter_main_0_bus_in_74_74,
    &pcie_g3x2_main_0_bus_pcie11_pend_14_14_to_main2mcu_lvl_introuter_main_0_bus_in_75_75,
    &pcie_g3x2_main_0_bus_pcie12_pend_6_6_to_main2mcu_lvl_introuter_main_0_bus_in_76_76,
    &pcie_g3x2_main_0_bus_pcie13_pend_10_10_to_main2mcu_lvl_introuter_main_0_bus_in_77_77,
    &pcie_g3x2_main_0_bus_pcie14_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_78_78,
    &pcie_g3x2_main_0_bus_pcie_cpts_pend_12_12_to_main2mcu_lvl_introuter_main_0_bus_in_79_79,
};
static const struct Sciclient_rmIrqNode tisci_irq_PCIE0 = {
    .id = TISCI_DEV_PCIE0,
    .n_if   = 20,
    .p_if   = &tisci_if_PCIE0[0],
};

/* Start of PCIE1 interface definition */
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie_cpts_comp_19_19_to_cmp_event_introuter_main_0_bus_in_6_6 = {
    .lbase  = 19,
    .len    = 1,
    .rid    = TISCI_DEV_CMPEVENT_INTRTR0,
    .rbase  = 6,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie_cpts_genf0_20_20_to_timesync_event_introuter_main_0_bus_in_11_11 = {
    .lbase  = 20,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 11,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie_cpts_hw1_push_17_17_to_timesync_event_introuter_main_0_bus_in_15_15 = {
    .lbase  = 17,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 15,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie_cpts_sync_21_21_to_timesync_event_introuter_main_0_bus_in_29_29 = {
    .lbase  = 21,
    .len    = 1,
    .rid    = TISCI_DEV_TIMESYNC_INTRTR0,
    .rbase  = 29,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie0_pend_13_13_to_main2mcu_lvl_introuter_main_0_bus_in_80_80 = {
    .lbase  = 13,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 80,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie1_pend_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_81_81 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 81,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie2_pend_7_7_to_main2mcu_lvl_introuter_main_0_bus_in_82_82 = {
    .lbase  = 7,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 82,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie3_pend_4_4_to_main2mcu_lvl_introuter_main_0_bus_in_83_83 = {
    .lbase  = 4,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 83,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie4_pend_5_5_to_main2mcu_lvl_introuter_main_0_bus_in_84_84 = {
    .lbase  = 5,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 84,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie5_pend_3_3_to_main2mcu_lvl_introuter_main_0_bus_in_85_85 = {
    .lbase  = 3,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 85,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie6_pend_11_11_to_main2mcu_lvl_introuter_main_0_bus_in_86_86 = {
    .lbase  = 11,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 86,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie7_pend_8_8_to_main2mcu_lvl_introuter_main_0_bus_in_87_87 = {
    .lbase  = 8,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 87,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie8_pend_9_9_to_main2mcu_lvl_introuter_main_0_bus_in_88_88 = {
    .lbase  = 9,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 88,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie9_pend_16_16_to_main2mcu_lvl_introuter_main_0_bus_in_89_89 = {
    .lbase  = 16,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 89,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie10_pend_15_15_to_main2mcu_lvl_introuter_main_0_bus_in_90_90 = {
    .lbase  = 15,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 90,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie11_pend_14_14_to_main2mcu_lvl_introuter_main_0_bus_in_91_91 = {
    .lbase  = 14,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 91,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie12_pend_6_6_to_main2mcu_lvl_introuter_main_0_bus_in_92_92 = {
    .lbase  = 6,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 92,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie13_pend_10_10_to_main2mcu_lvl_introuter_main_0_bus_in_93_93 = {
    .lbase  = 10,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 93,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie14_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_94_94 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 94,
};
const struct Sciclient_rmIrqIf pcie_g3x2_main_1_bus_pcie_cpts_pend_12_12_to_main2mcu_lvl_introuter_main_0_bus_in_95_95 = {
    .lbase  = 12,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 95,
};
const struct Sciclient_rmIrqIf *const tisci_if_PCIE1[] = {
    &pcie_g3x2_main_1_bus_pcie_cpts_comp_19_19_to_cmp_event_introuter_main_0_bus_in_6_6,
    &pcie_g3x2_main_1_bus_pcie_cpts_genf0_20_20_to_timesync_event_introuter_main_0_bus_in_11_11,
    &pcie_g3x2_main_1_bus_pcie_cpts_hw1_push_17_17_to_timesync_event_introuter_main_0_bus_in_15_15,
    &pcie_g3x2_main_1_bus_pcie_cpts_sync_21_21_to_timesync_event_introuter_main_0_bus_in_29_29,
    &pcie_g3x2_main_1_bus_pcie0_pend_13_13_to_main2mcu_lvl_introuter_main_0_bus_in_80_80,
    &pcie_g3x2_main_1_bus_pcie1_pend_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_81_81,
    &pcie_g3x2_main_1_bus_pcie2_pend_7_7_to_main2mcu_lvl_introuter_main_0_bus_in_82_82,
    &pcie_g3x2_main_1_bus_pcie3_pend_4_4_to_main2mcu_lvl_introuter_main_0_bus_in_83_83,
    &pcie_g3x2_main_1_bus_pcie4_pend_5_5_to_main2mcu_lvl_introuter_main_0_bus_in_84_84,
    &pcie_g3x2_main_1_bus_pcie5_pend_3_3_to_main2mcu_lvl_introuter_main_0_bus_in_85_85,
    &pcie_g3x2_main_1_bus_pcie6_pend_11_11_to_main2mcu_lvl_introuter_main_0_bus_in_86_86,
    &pcie_g3x2_main_1_bus_pcie7_pend_8_8_to_main2mcu_lvl_introuter_main_0_bus_in_87_87,
    &pcie_g3x2_main_1_bus_pcie8_pend_9_9_to_main2mcu_lvl_introuter_main_0_bus_in_88_88,
    &pcie_g3x2_main_1_bus_pcie9_pend_16_16_to_main2mcu_lvl_introuter_main_0_bus_in_89_89,
    &pcie_g3x2_main_1_bus_pcie10_pend_15_15_to_main2mcu_lvl_introuter_main_0_bus_in_90_90,
    &pcie_g3x2_main_1_bus_pcie11_pend_14_14_to_main2mcu_lvl_introuter_main_0_bus_in_91_91,
    &pcie_g3x2_main_1_bus_pcie12_pend_6_6_to_main2mcu_lvl_introuter_main_0_bus_in_92_92,
    &pcie_g3x2_main_1_bus_pcie13_pend_10_10_to_main2mcu_lvl_introuter_main_0_bus_in_93_93,
    &pcie_g3x2_main_1_bus_pcie14_pend_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_94_94,
    &pcie_g3x2_main_1_bus_pcie_cpts_pend_12_12_to_main2mcu_lvl_introuter_main_0_bus_in_95_95,
};
static const struct Sciclient_rmIrqNode tisci_irq_PCIE1 = {
    .id = TISCI_DEV_PCIE1,
    .n_if   = 20,
    .p_if   = &tisci_if_PCIE1[0],
};

/* Start of SA2_UL0 interface definition */
const struct Sciclient_rmIrqIf sa2_ul_main_0_bus_sa_ul_trng_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_4_4 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 4,
};
const struct Sciclient_rmIrqIf sa2_ul_main_0_bus_sa_ul_pka_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_5_5 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 5,
};
const struct Sciclient_rmIrqIf *const tisci_if_SA2_UL0[] = {
    &sa2_ul_main_0_bus_sa_ul_trng_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_4_4,
    &sa2_ul_main_0_bus_sa_ul_pka_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_5_5,
};
static const struct Sciclient_rmIrqNode tisci_irq_SA2_UL0 = {
    .id = TISCI_DEV_SA2_UL0,
    .n_if   = 2,
    .p_if   = &tisci_if_SA2_UL0[0],
};

/* Start of MCSPI0 interface definition */
const struct Sciclient_rmIrqIf spi_main_0_bus_intr_spi_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_96_96 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 96,
};
const struct Sciclient_rmIrqIf *const tisci_if_MCSPI0[] = {
    &spi_main_0_bus_intr_spi_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_96_96,
};
static const struct Sciclient_rmIrqNode tisci_irq_MCSPI0 = {
    .id = TISCI_DEV_MCSPI0,
    .n_if   = 1,
    .p_if   = &tisci_if_MCSPI0[0],
};

/* Start of MCSPI1 interface definition */
const struct Sciclient_rmIrqIf spi_main_1_bus_intr_spi_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_97_97 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 97,
};
const struct Sciclient_rmIrqIf *const tisci_if_MCSPI1[] = {
    &spi_main_1_bus_intr_spi_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_97_97,
};
static const struct Sciclient_rmIrqNode tisci_irq_MCSPI1 = {
    .id = TISCI_DEV_MCSPI1,
    .n_if   = 1,
    .p_if   = &tisci_if_MCSPI1[0],
};

/* Start of MCSPI2 interface definition */
const struct Sciclient_rmIrqIf spi_main_2_bus_intr_spi_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_98_98 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 98,
};
const struct Sciclient_rmIrqIf *const tisci_if_MCSPI2[] = {
    &spi_main_2_bus_intr_spi_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_98_98,
};
static const struct Sciclient_rmIrqNode tisci_irq_MCSPI2 = {
    .id = TISCI_DEV_MCSPI2,
    .n_if   = 1,
    .p_if   = &tisci_if_MCSPI2[0],
};

/* Start of MCSPI3 interface definition */
const struct Sciclient_rmIrqIf spi_main_3_bus_intr_spi_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_99_99 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 99,
};
const struct Sciclient_rmIrqIf *const tisci_if_MCSPI3[] = {
    &spi_main_3_bus_intr_spi_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_99_99,
};
static const struct Sciclient_rmIrqNode tisci_irq_MCSPI3 = {
    .id = TISCI_DEV_MCSPI3,
    .n_if   = 1,
    .p_if   = &tisci_if_MCSPI3[0],
};

/* Start of TIMESYNC_INTRTR0 interface definition */
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_32_39_to_pdma_main1_main_0_bus_levent_in_0_7 = {
    .lbase  = 32,
    .len    = 8,
    .rid    = TISCI_DEV_PDMA1,
    .rbase  = 0,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_20_20_to_pcie_g3x2_main_0_bus_pcie_cpts_hw2_push_18_18 = {
    .lbase  = 20,
    .len    = 1,
    .rid    = TISCI_DEV_PCIE0,
    .rbase  = 18,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_21_21_to_pcie_g3x2_main_1_bus_pcie_cpts_hw2_push_18_18 = {
    .lbase  = 21,
    .len    = 1,
    .rid    = TISCI_DEV_PCIE1,
    .rbase  = 18,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_8_8_to_icss_g_main_0_bus_pr1_edc0_latch0_in_308_308 = {
    .lbase  = 8,
    .len    = 1,
    .rid    = TISCI_DEV_PRU_ICSSG0,
    .rbase  = 308,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_9_9_to_icss_g_main_0_bus_pr1_edc0_latch1_in_309_309 = {
    .lbase  = 9,
    .len    = 1,
    .rid    = TISCI_DEV_PRU_ICSSG0,
    .rbase  = 309,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_10_10_to_icss_g_main_0_bus_pr1_edc1_latch0_in_310_310 = {
    .lbase  = 10,
    .len    = 1,
    .rid    = TISCI_DEV_PRU_ICSSG0,
    .rbase  = 310,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_11_11_to_icss_g_main_0_bus_pr1_edc1_latch1_in_311_311 = {
    .lbase  = 11,
    .len    = 1,
    .rid    = TISCI_DEV_PRU_ICSSG0,
    .rbase  = 311,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_12_12_to_icss_g_main_1_bus_pr1_edc0_latch0_in_308_308 = {
    .lbase  = 12,
    .len    = 1,
    .rid    = TISCI_DEV_PRU_ICSSG1,
    .rbase  = 308,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_13_13_to_icss_g_main_1_bus_pr1_edc0_latch1_in_309_309 = {
    .lbase  = 13,
    .len    = 1,
    .rid    = TISCI_DEV_PRU_ICSSG1,
    .rbase  = 309,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_14_14_to_icss_g_main_1_bus_pr1_edc1_latch0_in_310_310 = {
    .lbase  = 14,
    .len    = 1,
    .rid    = TISCI_DEV_PRU_ICSSG1,
    .rbase  = 310,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_15_15_to_icss_g_main_1_bus_pr1_edc1_latch1_in_311_311 = {
    .lbase  = 15,
    .len    = 1,
    .rid    = TISCI_DEV_PRU_ICSSG1,
    .rbase  = 311,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_16_16_to_icss_g_main_2_bus_pr1_edc0_latch0_in_308_308 = {
    .lbase  = 16,
    .len    = 1,
    .rid    = TISCI_DEV_PRU_ICSSG2,
    .rbase  = 308,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_17_17_to_icss_g_main_2_bus_pr1_edc0_latch1_in_309_309 = {
    .lbase  = 17,
    .len    = 1,
    .rid    = TISCI_DEV_PRU_ICSSG2,
    .rbase  = 309,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_18_18_to_icss_g_main_2_bus_pr1_edc1_latch0_in_310_310 = {
    .lbase  = 18,
    .len    = 1,
    .rid    = TISCI_DEV_PRU_ICSSG2,
    .rbase  = 310,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_19_19_to_icss_g_main_2_bus_pr1_edc1_latch1_in_311_311 = {
    .lbase  = 19,
    .len    = 1,
    .rid    = TISCI_DEV_PRU_ICSSG2,
    .rbase  = 311,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_24_24_to_cpsw_2guss_mcu_0_bus_cpts_hw3_push_0_0 = {
    .lbase  = 24,
    .len    = 1,
    .rid    = TISCI_DEV_MCU_CPSW0,
    .rbase  = 0,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_25_25_to_cpsw_2guss_mcu_0_bus_cpts_hw4_push_2_2 = {
    .lbase  = 25,
    .len    = 1,
    .rid    = TISCI_DEV_MCU_CPSW0,
    .rbase  = 2,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_0_0_to_navss256l_main_0_bus_cpts0_hw1_push_0_0 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_NAVSS0,
    .rbase  = 0,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_1_1_to_navss256l_main_0_bus_cpts0_hw2_push_2_2 = {
    .lbase  = 1,
    .len    = 1,
    .rid    = TISCI_DEV_NAVSS0,
    .rbase  = 2,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_2_2_to_navss256l_main_0_bus_cpts0_hw3_push_3_3 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_NAVSS0,
    .rbase  = 3,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_3_3_to_navss256l_main_0_bus_cpts0_hw4_push_4_4 = {
    .lbase  = 3,
    .len    = 1,
    .rid    = TISCI_DEV_NAVSS0,
    .rbase  = 4,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_4_4_to_navss256l_main_0_bus_cpts0_hw5_push_5_5 = {
    .lbase  = 4,
    .len    = 1,
    .rid    = TISCI_DEV_NAVSS0,
    .rbase  = 5,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_5_5_to_navss256l_main_0_bus_cpts0_hw6_push_6_6 = {
    .lbase  = 5,
    .len    = 1,
    .rid    = TISCI_DEV_NAVSS0,
    .rbase  = 6,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_6_6_to_navss256l_main_0_bus_cpts0_hw7_push_7_7 = {
    .lbase  = 6,
    .len    = 1,
    .rid    = TISCI_DEV_NAVSS0,
    .rbase  = 7,
};
const struct Sciclient_rmIrqIf timesync_event_introuter_main_0_bus_outl_7_7_to_navss256l_main_0_bus_cpts0_hw8_push_8_8 = {
    .lbase  = 7,
    .len    = 1,
    .rid    = TISCI_DEV_NAVSS0,
    .rbase  = 8,
};
const struct Sciclient_rmIrqIf *const tisci_if_TIMESYNC_INTRTR0[] = {
    &timesync_event_introuter_main_0_bus_outl_32_39_to_pdma_main1_main_0_bus_levent_in_0_7,
    &timesync_event_introuter_main_0_bus_outl_20_20_to_pcie_g3x2_main_0_bus_pcie_cpts_hw2_push_18_18,
    &timesync_event_introuter_main_0_bus_outl_21_21_to_pcie_g3x2_main_1_bus_pcie_cpts_hw2_push_18_18,
    &timesync_event_introuter_main_0_bus_outl_8_8_to_icss_g_main_0_bus_pr1_edc0_latch0_in_308_308,
    &timesync_event_introuter_main_0_bus_outl_9_9_to_icss_g_main_0_bus_pr1_edc0_latch1_in_309_309,
    &timesync_event_introuter_main_0_bus_outl_10_10_to_icss_g_main_0_bus_pr1_edc1_latch0_in_310_310,
    &timesync_event_introuter_main_0_bus_outl_11_11_to_icss_g_main_0_bus_pr1_edc1_latch1_in_311_311,
    &timesync_event_introuter_main_0_bus_outl_12_12_to_icss_g_main_1_bus_pr1_edc0_latch0_in_308_308,
    &timesync_event_introuter_main_0_bus_outl_13_13_to_icss_g_main_1_bus_pr1_edc0_latch1_in_309_309,
    &timesync_event_introuter_main_0_bus_outl_14_14_to_icss_g_main_1_bus_pr1_edc1_latch0_in_310_310,
    &timesync_event_introuter_main_0_bus_outl_15_15_to_icss_g_main_1_bus_pr1_edc1_latch1_in_311_311,
    &timesync_event_introuter_main_0_bus_outl_16_16_to_icss_g_main_2_bus_pr1_edc0_latch0_in_308_308,
    &timesync_event_introuter_main_0_bus_outl_17_17_to_icss_g_main_2_bus_pr1_edc0_latch1_in_309_309,
    &timesync_event_introuter_main_0_bus_outl_18_18_to_icss_g_main_2_bus_pr1_edc1_latch0_in_310_310,
    &timesync_event_introuter_main_0_bus_outl_19_19_to_icss_g_main_2_bus_pr1_edc1_latch1_in_311_311,
    &timesync_event_introuter_main_0_bus_outl_24_24_to_cpsw_2guss_mcu_0_bus_cpts_hw3_push_0_0,
    &timesync_event_introuter_main_0_bus_outl_25_25_to_cpsw_2guss_mcu_0_bus_cpts_hw4_push_2_2,
    &timesync_event_introuter_main_0_bus_outl_0_0_to_navss256l_main_0_bus_cpts0_hw1_push_0_0,
    &timesync_event_introuter_main_0_bus_outl_1_1_to_navss256l_main_0_bus_cpts0_hw2_push_2_2,
    &timesync_event_introuter_main_0_bus_outl_2_2_to_navss256l_main_0_bus_cpts0_hw3_push_3_3,
    &timesync_event_introuter_main_0_bus_outl_3_3_to_navss256l_main_0_bus_cpts0_hw4_push_4_4,
    &timesync_event_introuter_main_0_bus_outl_4_4_to_navss256l_main_0_bus_cpts0_hw5_push_5_5,
    &timesync_event_introuter_main_0_bus_outl_5_5_to_navss256l_main_0_bus_cpts0_hw6_push_6_6,
    &timesync_event_introuter_main_0_bus_outl_6_6_to_navss256l_main_0_bus_cpts0_hw7_push_7_7,
    &timesync_event_introuter_main_0_bus_outl_7_7_to_navss256l_main_0_bus_cpts0_hw8_push_8_8,
};
static const struct Sciclient_rmIrqNode tisci_irq_TIMESYNC_INTRTR0 = {
    .id = TISCI_DEV_TIMESYNC_INTRTR0,
    .n_if   = 25,
    .p_if   = &tisci_if_TIMESYNC_INTRTR0[0],
};

/* Start of UART0 interface definition */
const struct Sciclient_rmIrqIf usart_main_0_bus_usart_irq_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_104_104 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 104,
};
const struct Sciclient_rmIrqIf *const tisci_if_UART0[] = {
    &usart_main_0_bus_usart_irq_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_104_104,
};
static const struct Sciclient_rmIrqNode tisci_irq_UART0 = {
    .id = TISCI_DEV_UART0,
    .n_if   = 1,
    .p_if   = &tisci_if_UART0[0],
};

/* Start of UART1 interface definition */
const struct Sciclient_rmIrqIf usart_main_1_bus_usart_irq_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_105_105 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 105,
};
const struct Sciclient_rmIrqIf *const tisci_if_UART1[] = {
    &usart_main_1_bus_usart_irq_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_105_105,
};
static const struct Sciclient_rmIrqNode tisci_irq_UART1 = {
    .id = TISCI_DEV_UART1,
    .n_if   = 1,
    .p_if   = &tisci_if_UART1[0],
};

/* Start of UART2 interface definition */
const struct Sciclient_rmIrqIf usart_main_2_bus_usart_irq_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_106_106 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 106,
};
const struct Sciclient_rmIrqIf *const tisci_if_UART2[] = {
    &usart_main_2_bus_usart_irq_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_106_106,
};
static const struct Sciclient_rmIrqNode tisci_irq_UART2 = {
    .id = TISCI_DEV_UART2,
    .n_if   = 1,
    .p_if   = &tisci_if_UART2[0],
};

/* Start of USB3SS0 interface definition */
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_otg_lvl_14_14_to_main2mcu_lvl_introuter_main_0_bus_in_128_128 = {
    .lbase  = 14,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 128,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_misc_lvl_17_17_to_main2mcu_lvl_introuter_main_0_bus_in_129_129 = {
    .lbase  = 17,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 129,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_bc_lvl_18_18_to_main2mcu_lvl_introuter_main_0_bus_in_130_130 = {
    .lbase  = 18,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 130,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_pme_gen_lvl_16_16_to_main2mcu_lvl_introuter_main_0_bus_in_131_131 = {
    .lbase  = 16,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 131,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_i00_lvl_19_19_to_main2mcu_lvl_introuter_main_0_bus_in_132_132 = {
    .lbase  = 19,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 132,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_i01_lvl_8_8_to_main2mcu_lvl_introuter_main_0_bus_in_133_133 = {
    .lbase  = 8,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 133,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_i02_lvl_7_7_to_main2mcu_lvl_introuter_main_0_bus_in_134_134 = {
    .lbase  = 7,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 134,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_i03_lvl_13_13_to_main2mcu_lvl_introuter_main_0_bus_in_135_135 = {
    .lbase  = 13,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 135,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_i04_lvl_3_3_to_main2mcu_lvl_introuter_main_0_bus_in_136_136 = {
    .lbase  = 3,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 136,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_i05_lvl_12_12_to_main2mcu_lvl_introuter_main_0_bus_in_137_137 = {
    .lbase  = 12,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 137,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_i06_lvl_4_4_to_main2mcu_lvl_introuter_main_0_bus_in_138_138 = {
    .lbase  = 4,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 138,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_i07_lvl_6_6_to_main2mcu_lvl_introuter_main_0_bus_in_139_139 = {
    .lbase  = 6,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 139,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_i08_lvl_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_140_140 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 140,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_i09_lvl_11_11_to_main2mcu_lvl_introuter_main_0_bus_in_141_141 = {
    .lbase  = 11,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 141,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_i10_lvl_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_142_142 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 142,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_i11_lvl_20_20_to_main2mcu_lvl_introuter_main_0_bus_in_143_143 = {
    .lbase  = 20,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 143,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_i12_lvl_9_9_to_main2mcu_lvl_introuter_main_0_bus_in_144_144 = {
    .lbase  = 9,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 144,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_i13_lvl_15_15_to_main2mcu_lvl_introuter_main_0_bus_in_145_145 = {
    .lbase  = 15,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 145,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_i14_lvl_5_5_to_main2mcu_lvl_introuter_main_0_bus_in_146_146 = {
    .lbase  = 5,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 146,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_0_bus_i15_lvl_10_10_to_main2mcu_lvl_introuter_main_0_bus_in_147_147 = {
    .lbase  = 10,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 147,
};
const struct Sciclient_rmIrqIf *const tisci_if_USB3SS0[] = {
    &usb3ss2p0_gs80_main_0_bus_otg_lvl_14_14_to_main2mcu_lvl_introuter_main_0_bus_in_128_128,
    &usb3ss2p0_gs80_main_0_bus_misc_lvl_17_17_to_main2mcu_lvl_introuter_main_0_bus_in_129_129,
    &usb3ss2p0_gs80_main_0_bus_bc_lvl_18_18_to_main2mcu_lvl_introuter_main_0_bus_in_130_130,
    &usb3ss2p0_gs80_main_0_bus_pme_gen_lvl_16_16_to_main2mcu_lvl_introuter_main_0_bus_in_131_131,
    &usb3ss2p0_gs80_main_0_bus_i00_lvl_19_19_to_main2mcu_lvl_introuter_main_0_bus_in_132_132,
    &usb3ss2p0_gs80_main_0_bus_i01_lvl_8_8_to_main2mcu_lvl_introuter_main_0_bus_in_133_133,
    &usb3ss2p0_gs80_main_0_bus_i02_lvl_7_7_to_main2mcu_lvl_introuter_main_0_bus_in_134_134,
    &usb3ss2p0_gs80_main_0_bus_i03_lvl_13_13_to_main2mcu_lvl_introuter_main_0_bus_in_135_135,
    &usb3ss2p0_gs80_main_0_bus_i04_lvl_3_3_to_main2mcu_lvl_introuter_main_0_bus_in_136_136,
    &usb3ss2p0_gs80_main_0_bus_i05_lvl_12_12_to_main2mcu_lvl_introuter_main_0_bus_in_137_137,
    &usb3ss2p0_gs80_main_0_bus_i06_lvl_4_4_to_main2mcu_lvl_introuter_main_0_bus_in_138_138,
    &usb3ss2p0_gs80_main_0_bus_i07_lvl_6_6_to_main2mcu_lvl_introuter_main_0_bus_in_139_139,
    &usb3ss2p0_gs80_main_0_bus_i08_lvl_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_140_140,
    &usb3ss2p0_gs80_main_0_bus_i09_lvl_11_11_to_main2mcu_lvl_introuter_main_0_bus_in_141_141,
    &usb3ss2p0_gs80_main_0_bus_i10_lvl_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_142_142,
    &usb3ss2p0_gs80_main_0_bus_i11_lvl_20_20_to_main2mcu_lvl_introuter_main_0_bus_in_143_143,
    &usb3ss2p0_gs80_main_0_bus_i12_lvl_9_9_to_main2mcu_lvl_introuter_main_0_bus_in_144_144,
    &usb3ss2p0_gs80_main_0_bus_i13_lvl_15_15_to_main2mcu_lvl_introuter_main_0_bus_in_145_145,
    &usb3ss2p0_gs80_main_0_bus_i14_lvl_5_5_to_main2mcu_lvl_introuter_main_0_bus_in_146_146,
    &usb3ss2p0_gs80_main_0_bus_i15_lvl_10_10_to_main2mcu_lvl_introuter_main_0_bus_in_147_147,
};
static const struct Sciclient_rmIrqNode tisci_irq_USB3SS0 = {
    .id = TISCI_DEV_USB3SS0,
    .n_if   = 20,
    .p_if   = &tisci_if_USB3SS0[0],
};

/* Start of USB3SS1 interface definition */
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_otg_lvl_14_14_to_main2mcu_lvl_introuter_main_0_bus_in_148_148 = {
    .lbase  = 14,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 148,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_misc_lvl_17_17_to_main2mcu_lvl_introuter_main_0_bus_in_149_149 = {
    .lbase  = 17,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 149,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_bc_lvl_18_18_to_main2mcu_lvl_introuter_main_0_bus_in_150_150 = {
    .lbase  = 18,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 150,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_pme_gen_lvl_16_16_to_main2mcu_lvl_introuter_main_0_bus_in_151_151 = {
    .lbase  = 16,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 151,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_i00_lvl_19_19_to_main2mcu_lvl_introuter_main_0_bus_in_152_152 = {
    .lbase  = 19,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 152,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_i01_lvl_8_8_to_main2mcu_lvl_introuter_main_0_bus_in_153_153 = {
    .lbase  = 8,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 153,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_i02_lvl_7_7_to_main2mcu_lvl_introuter_main_0_bus_in_154_154 = {
    .lbase  = 7,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 154,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_i03_lvl_13_13_to_main2mcu_lvl_introuter_main_0_bus_in_155_155 = {
    .lbase  = 13,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 155,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_i04_lvl_3_3_to_main2mcu_lvl_introuter_main_0_bus_in_156_156 = {
    .lbase  = 3,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 156,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_i05_lvl_12_12_to_main2mcu_lvl_introuter_main_0_bus_in_157_157 = {
    .lbase  = 12,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 157,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_i06_lvl_4_4_to_main2mcu_lvl_introuter_main_0_bus_in_158_158 = {
    .lbase  = 4,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 158,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_i07_lvl_6_6_to_main2mcu_lvl_introuter_main_0_bus_in_159_159 = {
    .lbase  = 6,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 159,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_i08_lvl_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_160_160 = {
    .lbase  = 2,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 160,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_i09_lvl_11_11_to_main2mcu_lvl_introuter_main_0_bus_in_161_161 = {
    .lbase  = 11,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 161,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_i10_lvl_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_162_162 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 162,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_i11_lvl_20_20_to_main2mcu_lvl_introuter_main_0_bus_in_163_163 = {
    .lbase  = 20,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 163,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_i12_lvl_9_9_to_main2mcu_lvl_introuter_main_0_bus_in_164_164 = {
    .lbase  = 9,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 164,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_i13_lvl_15_15_to_main2mcu_lvl_introuter_main_0_bus_in_165_165 = {
    .lbase  = 15,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 165,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_i14_lvl_5_5_to_main2mcu_lvl_introuter_main_0_bus_in_166_166 = {
    .lbase  = 5,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 166,
};
const struct Sciclient_rmIrqIf usb3ss2p0_gs80_main_1_bus_i15_lvl_10_10_to_main2mcu_lvl_introuter_main_0_bus_in_167_167 = {
    .lbase  = 10,
    .len    = 1,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 167,
};
const struct Sciclient_rmIrqIf *const tisci_if_USB3SS1[] = {
    &usb3ss2p0_gs80_main_1_bus_otg_lvl_14_14_to_main2mcu_lvl_introuter_main_0_bus_in_148_148,
    &usb3ss2p0_gs80_main_1_bus_misc_lvl_17_17_to_main2mcu_lvl_introuter_main_0_bus_in_149_149,
    &usb3ss2p0_gs80_main_1_bus_bc_lvl_18_18_to_main2mcu_lvl_introuter_main_0_bus_in_150_150,
    &usb3ss2p0_gs80_main_1_bus_pme_gen_lvl_16_16_to_main2mcu_lvl_introuter_main_0_bus_in_151_151,
    &usb3ss2p0_gs80_main_1_bus_i00_lvl_19_19_to_main2mcu_lvl_introuter_main_0_bus_in_152_152,
    &usb3ss2p0_gs80_main_1_bus_i01_lvl_8_8_to_main2mcu_lvl_introuter_main_0_bus_in_153_153,
    &usb3ss2p0_gs80_main_1_bus_i02_lvl_7_7_to_main2mcu_lvl_introuter_main_0_bus_in_154_154,
    &usb3ss2p0_gs80_main_1_bus_i03_lvl_13_13_to_main2mcu_lvl_introuter_main_0_bus_in_155_155,
    &usb3ss2p0_gs80_main_1_bus_i04_lvl_3_3_to_main2mcu_lvl_introuter_main_0_bus_in_156_156,
    &usb3ss2p0_gs80_main_1_bus_i05_lvl_12_12_to_main2mcu_lvl_introuter_main_0_bus_in_157_157,
    &usb3ss2p0_gs80_main_1_bus_i06_lvl_4_4_to_main2mcu_lvl_introuter_main_0_bus_in_158_158,
    &usb3ss2p0_gs80_main_1_bus_i07_lvl_6_6_to_main2mcu_lvl_introuter_main_0_bus_in_159_159,
    &usb3ss2p0_gs80_main_1_bus_i08_lvl_2_2_to_main2mcu_lvl_introuter_main_0_bus_in_160_160,
    &usb3ss2p0_gs80_main_1_bus_i09_lvl_11_11_to_main2mcu_lvl_introuter_main_0_bus_in_161_161,
    &usb3ss2p0_gs80_main_1_bus_i10_lvl_0_0_to_main2mcu_lvl_introuter_main_0_bus_in_162_162,
    &usb3ss2p0_gs80_main_1_bus_i11_lvl_20_20_to_main2mcu_lvl_introuter_main_0_bus_in_163_163,
    &usb3ss2p0_gs80_main_1_bus_i12_lvl_9_9_to_main2mcu_lvl_introuter_main_0_bus_in_164_164,
    &usb3ss2p0_gs80_main_1_bus_i13_lvl_15_15_to_main2mcu_lvl_introuter_main_0_bus_in_165_165,
    &usb3ss2p0_gs80_main_1_bus_i14_lvl_5_5_to_main2mcu_lvl_introuter_main_0_bus_in_166_166,
    &usb3ss2p0_gs80_main_1_bus_i15_lvl_10_10_to_main2mcu_lvl_introuter_main_0_bus_in_167_167,
};
static const struct Sciclient_rmIrqNode tisci_irq_USB3SS1 = {
    .id = TISCI_DEV_USB3SS1,
    .n_if   = 20,
    .p_if   = &tisci_if_USB3SS1[0],
};

/* Start of WKUP_GPIOMUX_INTRTR0 interface definition */
const struct Sciclient_rmIrqIf wkup_gpiomux_introuter_wkup_0_bus_outp_0_15_to_mcu_armss0_cpu0_bus_intr_124_139 = {
    .lbase  = 0,
    .len    = 16,
    .rid    = TISCI_DEV_MCU_ARMSS0_CPU0,
    .rbase  = 124,
};
const struct Sciclient_rmIrqIf wkup_gpiomux_introuter_wkup_0_bus_outp_0_15_to_mcu_armss0_cpu1_bus_intr_124_139 = {
    .lbase  = 0,
    .len    = 16,
    .rid    = TISCI_DEV_MCU_ARMSS0_CPU1,
    .rbase  = 124,
};
const struct Sciclient_rmIrqIf wkup_gpiomux_introuter_wkup_0_bus_outp_0_15_to_gic500ss_main_0_bus_spi_712_727 = {
    .lbase  = 0,
    .len    = 16,
    .rid    = TISCI_DEV_GIC0,
    .rbase  = 712,
};
const struct Sciclient_rmIrqIf wkup_gpiomux_introuter_wkup_0_bus_outp_0_11_to_dmsc_wkup_0_bus_int_8_19 = {
    .lbase  = 0,
    .len    = 12,
    .rid    = TISCI_DEV_WKUP_DMSC0,
    .rbase  = 8,
};
const struct Sciclient_rmIrqIf wkup_gpiomux_introuter_wkup_0_bus_outp_8_15_to_esm_wkup_wkup_0_bus_esm_pls_event0_256_263 = {
    .lbase  = 8,
    .len    = 8,
    .rid    = TISCI_DEV_WKUP_ESM0,
    .rbase  = 256,
};
const struct Sciclient_rmIrqIf wkup_gpiomux_introuter_wkup_0_bus_outp_8_15_to_esm_wkup_wkup_0_bus_esm_pls_event1_264_271 = {
    .lbase  = 8,
    .len    = 8,
    .rid    = TISCI_DEV_WKUP_ESM0,
    .rbase  = 264,
};
const struct Sciclient_rmIrqIf wkup_gpiomux_introuter_wkup_0_bus_outp_8_15_to_esm_wkup_wkup_0_bus_esm_pls_event2_88_95 = {
    .lbase  = 8,
    .len    = 8,
    .rid    = TISCI_DEV_WKUP_ESM0,
    .rbase  = 88,
};
const struct Sciclient_rmIrqIf *const tisci_if_WKUP_GPIOMUX_INTRTR0[] = {
    &wkup_gpiomux_introuter_wkup_0_bus_outp_0_15_to_mcu_armss0_cpu0_bus_intr_124_139,
    &wkup_gpiomux_introuter_wkup_0_bus_outp_0_15_to_mcu_armss0_cpu1_bus_intr_124_139,
    &wkup_gpiomux_introuter_wkup_0_bus_outp_0_15_to_gic500ss_main_0_bus_spi_712_727,
    &wkup_gpiomux_introuter_wkup_0_bus_outp_0_11_to_dmsc_wkup_0_bus_int_8_19,
    &wkup_gpiomux_introuter_wkup_0_bus_outp_8_15_to_esm_wkup_wkup_0_bus_esm_pls_event0_256_263,
    &wkup_gpiomux_introuter_wkup_0_bus_outp_8_15_to_esm_wkup_wkup_0_bus_esm_pls_event1_264_271,
    &wkup_gpiomux_introuter_wkup_0_bus_outp_8_15_to_esm_wkup_wkup_0_bus_esm_pls_event2_88_95,
};
static const struct Sciclient_rmIrqNode tisci_irq_WKUP_GPIOMUX_INTRTR0 = {
    .id = TISCI_DEV_WKUP_GPIOMUX_INTRTR0,
    .n_if   = 7,
    .p_if   = &tisci_if_WKUP_GPIOMUX_INTRTR0[0],
};

/* Start of navss0_cpts0 interface definition */
const struct Sciclient_rmIrqIf navss0_cpts0_event_pend_intr_0_0_to_navss0_intr_router_0_in_intr_391_391 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 391,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_cpts0[] = {
    &navss0_cpts0_event_pend_intr_0_0_to_navss0_intr_router_0_in_intr_391_391,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_cpts0 = {
    .id = TISCI_DEV_NAVSS0_CPTS0,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_cpts0[0],
};

/* Start of navss0_mailbox0_cluster0 interface definition */
const struct Sciclient_rmIrqIf navss0_mailbox0_cluster0_pend_intr_0_3_to_navss0_intr_router_0_in_intr_436_439 = {
    .lbase  = 0,
    .len    = 4,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 436,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_mailbox0_cluster0[] = {
    &navss0_mailbox0_cluster0_pend_intr_0_3_to_navss0_intr_router_0_in_intr_436_439,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_mailbox0_cluster0 = {
    .id = TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER0,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_mailbox0_cluster0[0],
};

/* Start of navss0_mailbox0_cluster1 interface definition */
const struct Sciclient_rmIrqIf navss0_mailbox0_cluster1_pend_intr_0_3_to_navss0_intr_router_0_in_intr_432_435 = {
    .lbase  = 0,
    .len    = 4,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 432,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_mailbox0_cluster1[] = {
    &navss0_mailbox0_cluster1_pend_intr_0_3_to_navss0_intr_router_0_in_intr_432_435,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_mailbox0_cluster1 = {
    .id = TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER1,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_mailbox0_cluster1[0],
};

/* Start of navss0_mailbox0_cluster2 interface definition */
const struct Sciclient_rmIrqIf navss0_mailbox0_cluster2_pend_intr_0_3_to_navss0_intr_router_0_in_intr_428_431 = {
    .lbase  = 0,
    .len    = 4,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 428,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_mailbox0_cluster2[] = {
    &navss0_mailbox0_cluster2_pend_intr_0_3_to_navss0_intr_router_0_in_intr_428_431,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_mailbox0_cluster2 = {
    .id = TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER2,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_mailbox0_cluster2[0],
};

/* Start of navss0_mailbox0_cluster3 interface definition */
const struct Sciclient_rmIrqIf navss0_mailbox0_cluster3_pend_intr_0_3_to_navss0_intr_router_0_in_intr_424_427 = {
    .lbase  = 0,
    .len    = 4,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 424,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_mailbox0_cluster3[] = {
    &navss0_mailbox0_cluster3_pend_intr_0_3_to_navss0_intr_router_0_in_intr_424_427,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_mailbox0_cluster3 = {
    .id = TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER3,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_mailbox0_cluster3[0],
};

/* Start of navss0_mailbox0_cluster4 interface definition */
const struct Sciclient_rmIrqIf navss0_mailbox0_cluster4_pend_intr_0_3_to_navss0_intr_router_0_in_intr_420_423 = {
    .lbase  = 0,
    .len    = 4,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 420,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_mailbox0_cluster4[] = {
    &navss0_mailbox0_cluster4_pend_intr_0_3_to_navss0_intr_router_0_in_intr_420_423,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_mailbox0_cluster4 = {
    .id = TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER4,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_mailbox0_cluster4[0],
};

/* Start of navss0_mailbox0_cluster5 interface definition */
const struct Sciclient_rmIrqIf navss0_mailbox0_cluster5_pend_intr_0_3_to_navss0_intr_router_0_in_intr_416_419 = {
    .lbase  = 0,
    .len    = 4,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 416,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_mailbox0_cluster5[] = {
    &navss0_mailbox0_cluster5_pend_intr_0_3_to_navss0_intr_router_0_in_intr_416_419,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_mailbox0_cluster5 = {
    .id = TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER5,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_mailbox0_cluster5[0],
};

/* Start of navss0_mailbox0_cluster6 interface definition */
const struct Sciclient_rmIrqIf navss0_mailbox0_cluster6_pend_intr_0_3_to_navss0_intr_router_0_in_intr_412_415 = {
    .lbase  = 0,
    .len    = 4,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 412,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_mailbox0_cluster6[] = {
    &navss0_mailbox0_cluster6_pend_intr_0_3_to_navss0_intr_router_0_in_intr_412_415,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_mailbox0_cluster6 = {
    .id = TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER6,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_mailbox0_cluster6[0],
};

/* Start of navss0_mailbox0_cluster7 interface definition */
const struct Sciclient_rmIrqIf navss0_mailbox0_cluster7_pend_intr_0_3_to_navss0_intr_router_0_in_intr_408_411 = {
    .lbase  = 0,
    .len    = 4,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 408,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_mailbox0_cluster7[] = {
    &navss0_mailbox0_cluster7_pend_intr_0_3_to_navss0_intr_router_0_in_intr_408_411,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_mailbox0_cluster7 = {
    .id = TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER7,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_mailbox0_cluster7[0],
};

/* Start of navss0_mailbox0_cluster8 interface definition */
const struct Sciclient_rmIrqIf navss0_mailbox0_cluster8_pend_intr_0_3_to_navss0_intr_router_0_in_intr_404_407 = {
    .lbase  = 0,
    .len    = 4,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 404,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_mailbox0_cluster8[] = {
    &navss0_mailbox0_cluster8_pend_intr_0_3_to_navss0_intr_router_0_in_intr_404_407,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_mailbox0_cluster8 = {
    .id = TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER8,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_mailbox0_cluster8[0],
};

/* Start of navss0_mailbox0_cluster9 interface definition */
const struct Sciclient_rmIrqIf navss0_mailbox0_cluster9_pend_intr_0_3_to_navss0_intr_router_0_in_intr_400_403 = {
    .lbase  = 0,
    .len    = 4,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 400,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_mailbox0_cluster9[] = {
    &navss0_mailbox0_cluster9_pend_intr_0_3_to_navss0_intr_router_0_in_intr_400_403,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_mailbox0_cluster9 = {
    .id = TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER9,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_mailbox0_cluster9[0],
};

/* Start of navss0_mailbox0_cluster10 interface definition */
const struct Sciclient_rmIrqIf navss0_mailbox0_cluster10_pend_intr_0_3_to_navss0_intr_router_0_in_intr_396_399 = {
    .lbase  = 0,
    .len    = 4,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 396,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_mailbox0_cluster10[] = {
    &navss0_mailbox0_cluster10_pend_intr_0_3_to_navss0_intr_router_0_in_intr_396_399,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_mailbox0_cluster10 = {
    .id = TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER10,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_mailbox0_cluster10[0],
};

/* Start of navss0_mailbox0_cluster11 interface definition */
const struct Sciclient_rmIrqIf navss0_mailbox0_cluster11_pend_intr_0_3_to_navss0_intr_router_0_in_intr_392_395 = {
    .lbase  = 0,
    .len    = 4,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 392,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_mailbox0_cluster11[] = {
    &navss0_mailbox0_cluster11_pend_intr_0_3_to_navss0_intr_router_0_in_intr_392_395,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_mailbox0_cluster11 = {
    .id = TISCI_DEV_NAVSS0_MAILBOX0_CLUSTER11,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_mailbox0_cluster11[0],
};

/* Start of navss0_mcrc0 interface definition */
const struct Sciclient_rmIrqIf navss0_mcrc0_dma_event_intr_0_3_to_navss0_intr_router_0_in_intr_384_387 = {
    .lbase  = 0,
    .len    = 4,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 384,
};
const struct Sciclient_rmIrqIf navss0_mcrc0_int_mcrc_intr_8_8_to_navss0_intr_router_0_in_intr_388_388 = {
    .lbase  = 8,
    .len    = 1,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 388,
};
const struct Sciclient_rmIrqIf navss0_mcrc0_dma_event_intr_0_3_to_navss0_udmass_inta0_intaggr_levt_pend_0_3 = {
    .lbase  = 0,
    .len    = 4,
    .rid    = TISCI_DEV_NAVSS0_UDMASS_INTA0,
    .rbase  = 0,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_mcrc0[] = {
    &navss0_mcrc0_dma_event_intr_0_3_to_navss0_intr_router_0_in_intr_384_387,
    &navss0_mcrc0_int_mcrc_intr_8_8_to_navss0_intr_router_0_in_intr_388_388,
    &navss0_mcrc0_dma_event_intr_0_3_to_navss0_udmass_inta0_intaggr_levt_pend_0_3,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_mcrc0 = {
    .id = TISCI_DEV_NAVSS0_MCRC0,
    .n_if   = 3,
    .p_if   = &tisci_if_navss0_mcrc0[0],
};

/* Start of navss0_pvu0 interface definition */
const struct Sciclient_rmIrqIf navss0_pvu0_pend_intr_0_0_to_navss0_intr_router_0_in_intr_390_390 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 390,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_pvu0[] = {
    &navss0_pvu0_pend_intr_0_0_to_navss0_intr_router_0_in_intr_390_390,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_pvu0 = {
    .id = TISCI_DEV_NAVSS0_PVU0,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_pvu0[0],
};

/* Start of navss0_pvu1 interface definition */
const struct Sciclient_rmIrqIf navss0_pvu1_pend_intr_0_0_to_navss0_intr_router_0_in_intr_389_389 = {
    .lbase  = 0,
    .len    = 1,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 389,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_pvu1[] = {
    &navss0_pvu1_pend_intr_0_0_to_navss0_intr_router_0_in_intr_389_389,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_pvu1 = {
    .id = TISCI_DEV_NAVSS0_PVU1,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_pvu1[0],
};

/* Start of navss0_udmass_inta0 interface definition */
const struct Sciclient_rmIrqIf navss0_udmass_inta0_intaggr_vintr_pend_0_255_to_navss0_intr_router_0_in_intr_0_255 = {
    .lbase  = 0,
    .len    = 256,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 0,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_udmass_inta0[] = {
    &navss0_udmass_inta0_intaggr_vintr_pend_0_255_to_navss0_intr_router_0_in_intr_0_255,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_udmass_inta0 = {
    .id = TISCI_DEV_NAVSS0_UDMASS_INTA0,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_udmass_inta0[0],
};

/* Start of navss0_modss_inta0 interface definition */
const struct Sciclient_rmIrqIf navss0_modss_inta0_intaggr_vintr_pend_0_63_to_navss0_intr_router_0_in_intr_320_383 = {
    .lbase  = 0,
    .len    = 64,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 320,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_modss_inta0[] = {
    &navss0_modss_inta0_intaggr_vintr_pend_0_63_to_navss0_intr_router_0_in_intr_320_383,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_modss_inta0 = {
    .id = TISCI_DEV_NAVSS0_MODSS_INTA0,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_modss_inta0[0],
};

/* Start of navss0_modss_inta1 interface definition */
const struct Sciclient_rmIrqIf navss0_modss_inta1_intaggr_vintr_pend_0_63_to_navss0_intr_router_0_in_intr_256_319 = {
    .lbase  = 0,
    .len    = 64,
    .rid    = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .rbase  = 256,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_modss_inta1[] = {
    &navss0_modss_inta1_intaggr_vintr_pend_0_63_to_navss0_intr_router_0_in_intr_256_319,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_modss_inta1 = {
    .id = TISCI_DEV_NAVSS0_MODSS_INTA1,
    .n_if   = 1,
    .p_if   = &tisci_if_navss0_modss_inta1[0],
};

/* Start of navss0_intr_router_0 interface definition */
const struct Sciclient_rmIrqIf navss0_intr_router_0_outl_intr_136_143_to_icss_g_main_1_bus_pr1_slv_intr_46_53 = {
    .lbase  = 136,
    .len    = 8,
    .rid    = TISCI_DEV_PRU_ICSSG1,
    .rbase  = 46,
};
const struct Sciclient_rmIrqIf navss0_intr_router_0_outl_intr_128_135_to_icss_g_main_0_bus_pr1_slv_intr_46_53 = {
    .lbase  = 128,
    .len    = 8,
    .rid    = TISCI_DEV_PRU_ICSSG0,
    .rbase  = 46,
};
const struct Sciclient_rmIrqIf navss0_intr_router_0_outl_intr_0_63_to_gic500ss_main_0_bus_spi_64_127 = {
    .lbase  = 0,
    .len    = 64,
    .rid    = TISCI_DEV_GIC0,
    .rbase  = 64,
};
const struct Sciclient_rmIrqIf navss0_intr_router_0_outl_intr_64_119_to_gic500ss_main_0_bus_spi_448_503 = {
    .lbase  = 64,
    .len    = 56,
    .rid    = TISCI_DEV_GIC0,
    .rbase  = 448,
};
const struct Sciclient_rmIrqIf navss0_intr_router_0_outl_intr_144_151_to_icss_g_main_2_bus_pr1_slv_intr_46_53 = {
    .lbase  = 144,
    .len    = 8,
    .rid    = TISCI_DEV_PRU_ICSSG2,
    .rbase  = 46,
};
const struct Sciclient_rmIrqIf navss0_intr_router_0_outl_intr_120_127_to_main2mcu_lvl_introuter_main_0_bus_in_184_191 = {
    .lbase  = 120,
    .len    = 8,
    .rid    = TISCI_DEV_MAIN2MCU_LVL_INTRTR0,
    .rbase  = 184,
};
const struct Sciclient_rmIrqIf *const tisci_if_navss0_intr_router_0[] = {
    &navss0_intr_router_0_outl_intr_136_143_to_icss_g_main_1_bus_pr1_slv_intr_46_53,
    &navss0_intr_router_0_outl_intr_128_135_to_icss_g_main_0_bus_pr1_slv_intr_46_53,
    &navss0_intr_router_0_outl_intr_0_63_to_gic500ss_main_0_bus_spi_64_127,
    &navss0_intr_router_0_outl_intr_64_119_to_gic500ss_main_0_bus_spi_448_503,
    &navss0_intr_router_0_outl_intr_144_151_to_icss_g_main_2_bus_pr1_slv_intr_46_53,
    &navss0_intr_router_0_outl_intr_120_127_to_main2mcu_lvl_introuter_main_0_bus_in_184_191,
};
static const struct Sciclient_rmIrqNode tisci_irq_navss0_intr_router_0 = {
    .id = TISCI_DEV_NAVSS0_INTR_ROUTER_0,
    .n_if   = 6,
    .p_if   = &tisci_if_navss0_intr_router_0[0],
};

/* Start of mcu_navss0_intr_aggr_0 interface definition */
const struct Sciclient_rmIrqIf mcu_navss0_intr_aggr_0_intaggr_vintr_pend_0_255_to_mcu_navss0_intr_router_0_in_intr_0_255 = {
    .lbase  = 0,
    .len    = 256,
    .rid    = TISCI_DEV_MCU_NAVSS0_INTR_ROUTER_0,
    .rbase  = 0,
};
const struct Sciclient_rmIrqIf *const tisci_if_mcu_navss0_intr_aggr_0[] = {
    &mcu_navss0_intr_aggr_0_intaggr_vintr_pend_0_255_to_mcu_navss0_intr_router_0_in_intr_0_255,
};
static const struct Sciclient_rmIrqNode tisci_irq_mcu_navss0_intr_aggr_0 = {
    .id = TISCI_DEV_MCU_NAVSS0_INTR_AGGR_0,
    .n_if   = 1,
    .p_if   = &tisci_if_mcu_navss0_intr_aggr_0[0],
};

/* Start of mcu_navss0_intr_router_0 interface definition */
const struct Sciclient_rmIrqIf mcu_navss0_intr_router_0_outl_intr_0_31_to_mcu_armss0_cpu0_bus_intr_64_95 = {
    .lbase  = 0,
    .len    = 32,
    .rid    = TISCI_DEV_MCU_ARMSS0_CPU0,
    .rbase  = 64,
};
const struct Sciclient_rmIrqIf mcu_navss0_intr_router_0_outl_intr_32_63_to_mcu_armss0_cpu1_bus_intr_64_95 = {
    .lbase  = 32,
    .len    = 32,
    .rid    = TISCI_DEV_MCU_ARMSS0_CPU1,
    .rbase  = 64,
};
const struct Sciclient_rmIrqIf *const tisci_if_mcu_navss0_intr_router_0[] = {
    &mcu_navss0_intr_router_0_outl_intr_0_31_to_mcu_armss0_cpu0_bus_intr_64_95,
    &mcu_navss0_intr_router_0_outl_intr_32_63_to_mcu_armss0_cpu1_bus_intr_64_95,
};
static const struct Sciclient_rmIrqNode tisci_irq_mcu_navss0_intr_router_0 = {
    .id = TISCI_DEV_MCU_NAVSS0_INTR_ROUTER_0,
    .n_if   = 2,
    .p_if   = &tisci_if_mcu_navss0_intr_router_0[0],
};


const struct Sciclient_rmIrqNode *const gRmIrqTree[] = {
    &tisci_irq_CAL0,
    &tisci_irq_CMPEVENT_INTRTR0,
    &tisci_irq_MCU_CPSW0,
    &tisci_irq_DCC0,
    &tisci_irq_DCC1,
    &tisci_irq_DCC2,
    &tisci_irq_DCC3,
    &tisci_irq_DCC4,
    &tisci_irq_DCC5,
    &tisci_irq_DCC6,
    &tisci_irq_DCC7,
    &tisci_irq_DDRSS0,
    &tisci_irq_TIMER0,
    &tisci_irq_TIMER1,
    &tisci_irq_TIMER10,
    &tisci_irq_TIMER11,
    &tisci_irq_TIMER2,
    &tisci_irq_TIMER3,
    &tisci_irq_TIMER4,
    &tisci_irq_TIMER5,
    &tisci_irq_TIMER6,
    &tisci_irq_TIMER7,
    &tisci_irq_TIMER8,
    &tisci_irq_TIMER9,
    &tisci_irq_ECAP0,
    &tisci_irq_EHRPWM0,
    &tisci_irq_EHRPWM1,
    &tisci_irq_EHRPWM2,
    &tisci_irq_EHRPWM3,
    &tisci_irq_EHRPWM4,
    &tisci_irq_EHRPWM5,
    &tisci_irq_ELM0,
    &tisci_irq_MMCSD0,
    &tisci_irq_MMCSD1,
    &tisci_irq_EQEP0,
    &tisci_irq_EQEP1,
    &tisci_irq_EQEP2,
    &tisci_irq_GPIO0,
    &tisci_irq_GPIO1,
    &tisci_irq_WKUP_GPIO0,
    &tisci_irq_GPMC0,
    &tisci_irq_PRU_ICSSG0,
    &tisci_irq_PRU_ICSSG1,
    &tisci_irq_PRU_ICSSG2,
    &tisci_irq_GPU0,
    &tisci_irq_CCDEBUGSS0,
    &tisci_irq_DSS0,
    &tisci_irq_DEBUGSS0,
    &tisci_irq_CBASS0,
    &tisci_irq_CBASS_DEBUG0,
    &tisci_irq_CBASS_FW0,
    &tisci_irq_CBASS_INFRA0,
    &tisci_irq_MAIN2MCU_LVL_INTRTR0,
    &tisci_irq_MAIN2MCU_PLS_INTRTR0,
    &tisci_irq_CTRL_MMR0,
    &tisci_irq_GPIOMUX_INTRTR0,
    &tisci_irq_MCASP0,
    &tisci_irq_MCASP1,
    &tisci_irq_MCASP2,
    &tisci_irq_I2C0,
    &tisci_irq_I2C1,
    &tisci_irq_I2C2,
    &tisci_irq_I2C3,
    &tisci_irq_NAVSS0,
    &tisci_irq_PCIE0,
    &tisci_irq_PCIE1,
    &tisci_irq_SA2_UL0,
    &tisci_irq_MCSPI0,
    &tisci_irq_MCSPI1,
    &tisci_irq_MCSPI2,
    &tisci_irq_MCSPI3,
    &tisci_irq_TIMESYNC_INTRTR0,
    &tisci_irq_UART0,
    &tisci_irq_UART1,
    &tisci_irq_UART2,
    &tisci_irq_USB3SS0,
    &tisci_irq_USB3SS1,
    &tisci_irq_WKUP_GPIOMUX_INTRTR0,
    &tisci_irq_navss0_cpts0,
    &tisci_irq_navss0_mailbox0_cluster0,
    &tisci_irq_navss0_mailbox0_cluster1,
    &tisci_irq_navss0_mailbox0_cluster2,
    &tisci_irq_navss0_mailbox0_cluster3,
    &tisci_irq_navss0_mailbox0_cluster4,
    &tisci_irq_navss0_mailbox0_cluster5,
    &tisci_irq_navss0_mailbox0_cluster6,
    &tisci_irq_navss0_mailbox0_cluster7,
    &tisci_irq_navss0_mailbox0_cluster8,
    &tisci_irq_navss0_mailbox0_cluster9,
    &tisci_irq_navss0_mailbox0_cluster10,
    &tisci_irq_navss0_mailbox0_cluster11,
    &tisci_irq_navss0_mcrc0,
    &tisci_irq_navss0_pvu0,
    &tisci_irq_navss0_pvu1,
    &tisci_irq_navss0_udmass_inta0,
    &tisci_irq_navss0_modss_inta0,
    &tisci_irq_navss0_modss_inta1,
    &tisci_irq_navss0_intr_router_0,
    &tisci_irq_mcu_navss0_intr_aggr_0,
    &tisci_irq_mcu_navss0_intr_router_0,
};

const uint32_t gRmIrqTreeCount = sizeof(gRmIrqTree)/sizeof(gRmIrqTree[0]);
