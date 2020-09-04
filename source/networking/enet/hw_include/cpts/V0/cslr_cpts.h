/********************************************************************
 * Copyright (C) 2018-2019 Texas Instruments Incorporated.
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
 *
 *  Name        : cslr_cpts.h
*/
#ifndef CSLR_CPTS_H_
#define CSLR_CPTS_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Hardware Region  :
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t COMP_LOW_REG;              /* comp_low_reg */
    volatile uint32_t COMP_HIGH_REG;             /* comp_high_reg */
    volatile uint32_t CONTROL_REG;               /* control_reg */
    volatile uint32_t LENGTH_REG;                /* length_reg */
    volatile uint32_t PPM_LOW_REG;               /* ppm_low_reg */
    volatile uint32_t PPM_HIGH_REG;              /* ppm_high_reg */
    volatile uint32_t NUDGE_REG;                 /* nudge_reg */
    volatile uint8_t  Resv_32[4];
} CSL_cptsRegs_TS_GENF;


typedef struct {
    volatile uint32_t COMP_LOW_REG;              /* comp_low_reg */
    volatile uint32_t COMP_HIGH_REG;             /* comp_high_reg */
    volatile uint32_t CONTROL_REG;               /* control_reg */
    volatile uint32_t LENGTH_REG;                /* length_reg */
    volatile uint32_t PPM_LOW_REG;               /* ppm_low_reg */
    volatile uint32_t PPM_HIGH_REG;              /* ppm_high_reg */
    volatile uint32_t NUDGE_REG;                 /* nudge_reg */
    volatile uint8_t  Resv_32[4];
} CSL_cptsRegs_TS_ESTF;


typedef struct {
    volatile uint32_t IDVER_REG;                 /* idver_reg */
    volatile uint32_t CONTROL_REG;               /* control_reg */
    volatile uint32_t RFTCLK_SEL_REG;            /* rftclk_sel_reg */
    volatile uint32_t TS_PUSH_REG;               /* ts_push_reg */
    volatile uint32_t TS_LOAD_VAL_REG;           /* ts_load_low_val_reg */
    volatile uint32_t TS_LOAD_EN_REG;            /* ts_load_en_reg */
    volatile uint32_t TS_COMP_VAL_REG;           /* ts_comp_low_val_reg */
    volatile uint32_t TS_COMP_LEN_REG;           /* ts_comp_len_reg */
    volatile uint32_t INTSTAT_RAW_REG;           /* intstat_raw_reg */
    volatile uint32_t INTSTAT_MASKED_REG;        /* intstat_masked_reg */
    volatile uint32_t INT_ENABLE_REG;            /* int_enable_reg */
    volatile uint32_t TS_COMP_NUDGE_REG;         /* ts_comp_nudge_reg */
    volatile uint32_t EVENT_POP_REG;             /* event_pop_reg */
    volatile uint32_t EVENT_0_REG;               /* event_0_reg */
    volatile uint32_t EVENT_1_REG;               /* event_1_reg */
    volatile uint32_t EVENT_2_REG;               /* event_2_reg */
    volatile uint32_t EVENT_3_REG;               /* event_3_reg */
    volatile uint32_t TS_LOAD_HIGH_VAL_REG;      /* ts_load_high_val_reg */
    volatile uint32_t TS_COMP_HIGH_VAL_REG;      /* ts_comp_high_val_reg */
    volatile uint32_t TS_ADD_VAL_REG;            /* ts_add_val */
    volatile uint32_t TS_PPM_LOW_VAL_REG;        /* ts_ppm_low_val_reg */
    volatile uint32_t TS_PPM_HIGH_VAL_REG;       /* ts_ppm_high_val_reg */
    volatile uint32_t TS_NUDGE_VAL_REG;          /* ts_nudge_val_reg */
    volatile uint8_t  Resv_224[132];
    CSL_cptsRegs_TS_GENF TS_GENF[2];
    volatile uint8_t  Resv_512[224];
    CSL_cptsRegs_TS_ESTF TS_ESTF[8];
} CSL_cptsRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_CPTS_IDVER_REG                                                     (0x00000000U)
#define CSL_CPTS_CONTROL_REG                                                   (0x00000004U)
#define CSL_CPTS_RFTCLK_SEL_REG                                                (0x00000008U)
#define CSL_CPTS_TS_PUSH_REG                                                   (0x0000000CU)
#define CSL_CPTS_TS_LOAD_VAL_REG                                               (0x00000010U)
#define CSL_CPTS_TS_LOAD_EN_REG                                                (0x00000014U)
#define CSL_CPTS_TS_COMP_VAL_REG                                               (0x00000018U)
#define CSL_CPTS_TS_COMP_LEN_REG                                               (0x0000001CU)
#define CSL_CPTS_INTSTAT_RAW_REG                                               (0x00000020U)
#define CSL_CPTS_INTSTAT_MASKED_REG                                            (0x00000024U)
#define CSL_CPTS_INT_ENABLE_REG                                                (0x00000028U)
#define CSL_CPTS_TS_COMP_NUDGE_REG                                             (0x0000002CU)
#define CSL_CPTS_EVENT_POP_REG                                                 (0x00000030U)
#define CSL_CPTS_EVENT_0_REG                                                   (0x00000034U)
#define CSL_CPTS_EVENT_1_REG                                                   (0x00000038U)
#define CSL_CPTS_EVENT_2_REG                                                   (0x0000003CU)
#define CSL_CPTS_EVENT_3_REG                                                   (0x00000040U)
#define CSL_CPTS_TS_LOAD_HIGH_VAL_REG                                          (0x00000044U)
#define CSL_CPTS_TS_COMP_HIGH_VAL_REG                                          (0x00000048U)
#define CSL_CPTS_TS_ADD_VAL_REG                                                (0x0000004CU)
#define CSL_CPTS_TS_PPM_LOW_VAL_REG                                            (0x00000050U)
#define CSL_CPTS_TS_PPM_HIGH_VAL_REG                                           (0x00000054U)
#define CSL_CPTS_TS_NUDGE_VAL_REG                                              (0x00000058U)
#define CSL_CPTS_TS_GENF_COMP_LOW_REG(TS_GENF)                                 (0x000000E0U+((TS_GENF)*0x20U))
#define CSL_CPTS_TS_GENF_COMP_HIGH_REG(TS_GENF)                                (0x000000E4U+((TS_GENF)*0x20U))
#define CSL_CPTS_TS_GENF_CONTROL_REG(TS_GENF)                                  (0x000000E8U+((TS_GENF)*0x20U))
#define CSL_CPTS_TS_GENF_LENGTH_REG(TS_GENF)                                   (0x000000ECU+((TS_GENF)*0x20U))
#define CSL_CPTS_TS_GENF_PPM_LOW_REG(TS_GENF)                                  (0x000000F0U+((TS_GENF)*0x20U))
#define CSL_CPTS_TS_GENF_PPM_HIGH_REG(TS_GENF)                                 (0x000000F4U+((TS_GENF)*0x20U))
#define CSL_CPTS_TS_GENF_NUDGE_REG(TS_GENF)                                    (0x000000F8U+((TS_GENF)*0x20U))
#define CSL_CPTS_TS_ESTF_COMP_LOW_REG(TS_ESTF)                                 (0x00000200U+((TS_ESTF)*0x20U))
#define CSL_CPTS_TS_ESTF_COMP_HIGH_REG(TS_ESTF)                                (0x00000204U+((TS_ESTF)*0x20U))
#define CSL_CPTS_TS_ESTF_CONTROL_REG(TS_ESTF)                                  (0x00000208U+((TS_ESTF)*0x20U))
#define CSL_CPTS_TS_ESTF_LENGTH_REG(TS_ESTF)                                   (0x0000020CU+((TS_ESTF)*0x20U))
#define CSL_CPTS_TS_ESTF_PPM_LOW_REG(TS_ESTF)                                  (0x00000210U+((TS_ESTF)*0x20U))
#define CSL_CPTS_TS_ESTF_PPM_HIGH_REG(TS_ESTF)                                 (0x00000214U+((TS_ESTF)*0x20U))
#define CSL_CPTS_TS_ESTF_NUDGE_REG(TS_ESTF)                                    (0x00000218U+((TS_ESTF)*0x20U))

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* COMP_LOW_REG */

#define CSL_CPTS_TS_GENF_COMP_LOW_REG_COMP_LOW_MASK                            (0xFFFFFFFFU)
#define CSL_CPTS_TS_GENF_COMP_LOW_REG_COMP_LOW_SHIFT                           (0x00000000U)
#define CSL_CPTS_TS_GENF_COMP_LOW_REG_COMP_LOW_MAX                             (0xFFFFFFFFU)

/* COMP_HIGH_REG */

#define CSL_CPTS_TS_GENF_COMP_HIGH_REG_COMP_HIGH_MASK                          (0xFFFFFFFFU)
#define CSL_CPTS_TS_GENF_COMP_HIGH_REG_COMP_HIGH_SHIFT                         (0x00000000U)
#define CSL_CPTS_TS_GENF_COMP_HIGH_REG_COMP_HIGH_MAX                           (0xFFFFFFFFU)

/* CONTROL_REG */

#define CSL_CPTS_TS_GENF_CONTROL_REG_PPM_DIR_MASK                              (0x00000001U)
#define CSL_CPTS_TS_GENF_CONTROL_REG_PPM_DIR_SHIFT                             (0x00000000U)
#define CSL_CPTS_TS_GENF_CONTROL_REG_PPM_DIR_MAX                               (0x00000001U)

#define CSL_CPTS_TS_GENF_CONTROL_REG_POLARITY_INV_MASK                         (0x00000002U)
#define CSL_CPTS_TS_GENF_CONTROL_REG_POLARITY_INV_SHIFT                        (0x00000001U)
#define CSL_CPTS_TS_GENF_CONTROL_REG_POLARITY_INV_MAX                          (0x00000001U)

/* LENGTH_REG */

#define CSL_CPTS_TS_GENF_LENGTH_REG_LENGTH_MASK                                (0xFFFFFFFFU)
#define CSL_CPTS_TS_GENF_LENGTH_REG_LENGTH_SHIFT                               (0x00000000U)
#define CSL_CPTS_TS_GENF_LENGTH_REG_LENGTH_MAX                                 (0xFFFFFFFFU)

/* PPM_LOW_REG */

#define CSL_CPTS_TS_GENF_PPM_LOW_REG_PPM_LOW_MASK                              (0xFFFFFFFFU)
#define CSL_CPTS_TS_GENF_PPM_LOW_REG_PPM_LOW_SHIFT                             (0x00000000U)
#define CSL_CPTS_TS_GENF_PPM_LOW_REG_PPM_LOW_MAX                               (0xFFFFFFFFU)

/* PPM_HIGH_REG */

#define CSL_CPTS_TS_GENF_PPM_HIGH_REG_PPM_HIGH_MASK                            (0x000003FFU)
#define CSL_CPTS_TS_GENF_PPM_HIGH_REG_PPM_HIGH_SHIFT                           (0x00000000U)
#define CSL_CPTS_TS_GENF_PPM_HIGH_REG_PPM_HIGH_MAX                             (0x000003FFU)

/* NUDGE_REG */

#define CSL_CPTS_TS_GENF_NUDGE_REG_NUDGE_MASK                                  (0x000000FFU)
#define CSL_CPTS_TS_GENF_NUDGE_REG_NUDGE_SHIFT                                 (0x00000000U)
#define CSL_CPTS_TS_GENF_NUDGE_REG_NUDGE_MAX                                   (0x000000FFU)

/* COMP_LOW_REG */

#define CSL_CPTS_TS_ESTF_COMP_LOW_REG_COMP_LOW_MASK                            (0xFFFFFFFFU)
#define CSL_CPTS_TS_ESTF_COMP_LOW_REG_COMP_LOW_SHIFT                           (0x00000000U)
#define CSL_CPTS_TS_ESTF_COMP_LOW_REG_COMP_LOW_MAX                             (0xFFFFFFFFU)

/* COMP_HIGH_REG */

#define CSL_CPTS_TS_ESTF_COMP_HIGH_REG_COMP_HIGH_MASK                          (0xFFFFFFFFU)
#define CSL_CPTS_TS_ESTF_COMP_HIGH_REG_COMP_HIGH_SHIFT                         (0x00000000U)
#define CSL_CPTS_TS_ESTF_COMP_HIGH_REG_COMP_HIGH_MAX                           (0xFFFFFFFFU)

/* CONTROL_REG */

#define CSL_CPTS_TS_ESTF_CONTROL_REG_PPM_DIR_MASK                              (0x00000001U)
#define CSL_CPTS_TS_ESTF_CONTROL_REG_PPM_DIR_SHIFT                             (0x00000000U)
#define CSL_CPTS_TS_ESTF_CONTROL_REG_PPM_DIR_MAX                               (0x00000001U)

#define CSL_CPTS_TS_ESTF_CONTROL_REG_POLARITY_INV_MASK                         (0x00000002U)
#define CSL_CPTS_TS_ESTF_CONTROL_REG_POLARITY_INV_SHIFT                        (0x00000001U)
#define CSL_CPTS_TS_ESTF_CONTROL_REG_POLARITY_INV_MAX                          (0x00000001U)

/* LENGTH_REG */

#define CSL_CPTS_TS_ESTF_LENGTH_REG_LENGTH_MASK                                (0xFFFFFFFFU)
#define CSL_CPTS_TS_ESTF_LENGTH_REG_LENGTH_SHIFT                               (0x00000000U)
#define CSL_CPTS_TS_ESTF_LENGTH_REG_LENGTH_MAX                                 (0xFFFFFFFFU)

/* PPM_LOW_REG */

#define CSL_CPTS_TS_ESTF_PPM_LOW_REG_PPM_LOW_MASK                              (0xFFFFFFFFU)
#define CSL_CPTS_TS_ESTF_PPM_LOW_REG_PPM_LOW_SHIFT                             (0x00000000U)
#define CSL_CPTS_TS_ESTF_PPM_LOW_REG_PPM_LOW_MAX                               (0xFFFFFFFFU)

/* PPM_HIGH_REG */

#define CSL_CPTS_TS_ESTF_PPM_HIGH_REG_PPM_HIGH_MASK                            (0x000003FFU)
#define CSL_CPTS_TS_ESTF_PPM_HIGH_REG_PPM_HIGH_SHIFT                           (0x00000000U)
#define CSL_CPTS_TS_ESTF_PPM_HIGH_REG_PPM_HIGH_MAX                             (0x000003FFU)

/* NUDGE_REG */

#define CSL_CPTS_TS_ESTF_NUDGE_REG_NUDGE_MASK                                  (0x000000FFU)
#define CSL_CPTS_TS_ESTF_NUDGE_REG_NUDGE_SHIFT                                 (0x00000000U)
#define CSL_CPTS_TS_ESTF_NUDGE_REG_NUDGE_MAX                                   (0x000000FFU)

/* IDVER_REG */

#define CSL_CPTS_IDVER_REG_MINOR_VER_MASK                                      (0x000000FFU)
#define CSL_CPTS_IDVER_REG_MINOR_VER_SHIFT                                     (0x00000000U)
#define CSL_CPTS_IDVER_REG_MINOR_VER_MAX                                       (0x000000FFU)

#define CSL_CPTS_IDVER_REG_MAJOR_VER_MASK                                      (0x00000700U)
#define CSL_CPTS_IDVER_REG_MAJOR_VER_SHIFT                                     (0x00000008U)
#define CSL_CPTS_IDVER_REG_MAJOR_VER_MAX                                       (0x00000007U)

#define CSL_CPTS_IDVER_REG_RTL_VER_MASK                                        (0x0000F800U)
#define CSL_CPTS_IDVER_REG_RTL_VER_SHIFT                                       (0x0000000BU)
#define CSL_CPTS_IDVER_REG_RTL_VER_MAX                                         (0x0000001FU)

#define CSL_CPTS_IDVER_REG_TX_IDENT_MASK                                       (0xFFFF0000U)
#define CSL_CPTS_IDVER_REG_TX_IDENT_SHIFT                                      (0x00000010U)
#define CSL_CPTS_IDVER_REG_TX_IDENT_MAX                                        (0x0000FFFFU)

/* CONTROL_REG */

#define CSL_CPTS_CONTROL_REG_CPTS_EN_MASK                                      (0x00000001U)
#define CSL_CPTS_CONTROL_REG_CPTS_EN_SHIFT                                     (0x00000000U)
#define CSL_CPTS_CONTROL_REG_CPTS_EN_MAX                                       (0x00000001U)

#define CSL_CPTS_CONTROL_REG_INT_TEST_MASK                                     (0x00000002U)
#define CSL_CPTS_CONTROL_REG_INT_TEST_SHIFT                                    (0x00000001U)
#define CSL_CPTS_CONTROL_REG_INT_TEST_MAX                                      (0x00000001U)

#define CSL_CPTS_CONTROL_REG_TS_COMP_POLARITY_MASK                             (0x00000004U)
#define CSL_CPTS_CONTROL_REG_TS_COMP_POLARITY_SHIFT                            (0x00000002U)
#define CSL_CPTS_CONTROL_REG_TS_COMP_POLARITY_MAX                              (0x00000001U)

#define CSL_CPTS_CONTROL_REG_TSTAMP_EN_MASK                                    (0x00000008U)
#define CSL_CPTS_CONTROL_REG_TSTAMP_EN_SHIFT                                   (0x00000003U)
#define CSL_CPTS_CONTROL_REG_TSTAMP_EN_MAX                                     (0x00000001U)

#define CSL_CPTS_CONTROL_REG_SEQUENCE_EN_MASK                                  (0x00000010U)
#define CSL_CPTS_CONTROL_REG_SEQUENCE_EN_SHIFT                                 (0x00000004U)
#define CSL_CPTS_CONTROL_REG_SEQUENCE_EN_MAX                                   (0x00000001U)

#define CSL_CPTS_CONTROL_REG_MODE_MASK                                         (0x00000020U)
#define CSL_CPTS_CONTROL_REG_MODE_SHIFT                                        (0x00000005U)
#define CSL_CPTS_CONTROL_REG_MODE_MAX                                          (0x00000001U)

#define CSL_CPTS_CONTROL_REG_TS_COMP_TOG_MASK                                  (0x00000040U)
#define CSL_CPTS_CONTROL_REG_TS_COMP_TOG_SHIFT                                 (0x00000006U)
#define CSL_CPTS_CONTROL_REG_TS_COMP_TOG_MAX                                   (0x00000001U)

#define CSL_CPTS_CONTROL_REG_TS_PPM_DIR_MASK                                   (0x00000080U)
#define CSL_CPTS_CONTROL_REG_TS_PPM_DIR_SHIFT                                  (0x00000007U)
#define CSL_CPTS_CONTROL_REG_TS_PPM_DIR_MAX                                    (0x00000001U)

#define CSL_CPTS_CONTROL_REG_HW1_TS_PUSH_EN_MASK                               (0x00000100U)
#define CSL_CPTS_CONTROL_REG_HW1_TS_PUSH_EN_SHIFT                              (0x00000008U)
#define CSL_CPTS_CONTROL_REG_HW1_TS_PUSH_EN_MAX                                (0x00000001U)

#define CSL_CPTS_CONTROL_REG_HW2_TS_PUSH_EN_MASK                               (0x00000200U)
#define CSL_CPTS_CONTROL_REG_HW2_TS_PUSH_EN_SHIFT                              (0x00000009U)
#define CSL_CPTS_CONTROL_REG_HW2_TS_PUSH_EN_MAX                                (0x00000001U)

#define CSL_CPTS_CONTROL_REG_HW3_TS_PUSH_EN_MASK                               (0x00000400U)
#define CSL_CPTS_CONTROL_REG_HW3_TS_PUSH_EN_SHIFT                              (0x0000000AU)
#define CSL_CPTS_CONTROL_REG_HW3_TS_PUSH_EN_MAX                                (0x00000001U)

#define CSL_CPTS_CONTROL_REG_HW4_TS_PUSH_EN_MASK                               (0x00000800U)
#define CSL_CPTS_CONTROL_REG_HW4_TS_PUSH_EN_SHIFT                              (0x0000000BU)
#define CSL_CPTS_CONTROL_REG_HW4_TS_PUSH_EN_MAX                                (0x00000001U)

#define CSL_CPTS_CONTROL_REG_HW5_TS_PUSH_EN_MASK                               (0x00001000U)
#define CSL_CPTS_CONTROL_REG_HW5_TS_PUSH_EN_SHIFT                              (0x0000000CU)
#define CSL_CPTS_CONTROL_REG_HW5_TS_PUSH_EN_MAX                                (0x00000001U)

#define CSL_CPTS_CONTROL_REG_HW6_TS_PUSH_EN_MASK                               (0x00002000U)
#define CSL_CPTS_CONTROL_REG_HW6_TS_PUSH_EN_SHIFT                              (0x0000000DU)
#define CSL_CPTS_CONTROL_REG_HW6_TS_PUSH_EN_MAX                                (0x00000001U)

#define CSL_CPTS_CONTROL_REG_HW7_TS_PUSH_EN_MASK                               (0x00004000U)
#define CSL_CPTS_CONTROL_REG_HW7_TS_PUSH_EN_SHIFT                              (0x0000000EU)
#define CSL_CPTS_CONTROL_REG_HW7_TS_PUSH_EN_MAX                                (0x00000001U)

#define CSL_CPTS_CONTROL_REG_HW8_TS_PUSH_EN_MASK                               (0x00008000U)
#define CSL_CPTS_CONTROL_REG_HW8_TS_PUSH_EN_SHIFT                              (0x0000000FU)
#define CSL_CPTS_CONTROL_REG_HW8_TS_PUSH_EN_MAX                                (0x00000001U)

#define CSL_CPTS_CONTROL_REG_TS_RX_NO_EVENT_MASK                               (0x00010000U)
#define CSL_CPTS_CONTROL_REG_TS_RX_NO_EVENT_SHIFT                              (0x00000010U)
#define CSL_CPTS_CONTROL_REG_TS_RX_NO_EVENT_MAX                                (0x00000001U)

#define CSL_CPTS_CONTROL_REG_TS_GENF_CLR_EN_MASK                               (0x00020000U)
#define CSL_CPTS_CONTROL_REG_TS_GENF_CLR_EN_SHIFT                              (0x00000011U)
#define CSL_CPTS_CONTROL_REG_TS_GENF_CLR_EN_MAX                                (0x00000001U)

#define CSL_CPTS_CONTROL_REG_TS_SYNC_SEL_MASK                                  (0xF0000000U)
#define CSL_CPTS_CONTROL_REG_TS_SYNC_SEL_SHIFT                                 (0x0000001CU)
#define CSL_CPTS_CONTROL_REG_TS_SYNC_SEL_MAX                                   (0x0000000FU)

/* RFTCLK_SEL_REG */

#define CSL_CPTS_RFTCLK_SEL_REG_RFTCLK_SEL_MASK                                (0x0000001FU)
#define CSL_CPTS_RFTCLK_SEL_REG_RFTCLK_SEL_SHIFT                               (0x00000000U)
#define CSL_CPTS_RFTCLK_SEL_REG_RFTCLK_SEL_MAX                                 (0x0000001FU)

/* TS_PUSH_REG */

#define CSL_CPTS_TS_PUSH_REG_TS_PUSH_MASK                                      (0x00000001U)
#define CSL_CPTS_TS_PUSH_REG_TS_PUSH_SHIFT                                     (0x00000000U)
#define CSL_CPTS_TS_PUSH_REG_TS_PUSH_MAX                                       (0x00000001U)

/* TS_LOAD_VAL_REG */

#define CSL_CPTS_TS_LOAD_VAL_REG_TS_LOAD_VAL_MASK                              (0xFFFFFFFFU)
#define CSL_CPTS_TS_LOAD_VAL_REG_TS_LOAD_VAL_SHIFT                             (0x00000000U)
#define CSL_CPTS_TS_LOAD_VAL_REG_TS_LOAD_VAL_MAX                               (0xFFFFFFFFU)

/* TS_LOAD_EN_REG */

#define CSL_CPTS_TS_LOAD_EN_REG_TS_LOAD_EN_MASK                                (0x00000001U)
#define CSL_CPTS_TS_LOAD_EN_REG_TS_LOAD_EN_SHIFT                               (0x00000000U)
#define CSL_CPTS_TS_LOAD_EN_REG_TS_LOAD_EN_MAX                                 (0x00000001U)

/* TS_COMP_VAL_REG */

#define CSL_CPTS_TS_COMP_VAL_REG_TS_COMP_VAL_MASK                              (0xFFFFFFFFU)
#define CSL_CPTS_TS_COMP_VAL_REG_TS_COMP_VAL_SHIFT                             (0x00000000U)
#define CSL_CPTS_TS_COMP_VAL_REG_TS_COMP_VAL_MAX                               (0xFFFFFFFFU)

/* TS_COMP_LEN_REG */

#define CSL_CPTS_TS_COMP_LEN_REG_TS_COMP_LENGTH_MASK                           (0xFFFFFFFFU)
#define CSL_CPTS_TS_COMP_LEN_REG_TS_COMP_LENGTH_SHIFT                          (0x00000000U)
#define CSL_CPTS_TS_COMP_LEN_REG_TS_COMP_LENGTH_MAX                            (0xFFFFFFFFU)

/* INTSTAT_RAW_REG */

#define CSL_CPTS_INTSTAT_RAW_REG_TS_PEND_RAW_MASK                              (0x00000001U)
#define CSL_CPTS_INTSTAT_RAW_REG_TS_PEND_RAW_SHIFT                             (0x00000000U)
#define CSL_CPTS_INTSTAT_RAW_REG_TS_PEND_RAW_MAX                               (0x00000001U)

/* INTSTAT_MASKED_REG */

#define CSL_CPTS_INTSTAT_MASKED_REG_TS_PEND_MASK                               (0x00000001U)
#define CSL_CPTS_INTSTAT_MASKED_REG_TS_PEND_SHIFT                              (0x00000000U)
#define CSL_CPTS_INTSTAT_MASKED_REG_TS_PEND_MAX                                (0x00000001U)

/* INT_ENABLE_REG */

#define CSL_CPTS_INT_ENABLE_REG_TS_PEND_EN_MASK                                (0x00000001U)
#define CSL_CPTS_INT_ENABLE_REG_TS_PEND_EN_SHIFT                               (0x00000000U)
#define CSL_CPTS_INT_ENABLE_REG_TS_PEND_EN_MAX                                 (0x00000001U)

/* TS_COMP_NUDGE_REG */

#define CSL_CPTS_TS_COMP_NUDGE_REG_NUDGE_MASK                                  (0x000000FFU)
#define CSL_CPTS_TS_COMP_NUDGE_REG_NUDGE_SHIFT                                 (0x00000000U)
#define CSL_CPTS_TS_COMP_NUDGE_REG_NUDGE_MAX                                   (0x000000FFU)

/* EVENT_POP_REG */

#define CSL_CPTS_EVENT_POP_REG_EVENT_POP_MASK                                  (0x00000001U)
#define CSL_CPTS_EVENT_POP_REG_EVENT_POP_SHIFT                                 (0x00000000U)
#define CSL_CPTS_EVENT_POP_REG_EVENT_POP_MAX                                   (0x00000001U)

/* EVENT_0_REG */

#define CSL_CPTS_EVENT_0_REG_TIME_STAMP_MASK                                   (0xFFFFFFFFU)
#define CSL_CPTS_EVENT_0_REG_TIME_STAMP_SHIFT                                  (0x00000000U)
#define CSL_CPTS_EVENT_0_REG_TIME_STAMP_MAX                                    (0xFFFFFFFFU)

/* EVENT_1_REG */

#define CSL_CPTS_EVENT_1_REG_SEQUENCE_ID_MASK                                  (0x0000FFFFU)
#define CSL_CPTS_EVENT_1_REG_SEQUENCE_ID_SHIFT                                 (0x00000000U)
#define CSL_CPTS_EVENT_1_REG_SEQUENCE_ID_MAX                                   (0x0000FFFFU)

#define CSL_CPTS_EVENT_1_REG_MESSAGE_TYPE_MASK                                 (0x000F0000U)
#define CSL_CPTS_EVENT_1_REG_MESSAGE_TYPE_SHIFT                                (0x00000010U)
#define CSL_CPTS_EVENT_1_REG_MESSAGE_TYPE_MAX                                  (0x0000000FU)

#define CSL_CPTS_EVENT_1_REG_EVENT_TYPE_MASK                                   (0x00F00000U)
#define CSL_CPTS_EVENT_1_REG_EVENT_TYPE_SHIFT                                  (0x00000014U)
#define CSL_CPTS_EVENT_1_REG_EVENT_TYPE_MAX                                    (0x0000000FU)

#define CSL_CPTS_EVENT_1_REG_PORT_NUMBER_MASK                                  (0x1F000000U)
#define CSL_CPTS_EVENT_1_REG_PORT_NUMBER_SHIFT                                 (0x00000018U)
#define CSL_CPTS_EVENT_1_REG_PORT_NUMBER_MAX                                   (0x0000001FU)

#define CSL_CPTS_EVENT_1_REG_PREMPT_QUEUE_MASK                                 (0x20000000U)
#define CSL_CPTS_EVENT_1_REG_PREMPT_QUEUE_SHIFT                                (0x0000001DU)
#define CSL_CPTS_EVENT_1_REG_PREMPT_QUEUE_MAX                                  (0x00000001U)

/* EVENT_2_REG */

#define CSL_CPTS_EVENT_2_REG_DOMAIN_MASK                                       (0x000000FFU)
#define CSL_CPTS_EVENT_2_REG_DOMAIN_SHIFT                                      (0x00000000U)
#define CSL_CPTS_EVENT_2_REG_DOMAIN_MAX                                        (0x000000FFU)

/* EVENT_3_REG */

#define CSL_CPTS_EVENT_3_REG_TIME_STAMP_MASK                                   (0xFFFFFFFFU)
#define CSL_CPTS_EVENT_3_REG_TIME_STAMP_SHIFT                                  (0x00000000U)
#define CSL_CPTS_EVENT_3_REG_TIME_STAMP_MAX                                    (0xFFFFFFFFU)

/* TS_LOAD_HIGH_VAL_REG */

#define CSL_CPTS_TS_LOAD_HIGH_VAL_REG_TS_LOAD_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_CPTS_TS_LOAD_HIGH_VAL_REG_TS_LOAD_VAL_SHIFT                        (0x00000000U)
#define CSL_CPTS_TS_LOAD_HIGH_VAL_REG_TS_LOAD_VAL_MAX                          (0xFFFFFFFFU)

/* TS_COMP_HIGH_VAL_REG */

#define CSL_CPTS_TS_COMP_HIGH_VAL_REG_TS_COMP_HIGH_VAL_MASK                    (0xFFFFFFFFU)
#define CSL_CPTS_TS_COMP_HIGH_VAL_REG_TS_COMP_HIGH_VAL_SHIFT                   (0x00000000U)
#define CSL_CPTS_TS_COMP_HIGH_VAL_REG_TS_COMP_HIGH_VAL_MAX                     (0xFFFFFFFFU)

/* TS_ADD_VAL_REG */

#define CSL_CPTS_TS_ADD_VAL_REG_ADD_VAL_MASK                                   (0x00000007U)
#define CSL_CPTS_TS_ADD_VAL_REG_ADD_VAL_SHIFT                                  (0x00000000U)
#define CSL_CPTS_TS_ADD_VAL_REG_ADD_VAL_MAX                                    (0x00000007U)

/* TS_PPM_LOW_VAL_REG */

#define CSL_CPTS_TS_PPM_LOW_VAL_REG_TS_PPM_LOW_VAL_MASK                        (0xFFFFFFFFU)
#define CSL_CPTS_TS_PPM_LOW_VAL_REG_TS_PPM_LOW_VAL_SHIFT                       (0x00000000U)
#define CSL_CPTS_TS_PPM_LOW_VAL_REG_TS_PPM_LOW_VAL_MAX                         (0xFFFFFFFFU)

/* TS_PPM_HIGH_VAL_REG */

#define CSL_CPTS_TS_PPM_HIGH_VAL_REG_TS_PPM_HIGH_VAL_MASK                      (0x000003FFU)
#define CSL_CPTS_TS_PPM_HIGH_VAL_REG_TS_PPM_HIGH_VAL_SHIFT                     (0x00000000U)
#define CSL_CPTS_TS_PPM_HIGH_VAL_REG_TS_PPM_HIGH_VAL_MAX                       (0x000003FFU)

/* TS_NUDGE_VAL_REG */

#define CSL_CPTS_TS_NUDGE_VAL_REG_TS_NUDGE_VAL_MASK                            (0x000000FFU)
#define CSL_CPTS_TS_NUDGE_VAL_REG_TS_NUDGE_VAL_SHIFT                           (0x00000000U)
#define CSL_CPTS_TS_NUDGE_VAL_REG_TS_NUDGE_VAL_MAX                             (0x000000FFU)

#ifdef __cplusplus
}
#endif
#endif
