/********************************************************************
 * Copyright (C) 2020 Texas Instruments Incorporated.
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
 *  Name        : cslr_stc.h
*/
#ifndef CSLR_STC_H_
#define CSLR_STC_H_

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
    volatile uint32_t STCGCR0;
    volatile uint32_t STCGCR1;
    volatile uint32_t STCTPR;
    volatile uint32_t STC_CADDR;
    volatile uint32_t STCCICR;
    volatile uint32_t STCGSTAT;
    volatile uint32_t STCFSTAT;
    volatile uint32_t STCSCSCR;
    volatile uint32_t STC_CADDR2;
    volatile uint32_t STC_CLKDIV;
    volatile uint32_t STC_SEGPLR;
    volatile uint32_t SEG0_START_ADDR;
    volatile uint32_t SEG1_START_ADDR;
    volatile uint32_t SEG2_START_ADDR;
    volatile uint32_t SEG3_START_ADDR;
    volatile uint32_t CORE1_CURMISR_0;
    volatile uint32_t CORE1_CURMISR_1;
    volatile uint32_t CORE1_CURMISR_2;
    volatile uint32_t CORE1_CURMISR_3;
    volatile uint32_t CORE1_CURMISR_4;
    volatile uint32_t CORE1_CURMISR_5;
    volatile uint32_t CORE1_CURMISR_6;
    volatile uint32_t CORE1_CURMISR_7;
    volatile uint32_t CORE1_CURMISR_8;
    volatile uint32_t CORE1_CURMISR_9;
    volatile uint32_t CORE1_CURMISR_10;
    volatile uint32_t CORE1_CURMISR_11;
    volatile uint32_t CORE1_CURMISR_12;
    volatile uint32_t CORE1_CURMISR_13;
    volatile uint32_t CORE1_CURMISR_14;
    volatile uint32_t CORE1_CURMISR_15;
    volatile uint32_t CORE1_CURMISR_16;
    volatile uint32_t CORE1_CURMISR_17;
    volatile uint32_t CORE1_CURMISR_18;
    volatile uint32_t CORE1_CURMISR_19;
    volatile uint32_t CORE1_CURMISR_20;
    volatile uint32_t CORE1_CURMISR_21;
    volatile uint32_t CORE1_CURMISR_22;
    volatile uint32_t CORE1_CURMISR_23;
    volatile uint32_t CORE1_CURMISR_24;
    volatile uint32_t CORE1_CURMISR_25;
    volatile uint32_t CORE1_CURMISR_26;
    volatile uint32_t CORE1_CURMISR_27;
    volatile uint32_t CORE2_CURMISR_0;
    volatile uint32_t CORE2_CURMISR_1;
    volatile uint32_t CORE2_CURMISR_2;
    volatile uint32_t CORE2_CURMISR_3;
    volatile uint32_t CORE2_CURMISR_4;
    volatile uint32_t CORE2_CURMISR_5;
    volatile uint32_t CORE2_CURMISR_6;
    volatile uint32_t CORE2_CURMISR_7;
    volatile uint32_t CORE2_CURMISR_8;
    volatile uint32_t CORE2_CURMISR_9;
    volatile uint32_t CORE2_CURMISR_10;
    volatile uint32_t CORE2_CURMISR_11;
    volatile uint32_t CORE2_CURMISR_12;
    volatile uint32_t CORE2_CURMISR_13;
    volatile uint32_t CORE2_CURMISR_14;
    volatile uint32_t CORE2_CURMISR_15;
    volatile uint32_t CORE2_CURMISR_16;
    volatile uint32_t CORE2_CURMISR_17;
    volatile uint32_t CORE2_CURMISR_18;
    volatile uint32_t CORE2_CURMISR_19;
    volatile uint32_t CORE2_CURMISR_20;
    volatile uint32_t CORE2_CURMISR_21;
    volatile uint32_t CORE2_CURMISR_22;
    volatile uint32_t CORE2_CURMISR_23;
    volatile uint32_t CORE2_CURMISR_24;
    volatile uint32_t CORE2_CURMISR_25;
    volatile uint32_t CORE2_CURMISR_26;
    volatile uint32_t CORE2_CURMISR_27;
} CSL_stcRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_STC_STCGCR0                                                        (0x00000000U)
#define CSL_STC_STCGCR1                                                        (0x00000004U)
#define CSL_STC_STCTPR                                                         (0x00000008U)
#define CSL_STC_STC_CADDR                                                      (0x0000000CU)
#define CSL_STC_STCCICR                                                        (0x00000010U)
#define CSL_STC_STCGSTAT                                                       (0x00000014U)
#define CSL_STC_STCFSTAT                                                       (0x00000018U)
#define CSL_STC_STCSCSCR                                                       (0x0000001CU)
#define CSL_STC_STC_CADDR2                                                     (0x00000020U)
#define CSL_STC_STC_CLKDIV                                                     (0x00000024U)
#define CSL_STC_STC_SEGPLR                                                     (0x00000028U)
#define CSL_STC_SEG0_START_ADDR                                                (0x0000002CU)
#define CSL_STC_SEG1_START_ADDR                                                (0x00000030U)
#define CSL_STC_SEG2_START_ADDR                                                (0x00000034U)
#define CSL_STC_SEG3_START_ADDR                                                (0x00000038U)
#define CSL_STC_CORE1_CURMISR_0                                                (0x0000003CU)
#define CSL_STC_CORE1_CURMISR_1                                                (0x00000040U)
#define CSL_STC_CORE1_CURMISR_2                                                (0x00000044U)
#define CSL_STC_CORE1_CURMISR_3                                                (0x00000048U)
#define CSL_STC_CORE1_CURMISR_4                                                (0x0000004CU)
#define CSL_STC_CORE1_CURMISR_5                                                (0x00000050U)
#define CSL_STC_CORE1_CURMISR_6                                                (0x00000054U)
#define CSL_STC_CORE1_CURMISR_7                                                (0x00000058U)
#define CSL_STC_CORE1_CURMISR_8                                                (0x0000005CU)
#define CSL_STC_CORE1_CURMISR_9                                                (0x00000060U)
#define CSL_STC_CORE1_CURMISR_10                                               (0x00000064U)
#define CSL_STC_CORE1_CURMISR_11                                               (0x00000068U)
#define CSL_STC_CORE1_CURMISR_12                                               (0x0000006CU)
#define CSL_STC_CORE1_CURMISR_13                                               (0x00000070U)
#define CSL_STC_CORE1_CURMISR_14                                               (0x00000074U)
#define CSL_STC_CORE1_CURMISR_15                                               (0x00000078U)
#define CSL_STC_CORE1_CURMISR_16                                               (0x0000007CU)
#define CSL_STC_CORE1_CURMISR_17                                               (0x00000080U)
#define CSL_STC_CORE1_CURMISR_18                                               (0x00000084U)
#define CSL_STC_CORE1_CURMISR_19                                               (0x00000088U)
#define CSL_STC_CORE1_CURMISR_20                                               (0x0000008CU)
#define CSL_STC_CORE1_CURMISR_21                                               (0x00000090U)
#define CSL_STC_CORE1_CURMISR_22                                               (0x00000094U)
#define CSL_STC_CORE1_CURMISR_23                                               (0x00000098U)
#define CSL_STC_CORE1_CURMISR_24                                               (0x0000009CU)
#define CSL_STC_CORE1_CURMISR_25                                               (0x000000A0U)
#define CSL_STC_CORE1_CURMISR_26                                               (0x000000A4U)
#define CSL_STC_CORE1_CURMISR_27                                               (0x000000A8U)
#define CSL_STC_CORE2_CURMISR_0                                                (0x000000ACU)
#define CSL_STC_CORE2_CURMISR_1                                                (0x000000B0U)
#define CSL_STC_CORE2_CURMISR_2                                                (0x000000B4U)
#define CSL_STC_CORE2_CURMISR_3                                                (0x000000B8U)
#define CSL_STC_CORE2_CURMISR_4                                                (0x000000BCU)
#define CSL_STC_CORE2_CURMISR_5                                                (0x000000C0U)
#define CSL_STC_CORE2_CURMISR_6                                                (0x000000C4U)
#define CSL_STC_CORE2_CURMISR_7                                                (0x000000C8U)
#define CSL_STC_CORE2_CURMISR_8                                                (0x000000CCU)
#define CSL_STC_CORE2_CURMISR_9                                                (0x000000D0U)
#define CSL_STC_CORE2_CURMISR_10                                               (0x000000D4U)
#define CSL_STC_CORE2_CURMISR_11                                               (0x000000D8U)
#define CSL_STC_CORE2_CURMISR_12                                               (0x000000DCU)
#define CSL_STC_CORE2_CURMISR_13                                               (0x000000E0U)
#define CSL_STC_CORE2_CURMISR_14                                               (0x000000E4U)
#define CSL_STC_CORE2_CURMISR_15                                               (0x000000E8U)
#define CSL_STC_CORE2_CURMISR_16                                               (0x000000ECU)
#define CSL_STC_CORE2_CURMISR_17                                               (0x000000F0U)
#define CSL_STC_CORE2_CURMISR_18                                               (0x000000F4U)
#define CSL_STC_CORE2_CURMISR_19                                               (0x000000F8U)
#define CSL_STC_CORE2_CURMISR_20                                               (0x000000FCU)
#define CSL_STC_CORE2_CURMISR_21                                               (0x00000100U)
#define CSL_STC_CORE2_CURMISR_22                                               (0x00000104U)
#define CSL_STC_CORE2_CURMISR_23                                               (0x00000108U)
#define CSL_STC_CORE2_CURMISR_24                                               (0x0000010CU)
#define CSL_STC_CORE2_CURMISR_25                                               (0x00000110U)
#define CSL_STC_CORE2_CURMISR_26                                               (0x00000114U)
#define CSL_STC_CORE2_CURMISR_27                                               (0x00000118U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* STCGCR0 */

#define CSL_STC_STCGCR0_RS_CNT_B1_MASK                                         (0x00000003U)
#define CSL_STC_STCGCR0_RS_CNT_B1_SHIFT                                        (0x00000000U)
#define CSL_STC_STCGCR0_RS_CNT_B1_RESETVAL                                     (0x00000000U)
#define CSL_STC_STCGCR0_RS_CNT_B1_MAX                                          (0x00000003U)

#define CSL_STC_STCGCR0_NU1_MASK                                               (0x0000001CU)
#define CSL_STC_STCGCR0_NU1_SHIFT                                              (0x00000002U)
#define CSL_STC_STCGCR0_NU1_RESETVAL                                           (0x00000000U)
#define CSL_STC_STCGCR0_NU1_MAX                                                (0x00000007U)

#define CSL_STC_STCGCR0_SCANEN_HIGH_CAP_IDLE_CYCLE_MASK                        (0x000000E0U)
#define CSL_STC_STCGCR0_SCANEN_HIGH_CAP_IDLE_CYCLE_SHIFT                       (0x00000005U)
#define CSL_STC_STCGCR0_SCANEN_HIGH_CAP_IDLE_CYCLE_RESETVAL                    (0x00000001U)
#define CSL_STC_STCGCR0_SCANEN_HIGH_CAP_IDLE_CYCLE_MAX                         (0x00000007U)

#define CSL_STC_STCGCR0_CAP_IDLE_CYCLE_MASK                                    (0x00000700U)
#define CSL_STC_STCGCR0_CAP_IDLE_CYCLE_SHIFT                                   (0x00000008U)
#define CSL_STC_STCGCR0_CAP_IDLE_CYCLE_RESETVAL                                (0x00000001U)
#define CSL_STC_STCGCR0_CAP_IDLE_CYCLE_MAX                                     (0x00000007U)

#define CSL_STC_STCGCR0_NU0_MASK                                               (0x0000F800U)
#define CSL_STC_STCGCR0_NU0_SHIFT                                              (0x0000000BU)
#define CSL_STC_STCGCR0_NU0_RESETVAL                                           (0x00000000U)
#define CSL_STC_STCGCR0_NU0_MAX                                                (0x0000001FU)

#define CSL_STC_STCGCR0_INTCOUNT_B16_MASK                                      (0xFFFF0000U)
#define CSL_STC_STCGCR0_INTCOUNT_B16_SHIFT                                     (0x00000010U)
#define CSL_STC_STCGCR0_INTCOUNT_B16_RESETVAL                                  (0x00000001U)
#define CSL_STC_STCGCR0_INTCOUNT_B16_MAX                                       (0x0000FFFFU)

#define CSL_STC_STCGCR0_RESETVAL                                               (0x00010120U)

/* STCGCR1 */

#define CSL_STC_STCGCR1_ST_ENA_B4_MASK                                         (0x0000000FU)
#define CSL_STC_STCGCR1_ST_ENA_B4_SHIFT                                        (0x00000000U)
#define CSL_STC_STCGCR1_ST_ENA_B4_RESETVAL                                     (0x00000005U)
#define CSL_STC_STCGCR1_ST_ENA_B4_MAX                                          (0x0000000FU)

#define CSL_STC_STCGCR1_ROM_ACCESS_INV_MASK                                    (0x00000010U)
#define CSL_STC_STCGCR1_ROM_ACCESS_INV_SHIFT                                   (0x00000004U)
#define CSL_STC_STCGCR1_ROM_ACCESS_INV_RESETVAL                                (0x00000000U)
#define CSL_STC_STCGCR1_ROM_ACCESS_INV_MAX                                     (0x00000001U)

#define CSL_STC_STCGCR1_LP_SCAN_MODE_MASK                                      (0x00000020U)
#define CSL_STC_STCGCR1_LP_SCAN_MODE_SHIFT                                     (0x00000005U)
#define CSL_STC_STCGCR1_LP_SCAN_MODE_RESETVAL                                  (0x00000001U)
#define CSL_STC_STCGCR1_LP_SCAN_MODE_MAX                                       (0x00000001U)

#define CSL_STC_STCGCR1_CODEC_SPREAD_MODE_MASK                                 (0x00000040U)
#define CSL_STC_STCGCR1_CODEC_SPREAD_MODE_SHIFT                                (0x00000006U)
#define CSL_STC_STCGCR1_CODEC_SPREAD_MODE_RESETVAL                             (0x00000000U)
#define CSL_STC_STCGCR1_CODEC_SPREAD_MODE_MAX                                  (0x00000001U)

#define CSL_STC_STCGCR1_NU3_MASK                                               (0x00000080U)
#define CSL_STC_STCGCR1_NU3_SHIFT                                              (0x00000007U)
#define CSL_STC_STCGCR1_NU3_RESETVAL                                           (0x00000000U)
#define CSL_STC_STCGCR1_NU3_MAX                                                (0x00000001U)

#define CSL_STC_STCGCR1_SEG0_CORE_SEL_MASK                                     (0x00000F00U)
#define CSL_STC_STCGCR1_SEG0_CORE_SEL_SHIFT                                    (0x00000008U)
#define CSL_STC_STCGCR1_SEG0_CORE_SEL_RESETVAL                                 (0x00000000U)
#define CSL_STC_STCGCR1_SEG0_CORE_SEL_MAX                                      (0x0000000FU)

#define CSL_STC_STCGCR1_NU2_MASK                                               (0xFFFFF000U)
#define CSL_STC_STCGCR1_NU2_SHIFT                                              (0x0000000CU)
#define CSL_STC_STCGCR1_NU2_RESETVAL                                           (0x00000000U)
#define CSL_STC_STCGCR1_NU2_MAX                                                (0x000FFFFFU)

#define CSL_STC_STCGCR1_RESETVAL                                               (0x00000025U)

/* STCTPR */

#define CSL_STC_STCTPR_TO_PRELOAD_MASK                                         (0xFFFFFFFFU)
#define CSL_STC_STCTPR_TO_PRELOAD_SHIFT                                        (0x00000000U)
#define CSL_STC_STCTPR_TO_PRELOAD_RESETVAL                                     (0xFFFFFFFFU)
#define CSL_STC_STCTPR_TO_PRELOAD_MAX                                          (0xFFFFFFFFU)

#define CSL_STC_STCTPR_RESETVAL                                                (0xFFFFFFFFU)

/* STC_CADDR */

#define CSL_STC_STC_CADDR_ADDR_MASK                                            (0xFFFFFFFFU)
#define CSL_STC_STC_CADDR_ADDR_SHIFT                                           (0x00000000U)
#define CSL_STC_STC_CADDR_ADDR_RESETVAL                                        (0x00000000U)
#define CSL_STC_STC_CADDR_ADDR_MAX                                             (0xFFFFFFFFU)

#define CSL_STC_STC_CADDR_RESETVAL                                             (0x00000000U)

/* STCCICR */

#define CSL_STC_STCCICR_CORE1_ICOUNT_MASK                                      (0x0000FFFFU)
#define CSL_STC_STCCICR_CORE1_ICOUNT_SHIFT                                     (0x00000000U)
#define CSL_STC_STCCICR_CORE1_ICOUNT_RESETVAL                                  (0x00000000U)
#define CSL_STC_STCCICR_CORE1_ICOUNT_MAX                                       (0x0000FFFFU)

#define CSL_STC_STCCICR_CORE2_ICOUNT_MASK                                      (0xFFFF0000U)
#define CSL_STC_STCCICR_CORE2_ICOUNT_SHIFT                                     (0x00000010U)
#define CSL_STC_STCCICR_CORE2_ICOUNT_RESETVAL                                  (0x00000000U)
#define CSL_STC_STCCICR_CORE2_ICOUNT_MAX                                       (0x0000FFFFU)

#define CSL_STC_STCCICR_RESETVAL                                               (0x00000000U)

/* STCGSTAT */

#define CSL_STC_STCGSTAT_TEST_DONE_MASK                                        (0x00000001U)
#define CSL_STC_STCGSTAT_TEST_DONE_SHIFT                                       (0x00000000U)
#define CSL_STC_STCGSTAT_TEST_DONE_RESETVAL                                    (0x00000000U)
#define CSL_STC_STCGSTAT_TEST_DONE_MAX                                         (0x00000001U)

#define CSL_STC_STCGSTAT_TEST_FAIL_MASK                                        (0x00000002U)
#define CSL_STC_STCGSTAT_TEST_FAIL_SHIFT                                       (0x00000001U)
#define CSL_STC_STCGSTAT_TEST_FAIL_RESETVAL                                    (0x00000000U)
#define CSL_STC_STCGSTAT_TEST_FAIL_MAX                                         (0x00000001U)

#define CSL_STC_STCGSTAT_NU5_MASK                                              (0x000000FCU)
#define CSL_STC_STCGSTAT_NU5_SHIFT                                             (0x00000002U)
#define CSL_STC_STCGSTAT_NU5_RESETVAL                                          (0x00000000U)
#define CSL_STC_STCGSTAT_NU5_MAX                                               (0x0000003FU)

#define CSL_STC_STCGSTAT_ST_ACTIVE_MASK                                        (0x00000F00U)
#define CSL_STC_STCGSTAT_ST_ACTIVE_SHIFT                                       (0x00000008U)
#define CSL_STC_STCGSTAT_ST_ACTIVE_RESETVAL                                    (0x00000005U)
#define CSL_STC_STCGSTAT_ST_ACTIVE_MAX                                         (0x0000000FU)

#define CSL_STC_STCGSTAT_NU4_MASK                                              (0xFFFFF000U)
#define CSL_STC_STCGSTAT_NU4_SHIFT                                             (0x0000000CU)
#define CSL_STC_STCGSTAT_NU4_RESETVAL                                          (0x00000000U)
#define CSL_STC_STCGSTAT_NU4_MAX                                               (0x000FFFFFU)

#define CSL_STC_STCGSTAT_RESETVAL                                              (0x00000500U)

/* STCFSTAT */

#define CSL_STC_STCFSTAT_CPU1_FAIL_B1_MASK                                     (0x00000001U)
#define CSL_STC_STCFSTAT_CPU1_FAIL_B1_SHIFT                                    (0x00000000U)
#define CSL_STC_STCFSTAT_CPU1_FAIL_B1_RESETVAL                                 (0x00000000U)
#define CSL_STC_STCFSTAT_CPU1_FAIL_B1_MAX                                      (0x00000001U)

#define CSL_STC_STCFSTAT_CPU2_FAIL_B1_MASK                                     (0x00000002U)
#define CSL_STC_STCFSTAT_CPU2_FAIL_B1_SHIFT                                    (0x00000001U)
#define CSL_STC_STCFSTAT_CPU2_FAIL_B1_RESETVAL                                 (0x00000000U)
#define CSL_STC_STCFSTAT_CPU2_FAIL_B1_MAX                                      (0x00000001U)

#define CSL_STC_STCFSTAT_TO_ER_B1_MASK                                         (0x00000004U)
#define CSL_STC_STCFSTAT_TO_ER_B1_SHIFT                                        (0x00000002U)
#define CSL_STC_STCFSTAT_TO_ER_B1_RESETVAL                                     (0x00000000U)
#define CSL_STC_STCFSTAT_TO_ER_B1_MAX                                          (0x00000001U)

#define CSL_STC_STCFSTAT_FSEG_ID_MASK                                          (0x00000018U)
#define CSL_STC_STCFSTAT_FSEG_ID_SHIFT                                         (0x00000003U)
#define CSL_STC_STCFSTAT_FSEG_ID_RESETVAL                                      (0x00000000U)
#define CSL_STC_STCFSTAT_FSEG_ID_MAX                                           (0x00000003U)

#define CSL_STC_STCFSTAT_NU6_MASK                                              (0xFFFFFFE0U)
#define CSL_STC_STCFSTAT_NU6_SHIFT                                             (0x00000005U)
#define CSL_STC_STCFSTAT_NU6_RESETVAL                                          (0x00000000U)
#define CSL_STC_STCFSTAT_NU6_MAX                                               (0x07FFFFFFU)

#define CSL_STC_STCFSTAT_RESETVAL                                              (0x00000000U)

/* STCSCSCR */

#define CSL_STC_STCSCSCR_SELF_CHECK_KEY_B4_MASK                                (0x0000000FU)
#define CSL_STC_STCSCSCR_SELF_CHECK_KEY_B4_SHIFT                               (0x00000000U)
#define CSL_STC_STCSCSCR_SELF_CHECK_KEY_B4_RESETVAL                            (0x00000005U)
#define CSL_STC_STCSCSCR_SELF_CHECK_KEY_B4_MAX                                 (0x0000000FU)

#define CSL_STC_STCSCSCR_FAULT_INS_B1_MASK                                     (0x00000010U)
#define CSL_STC_STCSCSCR_FAULT_INS_B1_SHIFT                                    (0x00000004U)
#define CSL_STC_STCSCSCR_FAULT_INS_B1_RESETVAL                                 (0x00000000U)
#define CSL_STC_STCSCSCR_FAULT_INS_B1_MAX                                      (0x00000001U)

#define CSL_STC_STCSCSCR_NU7_MASK                                              (0xFFFFFFE0U)
#define CSL_STC_STCSCSCR_NU7_SHIFT                                             (0x00000005U)
#define CSL_STC_STCSCSCR_NU7_RESETVAL                                          (0x00000000U)
#define CSL_STC_STCSCSCR_NU7_MAX                                               (0x07FFFFFFU)

#define CSL_STC_STCSCSCR_RESETVAL                                              (0x00000005U)

/* STC_CADDR2 */

#define CSL_STC_STC_CADDR2_ADDR_MASK                                           (0xFFFFFFFFU)
#define CSL_STC_STC_CADDR2_ADDR_SHIFT                                          (0x00000000U)
#define CSL_STC_STC_CADDR2_ADDR_RESETVAL                                       (0x00000000U)
#define CSL_STC_STC_CADDR2_ADDR_MAX                                            (0xFFFFFFFFU)

#define CSL_STC_STC_CADDR2_RESETVAL                                            (0x00000000U)

/* STC_CLKDIV */

#define CSL_STC_STC_CLKDIV_CLKDIV3_MASK                                        (0x00000007U)
#define CSL_STC_STC_CLKDIV_CLKDIV3_SHIFT                                       (0x00000000U)
#define CSL_STC_STC_CLKDIV_CLKDIV3_RESETVAL                                    (0x00000000U)
#define CSL_STC_STC_CLKDIV_CLKDIV3_MAX                                         (0x00000007U)

#define CSL_STC_STC_CLKDIV_NU11_MASK                                           (0x000000F8U)
#define CSL_STC_STC_CLKDIV_NU11_SHIFT                                          (0x00000003U)
#define CSL_STC_STC_CLKDIV_NU11_RESETVAL                                       (0x00000000U)
#define CSL_STC_STC_CLKDIV_NU11_MAX                                            (0x0000001FU)

#define CSL_STC_STC_CLKDIV_CLKDIV2_MASK                                        (0x00000700U)
#define CSL_STC_STC_CLKDIV_CLKDIV2_SHIFT                                       (0x00000008U)
#define CSL_STC_STC_CLKDIV_CLKDIV2_RESETVAL                                    (0x00000000U)
#define CSL_STC_STC_CLKDIV_CLKDIV2_MAX                                         (0x00000007U)

#define CSL_STC_STC_CLKDIV_NU10_MASK                                           (0x0000F800U)
#define CSL_STC_STC_CLKDIV_NU10_SHIFT                                          (0x0000000BU)
#define CSL_STC_STC_CLKDIV_NU10_RESETVAL                                       (0x00000000U)
#define CSL_STC_STC_CLKDIV_NU10_MAX                                            (0x0000001FU)

#define CSL_STC_STC_CLKDIV_CLKDIV1_MASK                                        (0x00070000U)
#define CSL_STC_STC_CLKDIV_CLKDIV1_SHIFT                                       (0x00000010U)
#define CSL_STC_STC_CLKDIV_CLKDIV1_RESETVAL                                    (0x00000000U)
#define CSL_STC_STC_CLKDIV_CLKDIV1_MAX                                         (0x00000007U)

#define CSL_STC_STC_CLKDIV_NU9_MASK                                            (0x00F80000U)
#define CSL_STC_STC_CLKDIV_NU9_SHIFT                                           (0x00000013U)
#define CSL_STC_STC_CLKDIV_NU9_RESETVAL                                        (0x00000000U)
#define CSL_STC_STC_CLKDIV_NU9_MAX                                             (0x0000001FU)

#define CSL_STC_STC_CLKDIV_CLKDIV0_MASK                                        (0x07000000U)
#define CSL_STC_STC_CLKDIV_CLKDIV0_SHIFT                                       (0x00000018U)
#define CSL_STC_STC_CLKDIV_CLKDIV0_RESETVAL                                    (0x00000000U)
#define CSL_STC_STC_CLKDIV_CLKDIV0_MAX                                         (0x00000007U)

#define CSL_STC_STC_CLKDIV_NU8_MASK                                            (0xF8000000U)
#define CSL_STC_STC_CLKDIV_NU8_SHIFT                                           (0x0000001BU)
#define CSL_STC_STC_CLKDIV_NU8_RESETVAL                                        (0x00000000U)
#define CSL_STC_STC_CLKDIV_NU8_MAX                                             (0x0000001FU)

#define CSL_STC_STC_CLKDIV_RESETVAL                                            (0x00000000U)

/* STC_SEGPLR */

#define CSL_STC_STC_SEGPLR_SEGID_PLOAD_MASK                                    (0x00000003U)
#define CSL_STC_STC_SEGPLR_SEGID_PLOAD_SHIFT                                   (0x00000000U)
#define CSL_STC_STC_SEGPLR_SEGID_PLOAD_RESETVAL                                (0x00000000U)
#define CSL_STC_STC_SEGPLR_SEGID_PLOAD_MAX                                     (0x00000003U)

#define CSL_STC_STC_SEGPLR_NU12_MASK                                           (0xFFFFFFFCU)
#define CSL_STC_STC_SEGPLR_NU12_SHIFT                                          (0x00000002U)
#define CSL_STC_STC_SEGPLR_NU12_RESETVAL                                       (0x00000000U)
#define CSL_STC_STC_SEGPLR_NU12_MAX                                            (0x3FFFFFFFU)

#define CSL_STC_STC_SEGPLR_RESETVAL                                            (0x00000000U)

/* SEG0_START_ADDR */

#define CSL_STC_SEG0_START_ADDR_SEG_START_ADDR_MASK                            (0x000FFFFFU)
#define CSL_STC_SEG0_START_ADDR_SEG_START_ADDR_SHIFT                           (0x00000000U)
#define CSL_STC_SEG0_START_ADDR_SEG_START_ADDR_RESETVAL                        (0x00000000U)
#define CSL_STC_SEG0_START_ADDR_SEG_START_ADDR_MAX                             (0x000FFFFFU)

#define CSL_STC_SEG0_START_ADDR_NU13_MASK                                      (0xFFF00000U)
#define CSL_STC_SEG0_START_ADDR_NU13_SHIFT                                     (0x00000014U)
#define CSL_STC_SEG0_START_ADDR_NU13_RESETVAL                                  (0x00000000U)
#define CSL_STC_SEG0_START_ADDR_NU13_MAX                                       (0x00000FFFU)

#define CSL_STC_SEG0_START_ADDR_RESETVAL                                       (0x00000000U)

/* SEG1_START_ADDR */

#define CSL_STC_SEG1_START_ADDR_SEG_START_ADDR_MASK                            (0x000FFFFFU)
#define CSL_STC_SEG1_START_ADDR_SEG_START_ADDR_SHIFT                           (0x00000000U)
#define CSL_STC_SEG1_START_ADDR_SEG_START_ADDR_RESETVAL                        (0x00000000U)
#define CSL_STC_SEG1_START_ADDR_SEG_START_ADDR_MAX                             (0x000FFFFFU)

#define CSL_STC_SEG1_START_ADDR_NU14_MASK                                      (0xFFF00000U)
#define CSL_STC_SEG1_START_ADDR_NU14_SHIFT                                     (0x00000014U)
#define CSL_STC_SEG1_START_ADDR_NU14_RESETVAL                                  (0x00000000U)
#define CSL_STC_SEG1_START_ADDR_NU14_MAX                                       (0x00000FFFU)

#define CSL_STC_SEG1_START_ADDR_RESETVAL                                       (0x00000000U)

/* SEG2_START_ADDR */

#define CSL_STC_SEG2_START_ADDR_SEG_START_ADDR_MASK                            (0x000FFFFFU)
#define CSL_STC_SEG2_START_ADDR_SEG_START_ADDR_SHIFT                           (0x00000000U)
#define CSL_STC_SEG2_START_ADDR_SEG_START_ADDR_RESETVAL                        (0x00000000U)
#define CSL_STC_SEG2_START_ADDR_SEG_START_ADDR_MAX                             (0x000FFFFFU)

#define CSL_STC_SEG2_START_ADDR_NU15_MASK                                      (0xFFF00000U)
#define CSL_STC_SEG2_START_ADDR_NU15_SHIFT                                     (0x00000014U)
#define CSL_STC_SEG2_START_ADDR_NU15_RESETVAL                                  (0x00000000U)
#define CSL_STC_SEG2_START_ADDR_NU15_MAX                                       (0x00000FFFU)

#define CSL_STC_SEG2_START_ADDR_RESETVAL                                       (0x00000000U)

/* SEG3_START_ADDR */

#define CSL_STC_SEG3_START_ADDR_SEG_START_ADDR_MASK                            (0x000FFFFFU)
#define CSL_STC_SEG3_START_ADDR_SEG_START_ADDR_SHIFT                           (0x00000000U)
#define CSL_STC_SEG3_START_ADDR_SEG_START_ADDR_RESETVAL                        (0x00000000U)
#define CSL_STC_SEG3_START_ADDR_SEG_START_ADDR_MAX                             (0x000FFFFFU)

#define CSL_STC_SEG3_START_ADDR_NU16_MASK                                      (0xFFF00000U)
#define CSL_STC_SEG3_START_ADDR_NU16_SHIFT                                     (0x00000014U)
#define CSL_STC_SEG3_START_ADDR_NU16_RESETVAL                                  (0x00000000U)
#define CSL_STC_SEG3_START_ADDR_NU16_MAX                                       (0x00000FFFU)

#define CSL_STC_SEG3_START_ADDR_RESETVAL                                       (0x00000000U)

/* CORE1_CURMISR_0 */

#define CSL_STC_CORE1_CURMISR_0_C1MISR0_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_0_C1MISR0_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE1_CURMISR_0_C1MISR0_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE1_CURMISR_0_C1MISR0_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_0_RESETVAL                                       (0x00000000U)

/* CORE1_CURMISR_1 */

#define CSL_STC_CORE1_CURMISR_1_C1MISR1_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_1_C1MISR1_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE1_CURMISR_1_C1MISR1_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE1_CURMISR_1_C1MISR1_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_1_RESETVAL                                       (0x00000000U)

/* CORE1_CURMISR_2 */

#define CSL_STC_CORE1_CURMISR_2_C1MISR2_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_2_C1MISR2_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE1_CURMISR_2_C1MISR2_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE1_CURMISR_2_C1MISR2_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_2_RESETVAL                                       (0x00000000U)

/* CORE1_CURMISR_3 */

#define CSL_STC_CORE1_CURMISR_3_C1MISR3_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_3_C1MISR3_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE1_CURMISR_3_C1MISR3_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE1_CURMISR_3_C1MISR3_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_3_RESETVAL                                       (0x00000000U)

/* CORE1_CURMISR_4 */

#define CSL_STC_CORE1_CURMISR_4_C1MISR4_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_4_C1MISR4_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE1_CURMISR_4_C1MISR4_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE1_CURMISR_4_C1MISR4_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_4_RESETVAL                                       (0x00000000U)

/* CORE1_CURMISR_5 */

#define CSL_STC_CORE1_CURMISR_5_C1MISR5_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_5_C1MISR5_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE1_CURMISR_5_C1MISR5_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE1_CURMISR_5_C1MISR5_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_5_RESETVAL                                       (0x00000000U)

/* CORE1_CURMISR_6 */

#define CSL_STC_CORE1_CURMISR_6_C1MISR6_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_6_C1MISR6_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE1_CURMISR_6_C1MISR6_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE1_CURMISR_6_C1MISR6_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_6_RESETVAL                                       (0x00000000U)

/* CORE1_CURMISR_7 */

#define CSL_STC_CORE1_CURMISR_7_C1MISR7_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_7_C1MISR7_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE1_CURMISR_7_C1MISR7_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE1_CURMISR_7_C1MISR7_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_7_RESETVAL                                       (0x00000000U)

/* CORE1_CURMISR_8 */

#define CSL_STC_CORE1_CURMISR_8_C1MISR8_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_8_C1MISR8_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE1_CURMISR_8_C1MISR8_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE1_CURMISR_8_C1MISR8_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_8_RESETVAL                                       (0x00000000U)

/* CORE1_CURMISR_9 */

#define CSL_STC_CORE1_CURMISR_9_C1MISR9_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_9_C1MISR9_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE1_CURMISR_9_C1MISR9_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE1_CURMISR_9_C1MISR9_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_9_RESETVAL                                       (0x00000000U)

/* CORE1_CURMISR_10 */

#define CSL_STC_CORE1_CURMISR_10_C1MISR10_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_10_C1MISR10_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_10_C1MISR10_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_10_C1MISR10_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_10_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_11 */

#define CSL_STC_CORE1_CURMISR_11_C1MISR11_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_11_C1MISR11_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_11_C1MISR11_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_11_C1MISR11_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_11_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_12 */

#define CSL_STC_CORE1_CURMISR_12_C1MISR12_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_12_C1MISR12_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_12_C1MISR12_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_12_C1MISR12_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_12_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_13 */

#define CSL_STC_CORE1_CURMISR_13_C1MISR13_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_13_C1MISR13_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_13_C1MISR13_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_13_C1MISR13_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_13_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_14 */

#define CSL_STC_CORE1_CURMISR_14_C1MISR14_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_14_C1MISR14_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_14_C1MISR14_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_14_C1MISR14_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_14_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_15 */

#define CSL_STC_CORE1_CURMISR_15_C1MISR15_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_15_C1MISR15_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_15_C1MISR15_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_15_C1MISR15_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_15_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_16 */

#define CSL_STC_CORE1_CURMISR_16_C1MISR16_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_16_C1MISR16_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_16_C1MISR16_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_16_C1MISR16_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_16_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_17 */

#define CSL_STC_CORE1_CURMISR_17_C1MISR17_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_17_C1MISR17_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_17_C1MISR17_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_17_C1MISR17_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_17_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_18 */

#define CSL_STC_CORE1_CURMISR_18_C1MISR18_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_18_C1MISR18_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_18_C1MISR18_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_18_C1MISR18_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_18_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_19 */

#define CSL_STC_CORE1_CURMISR_19_C1MISR19_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_19_C1MISR19_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_19_C1MISR19_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_19_C1MISR19_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_19_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_20 */

#define CSL_STC_CORE1_CURMISR_20_C1MISR20_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_20_C1MISR20_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_20_C1MISR20_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_20_C1MISR20_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_20_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_21 */

#define CSL_STC_CORE1_CURMISR_21_C1MISR21_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_21_C1MISR21_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_21_C1MISR21_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_21_C1MISR21_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_21_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_22 */

#define CSL_STC_CORE1_CURMISR_22_C1MISR22_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_22_C1MISR22_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_22_C1MISR22_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_22_C1MISR22_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_22_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_23 */

#define CSL_STC_CORE1_CURMISR_23_C1MISR23_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_23_C1MISR23_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_23_C1MISR23_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_23_C1MISR23_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_23_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_24 */

#define CSL_STC_CORE1_CURMISR_24_C1MISR24_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_24_C1MISR24_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_24_C1MISR24_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_24_C1MISR24_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_24_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_25 */

#define CSL_STC_CORE1_CURMISR_25_C1MISR25_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_25_C1MISR25_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_25_C1MISR25_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_25_C1MISR25_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_25_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_26 */

#define CSL_STC_CORE1_CURMISR_26_C1MISR26_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_26_C1MISR26_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_26_C1MISR26_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_26_C1MISR26_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_26_RESETVAL                                      (0x00000000U)

/* CORE1_CURMISR_27 */

#define CSL_STC_CORE1_CURMISR_27_C1MISR27_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE1_CURMISR_27_C1MISR27_SHIFT                                (0x00000000U)
#define CSL_STC_CORE1_CURMISR_27_C1MISR27_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE1_CURMISR_27_C1MISR27_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE1_CURMISR_27_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_0 */

#define CSL_STC_CORE2_CURMISR_0_C2MISR0_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_0_C2MISR0_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE2_CURMISR_0_C2MISR0_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE2_CURMISR_0_C2MISR0_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_0_RESETVAL                                       (0x00000000U)

/* CORE2_CURMISR_1 */

#define CSL_STC_CORE2_CURMISR_1_C2MISR1_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_1_C2MISR1_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE2_CURMISR_1_C2MISR1_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE2_CURMISR_1_C2MISR1_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_1_RESETVAL                                       (0x00000000U)

/* CORE2_CURMISR_2 */

#define CSL_STC_CORE2_CURMISR_2_C2MISR2_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_2_C2MISR2_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE2_CURMISR_2_C2MISR2_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE2_CURMISR_2_C2MISR2_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_2_RESETVAL                                       (0x00000000U)

/* CORE2_CURMISR_3 */

#define CSL_STC_CORE2_CURMISR_3_C2MISR3_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_3_C2MISR3_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE2_CURMISR_3_C2MISR3_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE2_CURMISR_3_C2MISR3_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_3_RESETVAL                                       (0x00000000U)

/* CORE2_CURMISR_4 */

#define CSL_STC_CORE2_CURMISR_4_C2MISR4_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_4_C2MISR4_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE2_CURMISR_4_C2MISR4_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE2_CURMISR_4_C2MISR4_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_4_RESETVAL                                       (0x00000000U)

/* CORE2_CURMISR_5 */

#define CSL_STC_CORE2_CURMISR_5_C2MISR5_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_5_C2MISR5_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE2_CURMISR_5_C2MISR5_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE2_CURMISR_5_C2MISR5_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_5_RESETVAL                                       (0x00000000U)

/* CORE2_CURMISR_6 */

#define CSL_STC_CORE2_CURMISR_6_C2MISR6_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_6_C2MISR6_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE2_CURMISR_6_C2MISR6_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE2_CURMISR_6_C2MISR6_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_6_RESETVAL                                       (0x00000000U)

/* CORE2_CURMISR_7 */

#define CSL_STC_CORE2_CURMISR_7_C2MISR7_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_7_C2MISR7_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE2_CURMISR_7_C2MISR7_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE2_CURMISR_7_C2MISR7_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_7_RESETVAL                                       (0x00000000U)

/* CORE2_CURMISR_8 */

#define CSL_STC_CORE2_CURMISR_8_C2MISR8_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_8_C2MISR8_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE2_CURMISR_8_C2MISR8_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE2_CURMISR_8_C2MISR8_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_8_RESETVAL                                       (0x00000000U)

/* CORE2_CURMISR_9 */

#define CSL_STC_CORE2_CURMISR_9_C2MISR9_MASK                                   (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_9_C2MISR9_SHIFT                                  (0x00000000U)
#define CSL_STC_CORE2_CURMISR_9_C2MISR9_RESETVAL                               (0x00000000U)
#define CSL_STC_CORE2_CURMISR_9_C2MISR9_MAX                                    (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_9_RESETVAL                                       (0x00000000U)

/* CORE2_CURMISR_10 */

#define CSL_STC_CORE2_CURMISR_10_C2MISR10_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_10_C2MISR10_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_10_C2MISR10_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_10_C2MISR10_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_10_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_11 */

#define CSL_STC_CORE2_CURMISR_11_C2MISR11_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_11_C2MISR11_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_11_C2MISR11_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_11_C2MISR11_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_11_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_12 */

#define CSL_STC_CORE2_CURMISR_12_C2MISR12_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_12_C2MISR12_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_12_C2MISR12_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_12_C2MISR12_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_12_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_13 */

#define CSL_STC_CORE2_CURMISR_13_C2MISR13_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_13_C2MISR13_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_13_C2MISR13_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_13_C2MISR13_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_13_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_14 */

#define CSL_STC_CORE2_CURMISR_14_C2MISR14_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_14_C2MISR14_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_14_C2MISR14_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_14_C2MISR14_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_14_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_15 */

#define CSL_STC_CORE2_CURMISR_15_C2MISR15_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_15_C2MISR15_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_15_C2MISR15_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_15_C2MISR15_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_15_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_16 */

#define CSL_STC_CORE2_CURMISR_16_C2MISR16_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_16_C2MISR16_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_16_C2MISR16_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_16_C2MISR16_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_16_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_17 */

#define CSL_STC_CORE2_CURMISR_17_C2MISR17_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_17_C2MISR17_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_17_C2MISR17_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_17_C2MISR17_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_17_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_18 */

#define CSL_STC_CORE2_CURMISR_18_C2MISR18_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_18_C2MISR18_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_18_C2MISR18_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_18_C2MISR18_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_18_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_19 */

#define CSL_STC_CORE2_CURMISR_19_C2MISR19_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_19_C2MISR19_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_19_C2MISR19_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_19_C2MISR19_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_19_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_20 */

#define CSL_STC_CORE2_CURMISR_20_C2MISR20_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_20_C2MISR20_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_20_C2MISR20_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_20_C2MISR20_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_20_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_21 */

#define CSL_STC_CORE2_CURMISR_21_C2MISR21_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_21_C2MISR21_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_21_C2MISR21_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_21_C2MISR21_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_21_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_22 */

#define CSL_STC_CORE2_CURMISR_22_C2MISR22_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_22_C2MISR22_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_22_C2MISR22_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_22_C2MISR22_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_22_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_23 */

#define CSL_STC_CORE2_CURMISR_23_C2MISR23_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_23_C2MISR23_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_23_C2MISR23_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_23_C2MISR23_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_23_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_24 */

#define CSL_STC_CORE2_CURMISR_24_C2MISR24_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_24_C2MISR24_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_24_C2MISR24_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_24_C2MISR24_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_24_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_25 */

#define CSL_STC_CORE2_CURMISR_25_C2MISR25_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_25_C2MISR25_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_25_C2MISR25_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_25_C2MISR25_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_25_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_26 */

#define CSL_STC_CORE2_CURMISR_26_C2MISR26_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_26_C2MISR26_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_26_C2MISR26_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_26_C2MISR26_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_26_RESETVAL                                      (0x00000000U)

/* CORE2_CURMISR_27 */

#define CSL_STC_CORE2_CURMISR_27_C2MISR27_MASK                                 (0xFFFFFFFFU)
#define CSL_STC_CORE2_CURMISR_27_C2MISR27_SHIFT                                (0x00000000U)
#define CSL_STC_CORE2_CURMISR_27_C2MISR27_RESETVAL                             (0x00000000U)
#define CSL_STC_CORE2_CURMISR_27_C2MISR27_MAX                                  (0xFFFFFFFFU)

#define CSL_STC_CORE2_CURMISR_27_RESETVAL                                      (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
