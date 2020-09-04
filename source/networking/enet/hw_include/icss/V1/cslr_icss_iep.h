/********************************************************************
 * Copyright (C) 2013-2014 Texas Instruments Incorporated.
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
*/
#ifndef CSLR_ICSSIEP_H
#define CSLR_ICSSIEP_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <drivers/hw_include/tistdtypes.h>


/**************************************************************************
* Register Overlay Structure for __ALL__
**************************************************************************/
typedef struct {
    volatile Uint32 GLOBAL_CFG_REG;
    volatile Uint32 GLOBAL_STATUS_REG;
    volatile Uint32 COMPEN_REG;
    volatile Uint32 SLOW_COMPEN_REG;
    volatile Uint32 COUNT_REG0;
    volatile Uint32 COUNT_REG1;
    volatile Uint32 CAP_CFG_REG;
    volatile Uint32 CAP_STATUS_REG;
    volatile Uint32 CAPR0_REG0;
    volatile Uint32 CAPR0_REG1;
    volatile Uint32 CAPR1_REG0;
    volatile Uint32 CAPR1_REG1;
    volatile Uint32 CAPR2_REG0;
    volatile Uint32 CAPR2_REG1;
    volatile Uint32 CAPR3_REG0;
    volatile Uint32 CAPR3_REG1;
    volatile Uint32 CAPR4_REG0;
    volatile Uint32 CAPR4_REG1;
    volatile Uint32 CAPR5_REG0;
    volatile Uint32 CAPR5_REG1;
    volatile Uint32 CAPR6_REG0;
    volatile Uint32 CAPR6_REG1;
    volatile Uint32 CAPF6_REG0;
    volatile Uint32 CAPF6_REG1;
    volatile Uint32 CAPR7_REG0;
    volatile Uint32 CAPR7_REG1;
    volatile Uint32 CAPF7_REG0;
    volatile Uint32 CAPF7_REG1;
    volatile Uint32 CMP_CFG_REG;
    volatile Uint32 CMP_STATUS_REG;
    volatile Uint32 CMP0_REG0;
    volatile Uint32 CMP0_REG1;
    volatile Uint32 CMP1_REG0;
    volatile Uint32 CMP1_REG1;
    volatile Uint32 CMP2_REG0;
    volatile Uint32 CMP2_REG1;
    volatile Uint32 CMP3_REG0;
    volatile Uint32 CMP3_REG1;
    volatile Uint32 CMP4_REG0;
    volatile Uint32 CMP4_REG1;
    volatile Uint32 CMP5_REG0;
    volatile Uint32 CMP5_REG1;
    volatile Uint32 CMP6_REG0;
    volatile Uint32 CMP6_REG1;
    volatile Uint32 CMP7_REG0;
    volatile Uint32 CMP7_REG1;
    volatile Uint32 RXIPG0_REG;
    volatile Uint32 RXIPG1_REG;
    volatile Uint32 CMP8_REG0;
    volatile Uint32 CMP8_REG1;
    volatile Uint32 CMP9_REG0;
    volatile Uint32 CMP9_REG1;
    volatile Uint32 CMP10_REG0;
    volatile Uint32 CMP10_REG1;
    volatile Uint32 CMP11_REG0;
    volatile Uint32 CMP11_REG1;
    volatile Uint32 CMP12_REG0;
    volatile Uint32 CMP12_REG1;
    volatile Uint32 CMP13_REG0;
    volatile Uint32 CMP13_REG1;
    volatile Uint32 CMP14_REG0;
    volatile Uint32 CMP14_REG1;
    volatile Uint32 CMP15_REG0;
    volatile Uint32 CMP15_REG1;
    volatile Uint32 COUNT_RESET_VAL_REG0;
    volatile Uint32 COUNT_RESET_VAL_REG1;
    volatile Uint32 PWM_REG;
    volatile Uint8  RSVD0[116];
    volatile Uint32 SYNC_CTRL_REG;
    volatile Uint32 SYNC_FIRST_STAT_REG;
    volatile Uint32 SYNC0_STAT_REG;
    volatile Uint32 SYNC1_STAT_REG;
    volatile Uint32 SYNC_PWIDTH_REG;
    volatile Uint32 SYNC0_PERIOD_REG;
    volatile Uint32 SYNC1_DELAY_REG;
    volatile Uint32 SYNC_START_REG;
    volatile Uint8  RSVD1[96];
    volatile Uint32 WD_PREDIV_REG;
    volatile Uint32 PDI_WD_TIM_REG;
    volatile Uint32 PD_WD_TIM_REG;
    volatile Uint32 WD_STATUS_REG;
    volatile Uint32 WD_EXP_CNT_REG;
    volatile Uint32 WD_CTRL_REG;
    volatile Uint8  RSVD2[232];
    volatile Uint32 DIGIO_CTRL_REG;
    volatile Uint32 DIGIO_STATUS_REG;
    volatile Uint32 DIGIO_DATA_IN_REG;
    volatile Uint32 DIGIO_DATA_IN_RAW_REG;
    volatile Uint32 DIGIO_DATA_OUT_REG;
    volatile Uint32 DIGIO_DATA_OUT_EN_REG;
    volatile Uint32 DIGIO_EXP_REG;
    volatile Uint8  RSVD3[2376];
} CSL_IcssIepRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

/* GLOBAL_CFG_REG */
#define CSL_ICSSIEP_GLOBAL_CFG_REG                              (0x0U)

/* GLOBAL_STATUS_REG */
#define CSL_ICSSIEP_GLOBAL_STATUS_REG                           (0x4U)

/* COMPEN_REG */
#define CSL_ICSSIEP_COMPEN_REG                                  (0x8U)

/* SLOW_COMPEN_REG */
#define CSL_ICSSIEP_SLOW_COMPEN_REG                             (0xCU)

/* COUNT_REG0 */
#define CSL_ICSSIEP_COUNT_REG0                                  (0x10U)

/* COUNT_REG1 */
#define CSL_ICSSIEP_COUNT_REG1                                  (0x14U)

/* CAP_CFG_REG */
#define CSL_ICSSIEP_CAP_CFG_REG                                 (0x18U)

/* CAP_STATUS_REG */
#define CSL_ICSSIEP_CAP_STATUS_REG                              (0x1CU)

/* CAPR0_REG0 */
#define CSL_ICSSIEP_CAPR0_REG0                                  (0x20U)

/* CAPR0_REG1 */
#define CSL_ICSSIEP_CAPR0_REG1                                  (0x24U)

/* CAPR1_REG0 */
#define CSL_ICSSIEP_CAPR1_REG0                                  (0x28U)

/* CAPR1_REG1 */
#define CSL_ICSSIEP_CAPR1_REG1                                  (0x2CU)

/* CAPR2_REG0 */
#define CSL_ICSSIEP_CAPR2_REG0                                  (0x30U)

/* CAPR2_REG1 */
#define CSL_ICSSIEP_CAPR2_REG1                                  (0x34U)

/* CAPR3_REG0 */
#define CSL_ICSSIEP_CAPR3_REG0                                  (0x38U)

/* CAPR3_REG1 */
#define CSL_ICSSIEP_CAPR3_REG1                                  (0x3CU)

/* CAPR4_REG0 */
#define CSL_ICSSIEP_CAPR4_REG0                                  (0x40U)

/* CAPR4_REG1 */
#define CSL_ICSSIEP_CAPR4_REG1                                  (0x44U)

/* CAPR5_REG0 */
#define CSL_ICSSIEP_CAPR5_REG0                                  (0x48U)

/* CAPR5_REG1 */
#define CSL_ICSSIEP_CAPR5_REG1                                  (0x4CU)

/* CAPR6_REG0 */
#define CSL_ICSSIEP_CAPR6_REG0                                  (0x50U)

/* CAPR6_REG1 */
#define CSL_ICSSIEP_CAPR6_REG1                                  (0x54U)

/* CAPF6_REG0 */
#define CSL_ICSSIEP_CAPF6_REG0                                  (0x58U)

/* CAPF6_REG1 */
#define CSL_ICSSIEP_CAPF6_REG1                                  (0x5CU)

/* CAPR7_REG0 */
#define CSL_ICSSIEP_CAPR7_REG0                                  (0x60U)

/* CAPR7_REG1 */
#define CSL_ICSSIEP_CAPR7_REG1                                  (0x64U)

/* CAPF7_REG0 */
#define CSL_ICSSIEP_CAPF7_REG0                                  (0x68U)

/* CAPF7_REG1 */
#define CSL_ICSSIEP_CAPF7_REG1                                  (0x6CU)

/* CMP_CFG_REG */
#define CSL_ICSSIEP_CMP_CFG_REG                                 (0x70U)

/* CMP_STATUS_REG */
#define CSL_ICSSIEP_CMP_STATUS_REG                              (0x74U)

/* CMP0_REG0 */
#define CSL_ICSSIEP_CMP0_REG0                                   (0x78U)

/* CMP0_REG1 */
#define CSL_ICSSIEP_CMP0_REG1                                   (0x7CU)

/* CMP1_REG0 */
#define CSL_ICSSIEP_CMP1_REG0                                   (0x80U)

/* CMP1_REG1 */
#define CSL_ICSSIEP_CMP1_REG1                                   (0x84U)

/* CMP2_REG0 */
#define CSL_ICSSIEP_CMP2_REG0                                   (0x88U)

/* CMP2_REG1 */
#define CSL_ICSSIEP_CMP2_REG1                                   (0x8CU)

/* CMP3_REG0 */
#define CSL_ICSSIEP_CMP3_REG0                                   (0x90U)

/* CMP3_REG1 */
#define CSL_ICSSIEP_CMP3_REG1                                   (0x94U)

/* CMP4_REG0 */
#define CSL_ICSSIEP_CMP4_REG0                                   (0x98U)

/* CMP4_REG1 */
#define CSL_ICSSIEP_CMP4_REG1                                   (0x9CU)

/* CMP5_REG0 */
#define CSL_ICSSIEP_CMP5_REG0                                   (0xA0U)

/* CMP5_REG1 */
#define CSL_ICSSIEP_CMP5_REG1                                   (0xA4U)

/* CMP6_REG0 */
#define CSL_ICSSIEP_CMP6_REG0                                   (0xA8U)

/* CMP6_REG1 */
#define CSL_ICSSIEP_CMP6_REG1                                   (0xACU)

/* CMP7_REG0 */
#define CSL_ICSSIEP_CMP7_REG0                                   (0xB0U)

/* CMP7_REG1 */
#define CSL_ICSSIEP_CMP7_REG1                                   (0xB4U)

/* RXIPG0_REG */
#define CSL_ICSSIEP_RXIPG0_REG                                  (0xB8U)

/* RXIPG1_REG */
#define CSL_ICSSIEP_RXIPG1_REG                                  (0xBCU)

/* CMP8_REG0 */
#define CSL_ICSSIEP_CMP8_REG0                                   (0xC0U)

/* CMP8_REG1 */
#define CSL_ICSSIEP_CMP8_REG1                                   (0xC4U)

/* CMP9_REG0 */
#define CSL_ICSSIEP_CMP9_REG0                                   (0xC8U)

/* CMP9_REG1 */
#define CSL_ICSSIEP_CMP9_REG1                                   (0xCCU)

/* CMP10_REG0 */
#define CSL_ICSSIEP_CMP10_REG0                                  (0xD0U)

/* CMP10_REG1 */
#define CSL_ICSSIEP_CMP10_REG1                                  (0xD4U)

/* CMP11_REG0 */
#define CSL_ICSSIEP_CMP11_REG0                                  (0xD8U)

/* CMP11_REG1 */
#define CSL_ICSSIEP_CMP11_REG1                                  (0xDCU)

/* CMP12_REG0 */
#define CSL_ICSSIEP_CMP12_REG0                                  (0xE0U)

/* CMP12_REG1 */
#define CSL_ICSSIEP_CMP12_REG1                                  (0xE4U)

/* CMP13_REG0 */
#define CSL_ICSSIEP_CMP13_REG0                                  (0xE8U)

/* CMP13_REG1 */
#define CSL_ICSSIEP_CMP13_REG1                                  (0xECU)

/* CMP14_REG0 */
#define CSL_ICSSIEP_CMP14_REG0                                  (0xF0U)

/* CMP14_REG1 */
#define CSL_ICSSIEP_CMP14_REG1                                  (0xF4U)

/* CMP15_REG0 */
#define CSL_ICSSIEP_CMP15_REG0                                  (0xF8U)

/* CMP15_REG1 */
#define CSL_ICSSIEP_CMP15_REG1                                  (0xFCU)

/* COUNT_RESET_VAL_REG0 */
#define CSL_ICSSIEP_COUNT_RESET_VAL_REG0                        (0x100U)

/* COUNT_RESET_VAL_REG1 */
#define CSL_ICSSIEP_COUNT_RESET_VAL_REG1                        (0x104U)

/* PWM_REG */
#define CSL_ICSSIEP_PWM_REG                                     (0x108U)

/* SYNC_CTRL_REG */
#define CSL_ICSSIEP_SYNC_CTRL_REG                               (0x180U)

/* SYNC_FIRST_STAT_REG */
#define CSL_ICSSIEP_SYNC_FIRST_STAT_REG                         (0x184U)

/* SYNC0_STAT_REG */
#define CSL_ICSSIEP_SYNC0_STAT_REG                              (0x188U)

/* SYNC1_STAT_REG */
#define CSL_ICSSIEP_SYNC1_STAT_REG                              (0x18CU)

/* SYNC_PWIDTH_REG */
#define CSL_ICSSIEP_SYNC_PWIDTH_REG                             (0x190U)

/* SYNC0_PERIOD_REG */
#define CSL_ICSSIEP_SYNC0_PERIOD_REG                            (0x194U)

/* SYNC1_DELAY_REG */
#define CSL_ICSSIEP_SYNC1_DELAY_REG                             (0x198U)

/* SYNC_START_REG */
#define CSL_ICSSIEP_SYNC_START_REG                              (0x19CU)

/* WD_PREDIV_REG */
#define CSL_ICSSIEP_WD_PREDIV_REG                               (0x200U)

/* PDI_WD_TIM_REG */
#define CSL_ICSSIEP_PDI_WD_TIM_REG                              (0x204U)

/* PD_WD_TIM_REG */
#define CSL_ICSSIEP_PD_WD_TIM_REG                               (0x208U)

/* WD_STATUS_REG */
#define CSL_ICSSIEP_WD_STATUS_REG                               (0x20CU)

/* WD_EXP_CNT_REG */
#define CSL_ICSSIEP_WD_EXP_CNT_REG                              (0x210U)

/* WD_CTRL_REG */
#define CSL_ICSSIEP_WD_CTRL_REG                                 (0x214U)

/* DIGIO_CTRL_REG */
#define CSL_ICSSIEP_DIGIO_CTRL_REG                              (0x300U)

/* DIGIO_STATUS_REG */
#define CSL_ICSSIEP_DIGIO_STATUS_REG                            (0x304U)

/* DIGIO_DATA_IN_REG */
#define CSL_ICSSIEP_DIGIO_DATA_IN_REG                           (0x308U)

/* DIGIO_DATA_IN_RAW_REG */
#define CSL_ICSSIEP_DIGIO_DATA_IN_RAW_REG                       (0x30CU)

/* DIGIO_DATA_OUT_REG */
#define CSL_ICSSIEP_DIGIO_DATA_OUT_REG                          (0x310U)

/* DIGIO_DATA_OUT_EN_REG */
#define CSL_ICSSIEP_DIGIO_DATA_OUT_EN_REG                       (0x314U)

/* DIGIO_EXP_REG */
#define CSL_ICSSIEP_DIGIO_EXP_REG                               (0x318U)


/**************************************************************************
* Field Definition Macros
**************************************************************************/

/* GLOBAL_CFG_REG */

#define CSL_ICSSIEP_GLOBAL_CFG_REG_CNT_ENABLE_MASK              (0x00000001U)
#define CSL_ICSSIEP_GLOBAL_CFG_REG_CNT_ENABLE_SHIFT             (0U)
#define CSL_ICSSIEP_GLOBAL_CFG_REG_CNT_ENABLE_RESETVAL          (0x00000000U)
#define CSL_ICSSIEP_GLOBAL_CFG_REG_CNT_ENABLE_MAX               (0x00000001U)

#define CSL_ICSSIEP_GLOBAL_CFG_REG_DEFAULT_INC_MASK             (0x000000F0U)
#define CSL_ICSSIEP_GLOBAL_CFG_REG_DEFAULT_INC_SHIFT            (4U)
#define CSL_ICSSIEP_GLOBAL_CFG_REG_DEFAULT_INC_RESETVAL         (0x00000000U)
#define CSL_ICSSIEP_GLOBAL_CFG_REG_DEFAULT_INC_MAX              (0x0000000fU)

#define CSL_ICSSIEP_GLOBAL_CFG_REG_CMP_INC_MASK                 (0x000FFF00U)
#define CSL_ICSSIEP_GLOBAL_CFG_REG_CMP_INC_SHIFT                (8U)
#define CSL_ICSSIEP_GLOBAL_CFG_REG_CMP_INC_RESETVAL             (0x00000000U)
#define CSL_ICSSIEP_GLOBAL_CFG_REG_CMP_INC_MAX                  (0x00000fffU)

#define CSL_ICSSIEP_GLOBAL_CFG_REG_RESETVAL                     (0x00000000U)

/* GLOBAL_STATUS_REG */

#define CSL_ICSSIEP_GLOBAL_STATUS_REG_CNT_OVF_MASK              (0x00000001U)
#define CSL_ICSSIEP_GLOBAL_STATUS_REG_CNT_OVF_SHIFT             (0U)
#define CSL_ICSSIEP_GLOBAL_STATUS_REG_CNT_OVF_RESETVAL          (0x00000000U)
#define CSL_ICSSIEP_GLOBAL_STATUS_REG_CNT_OVF_MAX               (0x00000001U)

#define CSL_ICSSIEP_GLOBAL_STATUS_REG_RESETVAL                  (0x00000000U)

/* COMPEN_REG */

#define CSL_ICSSIEP_COMPEN_REG_COMPEN_CNT_MASK                  (0x007FFFFFU)
#define CSL_ICSSIEP_COMPEN_REG_COMPEN_CNT_SHIFT                 (0U)
#define CSL_ICSSIEP_COMPEN_REG_COMPEN_CNT_RESETVAL              (0x00000000U)
#define CSL_ICSSIEP_COMPEN_REG_COMPEN_CNT_MAX                   (0x007fffffU)

#define CSL_ICSSIEP_COMPEN_REG_RESETVAL                         (0x00000000U)

/* SLOW_COMPEN_REG */

#define CSL_ICSSIEP_SLOW_COMPEN_REG_SLOW_COMPEN_CNT_MASK        (0xFFFFFFFFU)
#define CSL_ICSSIEP_SLOW_COMPEN_REG_SLOW_COMPEN_CNT_SHIFT       (0U)
#define CSL_ICSSIEP_SLOW_COMPEN_REG_SLOW_COMPEN_CNT_RESETVAL    (0x00000000U)
#define CSL_ICSSIEP_SLOW_COMPEN_REG_SLOW_COMPEN_CNT_MAX         (0xffffffffU)

#define CSL_ICSSIEP_SLOW_COMPEN_REG_RESETVAL                    (0x00000000U)

/* COUNT_REG0 */

#define CSL_ICSSIEP_COUNT_REG0_LOW_COUNT_MASK                   (0xFFFFFFFFU)
#define CSL_ICSSIEP_COUNT_REG0_LOW_COUNT_SHIFT                  (0U)
#define CSL_ICSSIEP_COUNT_REG0_LOW_COUNT_RESETVAL               (0x00000000U)
#define CSL_ICSSIEP_COUNT_REG0_LOW_COUNT_MAX                    (0xffffffffU)

#define CSL_ICSSIEP_COUNT_REG0_RESETVAL                         (0x00000000U)

/* COUNT_REG1 */

#define CSL_ICSSIEP_COUNT_REG1_HIGH_COUNT_MASK                  (0xFFFFFFFFU)
#define CSL_ICSSIEP_COUNT_REG1_HIGH_COUNT_SHIFT                 (0U)
#define CSL_ICSSIEP_COUNT_REG1_HIGH_COUNT_RESETVAL              (0x00000000U)
#define CSL_ICSSIEP_COUNT_REG1_HIGH_COUNT_MAX                   (0xffffffffU)

#define CSL_ICSSIEP_COUNT_REG1_RESETVAL                         (0x00000000U)

/* CAP_CFG_REG */

#define CSL_ICSSIEP_CAP_CFG_REG_CAP_EN_MASK                     (0x000003FFU)
#define CSL_ICSSIEP_CAP_CFG_REG_CAP_EN_SHIFT                    (0U)
#define CSL_ICSSIEP_CAP_CFG_REG_CAP_EN_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAP_CFG_REG_CAP_EN_MAX                      (0x000003ffU)

#define CSL_ICSSIEP_CAP_CFG_REG_CAP_ASYNC_EN_MASK               (0x0003FC00U)
#define CSL_ICSSIEP_CAP_CFG_REG_CAP_ASYNC_EN_SHIFT              (10U)
#define CSL_ICSSIEP_CAP_CFG_REG_CAP_ASYNC_EN_RESETVAL           (0x00000000U)
#define CSL_ICSSIEP_CAP_CFG_REG_CAP_ASYNC_EN_MAX                (0x000000ffU)

#define CSL_ICSSIEP_CAP_CFG_REG_RESETVAL                        (0x00000000U)

/* CAP_STATUS_REG */

#define CSL_ICSSIEP_CAP_STATUS_REG_CAP_VALID_MASK               (0x000007FFU)
#define CSL_ICSSIEP_CAP_STATUS_REG_CAP_VALID_SHIFT              (0U)
#define CSL_ICSSIEP_CAP_STATUS_REG_CAP_VALID_RESETVAL           (0x00000000U)
#define CSL_ICSSIEP_CAP_STATUS_REG_CAP_VALID_MAX                (0x000007ffU)

#define CSL_ICSSIEP_CAP_STATUS_REG_CAP_RAW_MASK                 (0x00FF0000U)
#define CSL_ICSSIEP_CAP_STATUS_REG_CAP_RAW_SHIFT                (16U)
#define CSL_ICSSIEP_CAP_STATUS_REG_CAP_RAW_RESETVAL             (0x00000000U)
#define CSL_ICSSIEP_CAP_STATUS_REG_CAP_RAW_MAX                  (0x000000ffU)

#define CSL_ICSSIEP_CAP_STATUS_REG_RESETVAL                     (0x00000000U)

/* CAPR0_REG0 */

#define CSL_ICSSIEP_CAPR0_REG0_CAPR0_0_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPR0_REG0_CAPR0_0_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPR0_REG0_CAPR0_0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPR0_REG0_CAPR0_0_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPR0_REG0_RESETVAL                         (0x00000000U)

/* CAPR0_REG1 */

#define CSL_ICSSIEP_CAPR0_REG1_CAPR0_1_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPR0_REG1_CAPR0_1_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPR0_REG1_CAPR0_1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPR0_REG1_CAPR0_1_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPR0_REG1_RESETVAL                         (0x00000000U)

/* CAPR1_REG0 */

#define CSL_ICSSIEP_CAPR1_REG0_CAPR1_0_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPR1_REG0_CAPR1_0_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPR1_REG0_CAPR1_0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPR1_REG0_CAPR1_0_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPR1_REG0_RESETVAL                         (0x00000000U)

/* CAPR1_REG1 */

#define CSL_ICSSIEP_CAPR1_REG1_CAPR1_1_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPR1_REG1_CAPR1_1_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPR1_REG1_CAPR1_1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPR1_REG1_CAPR1_1_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPR1_REG1_RESETVAL                         (0x00000000U)

/* CAPR2_REG0 */

#define CSL_ICSSIEP_CAPR2_REG0_CAPR2_0_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPR2_REG0_CAPR2_0_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPR2_REG0_CAPR2_0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPR2_REG0_CAPR2_0_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPR2_REG0_RESETVAL                         (0x00000000U)

/* CAPR2_REG1 */

#define CSL_ICSSIEP_CAPR2_REG1_CAPR2_1_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPR2_REG1_CAPR2_1_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPR2_REG1_CAPR2_1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPR2_REG1_CAPR2_1_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPR2_REG1_RESETVAL                         (0x00000000U)

/* CAPR3_REG0 */

#define CSL_ICSSIEP_CAPR3_REG0_CAPR3_0_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPR3_REG0_CAPR3_0_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPR3_REG0_CAPR3_0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPR3_REG0_CAPR3_0_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPR3_REG0_RESETVAL                         (0x00000000U)

/* CAPR3_REG1 */

#define CSL_ICSSIEP_CAPR3_REG1_CAPR3_1_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPR3_REG1_CAPR3_1_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPR3_REG1_CAPR3_1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPR3_REG1_CAPR3_1_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPR3_REG1_RESETVAL                         (0x00000000U)

/* CAPR4_REG0 */

#define CSL_ICSSIEP_CAPR4_REG0_CAPR4_0_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPR4_REG0_CAPR4_0_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPR4_REG0_CAPR4_0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPR4_REG0_CAPR4_0_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPR4_REG0_RESETVAL                         (0x00000000U)

/* CAPR4_REG1 */

#define CSL_ICSSIEP_CAPR4_REG1_CAPR4_1_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPR4_REG1_CAPR4_1_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPR4_REG1_CAPR4_1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPR4_REG1_CAPR4_1_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPR4_REG1_RESETVAL                         (0x00000000U)

/* CAPR5_REG0 */

#define CSL_ICSSIEP_CAPR5_REG0_CAPR5_0_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPR5_REG0_CAPR5_0_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPR5_REG0_CAPR5_0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPR5_REG0_CAPR5_0_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPR5_REG0_RESETVAL                         (0x00000000U)

/* CAPR5_REG1 */

#define CSL_ICSSIEP_CAPR5_REG1_CAPR5_1_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPR5_REG1_CAPR5_1_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPR5_REG1_CAPR5_1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPR5_REG1_CAPR5_1_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPR5_REG1_RESETVAL                         (0x00000000U)

/* CAPR6_REG0 */

#define CSL_ICSSIEP_CAPR6_REG0_CAPR6_0_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPR6_REG0_CAPR6_0_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPR6_REG0_CAPR6_0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPR6_REG0_CAPR6_0_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPR6_REG0_RESETVAL                         (0x00000000U)

/* CAPR6_REG1 */

#define CSL_ICSSIEP_CAPR6_REG1_CAPR6_1_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPR6_REG1_CAPR6_1_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPR6_REG1_CAPR6_1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPR6_REG1_CAPR6_1_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPR6_REG1_RESETVAL                         (0x00000000U)

/* CAPF6_REG0 */

#define CSL_ICSSIEP_CAPF6_REG0_CAPF6_0_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPF6_REG0_CAPF6_0_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPF6_REG0_CAPF6_0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPF6_REG0_CAPF6_0_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPF6_REG0_RESETVAL                         (0x00000000U)

/* CAPF6_REG1 */

#define CSL_ICSSIEP_CAPF6_REG1_CAPF6_1_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPF6_REG1_CAPF6_1_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPF6_REG1_CAPF6_1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPF6_REG1_CAPF6_1_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPF6_REG1_RESETVAL                         (0x00000000U)

/* CAPR7_REG0 */

#define CSL_ICSSIEP_CAPR7_REG0_CAPR7_0_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPR7_REG0_CAPR7_0_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPR7_REG0_CAPR7_0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPR7_REG0_CAPR7_0_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPR7_REG0_RESETVAL                         (0x00000000U)

/* CAPR7_REG1 */

#define CSL_ICSSIEP_CAPR7_REG1_CAPR7_1_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPR7_REG1_CAPR7_1_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPR7_REG1_CAPR7_1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPR7_REG1_CAPR7_1_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPR7_REG1_RESETVAL                         (0x00000000U)

/* CAPF7_REG0 */

#define CSL_ICSSIEP_CAPF7_REG0_CAPF7_0_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPF7_REG0_CAPF7_0_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPF7_REG0_CAPF7_0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPF7_REG0_CAPF7_0_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPF7_REG0_RESETVAL                         (0x00000000U)

/* CAPF7_REG1 */

#define CSL_ICSSIEP_CAPF7_REG1_CAPF7_1_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CAPF7_REG1_CAPF7_1_SHIFT                    (0U)
#define CSL_ICSSIEP_CAPF7_REG1_CAPF7_1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CAPF7_REG1_CAPF7_1_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CAPF7_REG1_RESETVAL                         (0x00000000U)

/* CMP_CFG_REG */

#define CSL_ICSSIEP_CMP_CFG_REG_CMP0_RST_CNT_EN_MASK            (0x00000001U)
#define CSL_ICSSIEP_CMP_CFG_REG_CMP0_RST_CNT_EN_SHIFT           (0U)
#define CSL_ICSSIEP_CMP_CFG_REG_CMP0_RST_CNT_EN_RESETVAL        (0x00000000U)
#define CSL_ICSSIEP_CMP_CFG_REG_CMP0_RST_CNT_EN_MAX             (0x00000001U)

#define CSL_ICSSIEP_CMP_CFG_REG_CMP_EN_MASK                     (0x0001FFFEU)
#define CSL_ICSSIEP_CMP_CFG_REG_CMP_EN_SHIFT                    (1U)
#define CSL_ICSSIEP_CMP_CFG_REG_CMP_EN_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CMP_CFG_REG_CMP_EN_MAX                      (0x0000ffffU)

#define CSL_ICSSIEP_CMP_CFG_REG_RESETVAL                        (0x00000000U)

/* CMP_STATUS_REG */

#define CSL_ICSSIEP_CMP_STATUS_REG_CMP_STATUS_MASK              (0x0000FFFFU)
#define CSL_ICSSIEP_CMP_STATUS_REG_CMP_STATUS_SHIFT             (0U)
#define CSL_ICSSIEP_CMP_STATUS_REG_CMP_STATUS_RESETVAL          (0x00000000U)
#define CSL_ICSSIEP_CMP_STATUS_REG_CMP_STATUS_MAX               (0x0000ffffU)

#define CSL_ICSSIEP_CMP_STATUS_REG_RESETVAL                     (0x00000000U)

/* CMP0_REG0 */

#define CSL_ICSSIEP_CMP0_REG0_CMP0_0_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP0_REG0_CMP0_0_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP0_REG0_CMP0_0_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP0_REG0_CMP0_0_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP0_REG0_RESETVAL                          (0x00000000U)

/* CMP0_REG1 */

#define CSL_ICSSIEP_CMP0_REG1_CMP0_1_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP0_REG1_CMP0_1_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP0_REG1_CMP0_1_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP0_REG1_CMP0_1_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP0_REG1_RESETVAL                          (0x00000000U)

/* CMP1_REG0 */

#define CSL_ICSSIEP_CMP1_REG0_CMP1_0_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP1_REG0_CMP1_0_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP1_REG0_CMP1_0_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP1_REG0_CMP1_0_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP1_REG0_RESETVAL                          (0x00000000U)

/* CMP1_REG1 */

#define CSL_ICSSIEP_CMP1_REG1_CMP1_1_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP1_REG1_CMP1_1_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP1_REG1_CMP1_1_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP1_REG1_CMP1_1_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP1_REG1_RESETVAL                          (0x00000000U)

/* CMP2_REG0 */

#define CSL_ICSSIEP_CMP2_REG0_CMP2_0_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP2_REG0_CMP2_0_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP2_REG0_CMP2_0_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP2_REG0_CMP2_0_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP2_REG0_RESETVAL                          (0x00000000U)

/* CMP2_REG1 */

#define CSL_ICSSIEP_CMP2_REG1_CMP2_1_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP2_REG1_CMP2_1_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP2_REG1_CMP2_1_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP2_REG1_CMP2_1_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP2_REG1_RESETVAL                          (0x00000000U)

/* CMP3_REG0 */

#define CSL_ICSSIEP_CMP3_REG0_CMP3_0_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP3_REG0_CMP3_0_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP3_REG0_CMP3_0_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP3_REG0_CMP3_0_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP3_REG0_RESETVAL                          (0x00000000U)

/* CMP3_REG1 */

#define CSL_ICSSIEP_CMP3_REG1_CMP3_1_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP3_REG1_CMP3_1_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP3_REG1_CMP3_1_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP3_REG1_CMP3_1_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP3_REG1_RESETVAL                          (0x00000000U)

/* CMP4_REG0 */

#define CSL_ICSSIEP_CMP4_REG0_CMP4_0_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP4_REG0_CMP4_0_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP4_REG0_CMP4_0_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP4_REG0_CMP4_0_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP4_REG0_RESETVAL                          (0x00000000U)

/* CMP4_REG1 */

#define CSL_ICSSIEP_CMP4_REG1_CMP4_1_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP4_REG1_CMP4_1_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP4_REG1_CMP4_1_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP4_REG1_CMP4_1_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP4_REG1_RESETVAL                          (0x00000000U)

/* CMP5_REG0 */

#define CSL_ICSSIEP_CMP5_REG0_CMP5_0_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP5_REG0_CMP5_0_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP5_REG0_CMP5_0_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP5_REG0_CMP5_0_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP5_REG0_RESETVAL                          (0x00000000U)

/* CMP5_REG1 */

#define CSL_ICSSIEP_CMP5_REG1_CMP5_1_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP5_REG1_CMP5_1_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP5_REG1_CMP5_1_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP5_REG1_CMP5_1_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP5_REG1_RESETVAL                          (0x00000000U)

/* CMP6_REG0 */

#define CSL_ICSSIEP_CMP6_REG0_CMP6_0_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP6_REG0_CMP6_0_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP6_REG0_CMP6_0_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP6_REG0_CMP6_0_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP6_REG0_RESETVAL                          (0x00000000U)

/* CMP6_REG1 */

#define CSL_ICSSIEP_CMP6_REG1_CMP6_1_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP6_REG1_CMP6_1_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP6_REG1_CMP6_1_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP6_REG1_CMP6_1_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP6_REG1_RESETVAL                          (0x00000000U)

/* CMP7_REG0 */

#define CSL_ICSSIEP_CMP7_REG0_CMP7_0_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP7_REG0_CMP7_0_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP7_REG0_CMP7_0_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP7_REG0_CMP7_0_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP7_REG0_RESETVAL                          (0x00000000U)

/* CMP7_REG1 */

#define CSL_ICSSIEP_CMP7_REG1_CMP7_1_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP7_REG1_CMP7_1_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP7_REG1_CMP7_1_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP7_REG1_CMP7_1_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP7_REG1_RESETVAL                          (0x00000000U)

/* RXIPG0_REG */

#define CSL_ICSSIEP_RXIPG0_REG_RX_IPG0_MASK                     (0x0000FFFFU)
#define CSL_ICSSIEP_RXIPG0_REG_RX_IPG0_SHIFT                    (0U)
#define CSL_ICSSIEP_RXIPG0_REG_RX_IPG0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_RXIPG0_REG_RX_IPG0_MAX                      (0x0000ffffU)

#define CSL_ICSSIEP_RXIPG0_REG_RX_MIN_IPG0_MASK                 (0xFFFF0000U)
#define CSL_ICSSIEP_RXIPG0_REG_RX_MIN_IPG0_SHIFT                (16U)
#define CSL_ICSSIEP_RXIPG0_REG_RX_MIN_IPG0_RESETVAL             (0x00000000U)
#define CSL_ICSSIEP_RXIPG0_REG_RX_MIN_IPG0_MAX                  (0x0000ffffU)

#define CSL_ICSSIEP_RXIPG0_REG_RESETVAL                         (0x00000000U)

/* RXIPG1_REG */

#define CSL_ICSSIEP_RXIPG1_REG_RX_IPG1_MASK                     (0x0000FFFFU)
#define CSL_ICSSIEP_RXIPG1_REG_RX_IPG1_SHIFT                    (0U)
#define CSL_ICSSIEP_RXIPG1_REG_RX_IPG1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_RXIPG1_REG_RX_IPG1_MAX                      (0x0000ffffU)

#define CSL_ICSSIEP_RXIPG1_REG_RX_MIN_IPG1_MASK                 (0xFFFF0000U)
#define CSL_ICSSIEP_RXIPG1_REG_RX_MIN_IPG1_SHIFT                (16U)
#define CSL_ICSSIEP_RXIPG1_REG_RX_MIN_IPG1_RESETVAL             (0x00000000U)
#define CSL_ICSSIEP_RXIPG1_REG_RX_MIN_IPG1_MAX                  (0x0000ffffU)

#define CSL_ICSSIEP_RXIPG1_REG_RESETVAL                         (0x00000000U)

/* CMP8_REG0 */

#define CSL_ICSSIEP_CMP8_REG0_CMP8_0_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP8_REG0_CMP8_0_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP8_REG0_CMP8_0_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP8_REG0_CMP8_0_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP8_REG0_RESETVAL                          (0x00000000U)

/* CMP8_REG1 */

#define CSL_ICSSIEP_CMP8_REG1_CMP8_1_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP8_REG1_CMP8_1_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP8_REG1_CMP8_1_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP8_REG1_CMP8_1_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP8_REG1_RESETVAL                          (0x00000000U)

/* CMP9_REG0 */

#define CSL_ICSSIEP_CMP9_REG0_CMP9_0_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP9_REG0_CMP9_0_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP9_REG0_CMP9_0_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP9_REG0_CMP9_0_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP9_REG0_RESETVAL                          (0x00000000U)

/* CMP9_REG1 */

#define CSL_ICSSIEP_CMP9_REG1_CMP9_1_MASK                       (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP9_REG1_CMP9_1_SHIFT                      (0U)
#define CSL_ICSSIEP_CMP9_REG1_CMP9_1_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_CMP9_REG1_CMP9_1_MAX                        (0xffffffffU)

#define CSL_ICSSIEP_CMP9_REG1_RESETVAL                          (0x00000000U)

/* CMP10_REG0 */

#define CSL_ICSSIEP_CMP10_REG0_CMP10_0_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP10_REG0_CMP10_0_SHIFT                    (0U)
#define CSL_ICSSIEP_CMP10_REG0_CMP10_0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CMP10_REG0_CMP10_0_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CMP10_REG0_RESETVAL                         (0x00000000U)

/* CMP10_REG1 */

#define CSL_ICSSIEP_CMP10_REG1_CMP10_1_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP10_REG1_CMP10_1_SHIFT                    (0U)
#define CSL_ICSSIEP_CMP10_REG1_CMP10_1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CMP10_REG1_CMP10_1_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CMP10_REG1_RESETVAL                         (0x00000000U)

/* CMP11_REG0 */

#define CSL_ICSSIEP_CMP11_REG0_CMP11_0_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP11_REG0_CMP11_0_SHIFT                    (0U)
#define CSL_ICSSIEP_CMP11_REG0_CMP11_0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CMP11_REG0_CMP11_0_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CMP11_REG0_RESETVAL                         (0x00000000U)

/* CMP11_REG1 */

#define CSL_ICSSIEP_CMP11_REG1_CMP11_1_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP11_REG1_CMP11_1_SHIFT                    (0U)
#define CSL_ICSSIEP_CMP11_REG1_CMP11_1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CMP11_REG1_CMP11_1_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CMP11_REG1_RESETVAL                         (0x00000000U)

/* CMP12_REG0 */

#define CSL_ICSSIEP_CMP12_REG0_CMP12_0_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP12_REG0_CMP12_0_SHIFT                    (0U)
#define CSL_ICSSIEP_CMP12_REG0_CMP12_0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CMP12_REG0_CMP12_0_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CMP12_REG0_RESETVAL                         (0x00000000U)

/* CMP12_REG1 */

#define CSL_ICSSIEP_CMP12_REG1_CMP12_1_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP12_REG1_CMP12_1_SHIFT                    (0U)
#define CSL_ICSSIEP_CMP12_REG1_CMP12_1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CMP12_REG1_CMP12_1_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CMP12_REG1_RESETVAL                         (0x00000000U)

/* CMP13_REG0 */

#define CSL_ICSSIEP_CMP13_REG0_CMP13_0_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP13_REG0_CMP13_0_SHIFT                    (0U)
#define CSL_ICSSIEP_CMP13_REG0_CMP13_0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CMP13_REG0_CMP13_0_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CMP13_REG0_RESETVAL                         (0x00000000U)

/* CMP13_REG1 */

#define CSL_ICSSIEP_CMP13_REG1_CMP13_1_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP13_REG1_CMP13_1_SHIFT                    (0U)
#define CSL_ICSSIEP_CMP13_REG1_CMP13_1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CMP13_REG1_CMP13_1_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CMP13_REG1_RESETVAL                         (0x00000000U)

/* CMP14_REG0 */

#define CSL_ICSSIEP_CMP14_REG0_CMP14_0_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP14_REG0_CMP14_0_SHIFT                    (0U)
#define CSL_ICSSIEP_CMP14_REG0_CMP14_0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CMP14_REG0_CMP14_0_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CMP14_REG0_RESETVAL                         (0x00000000U)

/* CMP14_REG1 */

#define CSL_ICSSIEP_CMP14_REG1_CMP14_1_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP14_REG1_CMP14_1_SHIFT                    (0U)
#define CSL_ICSSIEP_CMP14_REG1_CMP14_1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CMP14_REG1_CMP14_1_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CMP14_REG1_RESETVAL                         (0x00000000U)

/* CMP15_REG0 */

#define CSL_ICSSIEP_CMP15_REG0_CMP15_0_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP15_REG0_CMP15_0_SHIFT                    (0U)
#define CSL_ICSSIEP_CMP15_REG0_CMP15_0_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CMP15_REG0_CMP15_0_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CMP15_REG0_RESETVAL                         (0x00000000U)

/* CMP15_REG1 */

#define CSL_ICSSIEP_CMP15_REG1_CMP15_1_MASK                     (0xFFFFFFFFU)
#define CSL_ICSSIEP_CMP15_REG1_CMP15_1_SHIFT                    (0U)
#define CSL_ICSSIEP_CMP15_REG1_CMP15_1_RESETVAL                 (0x00000000U)
#define CSL_ICSSIEP_CMP15_REG1_CMP15_1_MAX                      (0xffffffffU)

#define CSL_ICSSIEP_CMP15_REG1_RESETVAL                         (0x00000000U)

/* COUNT_RESET_VAL_REG0 */

#define CSL_ICSSIEP_COUNT_RESET_VAL_REG0_RESET_VAL_0_MASK       (0xFFFFFFFFU)
#define CSL_ICSSIEP_COUNT_RESET_VAL_REG0_RESET_VAL_0_SHIFT      (0U)
#define CSL_ICSSIEP_COUNT_RESET_VAL_REG0_RESET_VAL_0_RESETVAL   (0x00000000U)
#define CSL_ICSSIEP_COUNT_RESET_VAL_REG0_RESET_VAL_0_MAX        (0xffffffffU)

#define CSL_ICSSIEP_COUNT_RESET_VAL_REG0_RESETVAL               (0x00000000U)

/* COUNT_RESET_VAL_REG1 */

#define CSL_ICSSIEP_COUNT_RESET_VAL_REG1_RESET_VAL_1_MASK       (0xFFFFFFFFU)
#define CSL_ICSSIEP_COUNT_RESET_VAL_REG1_RESET_VAL_1_SHIFT      (0U)
#define CSL_ICSSIEP_COUNT_RESET_VAL_REG1_RESET_VAL_1_RESETVAL   (0x00000000U)
#define CSL_ICSSIEP_COUNT_RESET_VAL_REG1_RESET_VAL_1_MAX        (0xffffffffU)

#define CSL_ICSSIEP_COUNT_RESET_VAL_REG1_RESETVAL               (0x00000000U)

/* PWM_REG */

#define CSL_ICSSIEP_PWM_REG_PWM0_RST_CNT_EN_MASK                (0x00000001U)
#define CSL_ICSSIEP_PWM_REG_PWM0_RST_CNT_EN_SHIFT               (0U)
#define CSL_ICSSIEP_PWM_REG_PWM0_RST_CNT_EN_RESETVAL            (0x00000000U)
#define CSL_ICSSIEP_PWM_REG_PWM0_RST_CNT_EN_MAX                 (0x00000001U)

#define CSL_ICSSIEP_PWM_REG_PWM0_HIT_MASK                       (0x00000002U)
#define CSL_ICSSIEP_PWM_REG_PWM0_HIT_SHIFT                      (1U)
#define CSL_ICSSIEP_PWM_REG_PWM0_HIT_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_PWM_REG_PWM0_HIT_MAX                        (0x00000001U)

#define CSL_ICSSIEP_PWM_REG_PWM3_RST_CNT_EN_MASK                (0x00000004U)
#define CSL_ICSSIEP_PWM_REG_PWM3_RST_CNT_EN_SHIFT               (2U)
#define CSL_ICSSIEP_PWM_REG_PWM3_RST_CNT_EN_RESETVAL            (0x00000000U)
#define CSL_ICSSIEP_PWM_REG_PWM3_RST_CNT_EN_MAX                 (0x00000001U)

#define CSL_ICSSIEP_PWM_REG_PWM3_HIT_MASK                       (0x00000008U)
#define CSL_ICSSIEP_PWM_REG_PWM3_HIT_SHIFT                      (3U)
#define CSL_ICSSIEP_PWM_REG_PWM3_HIT_RESETVAL                   (0x00000000U)
#define CSL_ICSSIEP_PWM_REG_PWM3_HIT_MAX                        (0x00000001U)

#define CSL_ICSSIEP_PWM_REG_RESETVAL                            (0x00000000U)

/* SYNC_CTRL_REG */

#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC_EN_MASK                  (0x00000001U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC_EN_SHIFT                 (0U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC_EN_RESETVAL              (0x00000000U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC_EN_MAX                   (0x00000001U)

#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC0_EN_MASK                 (0x00000002U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC0_EN_SHIFT                (1U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC0_EN_RESETVAL             (0x00000000U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC0_EN_MAX                  (0x00000001U)

#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC1_EN_MASK                 (0x00000004U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC1_EN_SHIFT                (2U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC1_EN_RESETVAL             (0x00000000U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC1_EN_MAX                  (0x00000001U)

#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC0_ACK_EN_MASK             (0x00000010U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC0_ACK_EN_SHIFT            (4U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC0_ACK_EN_RESETVAL         (0x00000000U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC0_ACK_EN_MAX              (0x00000001U)

#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC0_CYCLIC_EN_MASK          (0x00000020U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC0_CYCLIC_EN_SHIFT         (5U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC0_CYCLIC_EN_RESETVAL      (0x00000000U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC0_CYCLIC_EN_MAX           (0x00000001U)

#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC1_ACK_EN_MASK             (0x00000040U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC1_ACK_EN_SHIFT            (6U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC1_ACK_EN_RESETVAL         (0x00000000U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC1_ACK_EN_MAX              (0x00000001U)

#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC1_CYCLIC_EN_MASK          (0x00000080U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC1_CYCLIC_EN_SHIFT         (7U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC1_CYCLIC_EN_RESETVAL      (0x00000000U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC1_CYCLIC_EN_MAX           (0x00000001U)

#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC1_IND_EN_MASK             (0x00000100U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC1_IND_EN_SHIFT            (8U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC1_IND_EN_RESETVAL         (0x00000000U)
#define CSL_ICSSIEP_SYNC_CTRL_REG_SYNC1_IND_EN_MAX              (0x00000001U)

#define CSL_ICSSIEP_SYNC_CTRL_REG_RESETVAL                      (0x00000000U)

/* SYNC_FIRST_STAT_REG */

#define CSL_ICSSIEP_SYNC_FIRST_STAT_REG_FIRST_SYNC0_MASK        (0x00000001U)
#define CSL_ICSSIEP_SYNC_FIRST_STAT_REG_FIRST_SYNC0_SHIFT       (0U)
#define CSL_ICSSIEP_SYNC_FIRST_STAT_REG_FIRST_SYNC0_RESETVAL    (0x00000000U)
#define CSL_ICSSIEP_SYNC_FIRST_STAT_REG_FIRST_SYNC0_MAX         (0x00000001U)

#define CSL_ICSSIEP_SYNC_FIRST_STAT_REG_FIRST_SYNC1_MASK        (0x00000002U)
#define CSL_ICSSIEP_SYNC_FIRST_STAT_REG_FIRST_SYNC1_SHIFT       (1U)
#define CSL_ICSSIEP_SYNC_FIRST_STAT_REG_FIRST_SYNC1_RESETVAL    (0x00000000U)
#define CSL_ICSSIEP_SYNC_FIRST_STAT_REG_FIRST_SYNC1_MAX         (0x00000001U)

#define CSL_ICSSIEP_SYNC_FIRST_STAT_REG_RESETVAL                (0x00000000U)

/* SYNC0_STAT_REG */

#define CSL_ICSSIEP_SYNC0_STAT_REG_SYNC0_PEND_MASK              (0x00000001U)
#define CSL_ICSSIEP_SYNC0_STAT_REG_SYNC0_PEND_SHIFT             (0U)
#define CSL_ICSSIEP_SYNC0_STAT_REG_SYNC0_PEND_RESETVAL          (0x00000000U)
#define CSL_ICSSIEP_SYNC0_STAT_REG_SYNC0_PEND_MAX               (0x00000001U)

#define CSL_ICSSIEP_SYNC0_STAT_REG_RESETVAL                     (0x00000000U)

/* SYNC1_STAT_REG */

#define CSL_ICSSIEP_SYNC1_STAT_REG_SYNC1_PEND_MASK              (0x00000001U)
#define CSL_ICSSIEP_SYNC1_STAT_REG_SYNC1_PEND_SHIFT             (0U)
#define CSL_ICSSIEP_SYNC1_STAT_REG_SYNC1_PEND_RESETVAL          (0x00000000U)
#define CSL_ICSSIEP_SYNC1_STAT_REG_SYNC1_PEND_MAX               (0x00000001U)

#define CSL_ICSSIEP_SYNC1_STAT_REG_RESETVAL                     (0x00000000U)

/* SYNC_PWIDTH_REG */

#define CSL_ICSSIEP_SYNC_PWIDTH_REG_SYNC_HPW_MASK               (0xFFFFFFFFU)
#define CSL_ICSSIEP_SYNC_PWIDTH_REG_SYNC_HPW_SHIFT              (0U)
#define CSL_ICSSIEP_SYNC_PWIDTH_REG_SYNC_HPW_RESETVAL           (0x00000000U)
#define CSL_ICSSIEP_SYNC_PWIDTH_REG_SYNC_HPW_MAX                (0xffffffffU)

#define CSL_ICSSIEP_SYNC_PWIDTH_REG_RESETVAL                    (0x00000000U)

/* SYNC0_PERIOD_REG */

#define CSL_ICSSIEP_SYNC0_PERIOD_REG_SYNC0_PERIOD_MASK          (0xFFFFFFFFU)
#define CSL_ICSSIEP_SYNC0_PERIOD_REG_SYNC0_PERIOD_SHIFT         (0U)
#define CSL_ICSSIEP_SYNC0_PERIOD_REG_SYNC0_PERIOD_RESETVAL      (0x00000000U)
#define CSL_ICSSIEP_SYNC0_PERIOD_REG_SYNC0_PERIOD_MAX           (0xffffffffU)

#define CSL_ICSSIEP_SYNC0_PERIOD_REG_RESETVAL                   (0x00000000U)

/* SYNC1_DELAY_REG */

#define CSL_ICSSIEP_SYNC1_DELAY_REG_SYNC1_DELAY_MASK            (0xFFFFFFFFU)
#define CSL_ICSSIEP_SYNC1_DELAY_REG_SYNC1_DELAY_SHIFT           (0U)
#define CSL_ICSSIEP_SYNC1_DELAY_REG_SYNC1_DELAY_RESETVAL        (0x00000000U)
#define CSL_ICSSIEP_SYNC1_DELAY_REG_SYNC1_DELAY_MAX             (0xffffffffU)

#define CSL_ICSSIEP_SYNC1_DELAY_REG_RESETVAL                    (0x00000000U)

/* SYNC_START_REG */

#define CSL_ICSSIEP_SYNC_START_REG_SYNC_START_MASK              (0xFFFFFFFFU)
#define CSL_ICSSIEP_SYNC_START_REG_SYNC_START_SHIFT             (0U)
#define CSL_ICSSIEP_SYNC_START_REG_SYNC_START_RESETVAL          (0x00000000U)
#define CSL_ICSSIEP_SYNC_START_REG_SYNC_START_MAX               (0xffffffffU)

#define CSL_ICSSIEP_SYNC_START_REG_RESETVAL                     (0x00000000U)

/* WD_PREDIV_REG */

#define CSL_ICSSIEP_WD_PREDIV_REG_PRE_DIV_MASK                  (0x0000FFFFU)
#define CSL_ICSSIEP_WD_PREDIV_REG_PRE_DIV_SHIFT                 (0U)
#define CSL_ICSSIEP_WD_PREDIV_REG_PRE_DIV_RESETVAL              (0x00000000U)
#define CSL_ICSSIEP_WD_PREDIV_REG_PRE_DIV_MAX                   (0x0000ffffU)

#define CSL_ICSSIEP_WD_PREDIV_REG_RESETVAL                      (0x00000000U)

/* PDI_WD_TIM_REG */

#define CSL_ICSSIEP_PDI_WD_TIM_REG_PDI_WD_TIME_MASK             (0x0000FFFFU)
#define CSL_ICSSIEP_PDI_WD_TIM_REG_PDI_WD_TIME_SHIFT            (0U)
#define CSL_ICSSIEP_PDI_WD_TIM_REG_PDI_WD_TIME_RESETVAL         (0x00000000U)
#define CSL_ICSSIEP_PDI_WD_TIM_REG_PDI_WD_TIME_MAX              (0x0000ffffU)

#define CSL_ICSSIEP_PDI_WD_TIM_REG_RESETVAL                     (0x00000000U)

/* PD_WD_TIM_REG */

#define CSL_ICSSIEP_PD_WD_TIM_REG_PD_WD_TIME_MASK               (0x0000FFFFU)
#define CSL_ICSSIEP_PD_WD_TIM_REG_PD_WD_TIME_SHIFT              (0U)
#define CSL_ICSSIEP_PD_WD_TIM_REG_PD_WD_TIME_RESETVAL           (0x00000000U)
#define CSL_ICSSIEP_PD_WD_TIM_REG_PD_WD_TIME_MAX                (0x0000ffffU)

#define CSL_ICSSIEP_PD_WD_TIM_REG_RESETVAL                      (0x00000000U)

/* WD_STATUS_REG */

#define CSL_ICSSIEP_WD_STATUS_REG_PD_WD_STAT_MASK               (0x00000001U)
#define CSL_ICSSIEP_WD_STATUS_REG_PD_WD_STAT_SHIFT              (0U)
#define CSL_ICSSIEP_WD_STATUS_REG_PD_WD_STAT_RESETVAL           (0x00000000U)
#define CSL_ICSSIEP_WD_STATUS_REG_PD_WD_STAT_MAX                (0x00000001U)

#define CSL_ICSSIEP_WD_STATUS_REG_PDI_WD_STAT_MASK              (0x00010000U)
#define CSL_ICSSIEP_WD_STATUS_REG_PDI_WD_STAT_SHIFT             (16U)
#define CSL_ICSSIEP_WD_STATUS_REG_PDI_WD_STAT_RESETVAL          (0x00000000U)
#define CSL_ICSSIEP_WD_STATUS_REG_PDI_WD_STAT_MAX               (0x00000001U)

#define CSL_ICSSIEP_WD_STATUS_REG_RESETVAL                      (0x00000000U)

/* WD_EXP_CNT_REG */

#define CSL_ICSSIEP_WD_EXP_CNT_REG_PDI_EXP_CNT_MASK             (0x000000FFU)
#define CSL_ICSSIEP_WD_EXP_CNT_REG_PDI_EXP_CNT_SHIFT            (0U)
#define CSL_ICSSIEP_WD_EXP_CNT_REG_PDI_EXP_CNT_RESETVAL         (0x00000000U)
#define CSL_ICSSIEP_WD_EXP_CNT_REG_PDI_EXP_CNT_MAX              (0x000000ffU)

#define CSL_ICSSIEP_WD_EXP_CNT_REG_PD_EXP_CNT_MASK              (0x0000FF00U)
#define CSL_ICSSIEP_WD_EXP_CNT_REG_PD_EXP_CNT_SHIFT             (8U)
#define CSL_ICSSIEP_WD_EXP_CNT_REG_PD_EXP_CNT_RESETVAL          (0x00000000U)
#define CSL_ICSSIEP_WD_EXP_CNT_REG_PD_EXP_CNT_MAX               (0x000000ffU)

#define CSL_ICSSIEP_WD_EXP_CNT_REG_RESETVAL                     (0x00000000U)

/* WD_CTRL_REG */

#define CSL_ICSSIEP_WD_CTRL_REG_PD_WD_EN_MASK                   (0x00000001U)
#define CSL_ICSSIEP_WD_CTRL_REG_PD_WD_EN_SHIFT                  (0U)
#define CSL_ICSSIEP_WD_CTRL_REG_PD_WD_EN_RESETVAL               (0x00000000U)
#define CSL_ICSSIEP_WD_CTRL_REG_PD_WD_EN_MAX                    (0x00000001U)

#define CSL_ICSSIEP_WD_CTRL_REG_PDI_WD_EN_MASK                  (0x00010000U)
#define CSL_ICSSIEP_WD_CTRL_REG_PDI_WD_EN_SHIFT                 (16U)
#define CSL_ICSSIEP_WD_CTRL_REG_PDI_WD_EN_RESETVAL              (0x00000000U)
#define CSL_ICSSIEP_WD_CTRL_REG_PDI_WD_EN_MAX                   (0x00000001U)

#define CSL_ICSSIEP_WD_CTRL_REG_RESETVAL                        (0x00000000U)

/* DIGIO_CTRL_REG */

#define CSL_ICSSIEP_DIGIO_CTRL_REG_OUTVALID_POL_MASK            (0x00000001U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_OUTVALID_POL_SHIFT           (0U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_OUTVALID_POL_RESETVAL        (0x00000000U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_OUTVALID_POL_MAX             (0x00000001U)

#define CSL_ICSSIEP_DIGIO_CTRL_REG_OUTVALID_MODE_MASK           (0x00000002U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_OUTVALID_MODE_SHIFT          (1U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_OUTVALID_MODE_RESETVAL       (0x00000000U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_OUTVALID_MODE_MAX            (0x00000001U)

#define CSL_ICSSIEP_DIGIO_CTRL_REG_BIDI_MODE_MASK               (0x00000004U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_BIDI_MODE_SHIFT              (2U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_BIDI_MODE_RESETVAL           (0x00000000U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_BIDI_MODE_MAX                (0x00000001U)

#define CSL_ICSSIEP_DIGIO_CTRL_REG_WD_MODE_MASK                 (0x00000008U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_WD_MODE_SHIFT                (3U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_WD_MODE_RESETVAL             (0x00000000U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_WD_MODE_MAX                  (0x00000001U)

#define CSL_ICSSIEP_DIGIO_CTRL_REG_IN_MODE_MASK                 (0x00000030U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_IN_MODE_SHIFT                (4U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_IN_MODE_RESETVAL             (0x00000000U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_IN_MODE_MAX                  (0x00000003U)

#define CSL_ICSSIEP_DIGIO_CTRL_REG_OUT_MODE_MASK                (0x000000C0U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_OUT_MODE_SHIFT               (6U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_OUT_MODE_RESETVAL            (0x00000000U)
#define CSL_ICSSIEP_DIGIO_CTRL_REG_OUT_MODE_MAX                 (0x00000003U)

#define CSL_ICSSIEP_DIGIO_CTRL_REG_RESETVAL                     (0x00000000U)

/* DIGIO_STATUS_REG */

#define CSL_ICSSIEP_DIGIO_STATUS_REG_DIGIO_STAT_MASK            (0xFFFFFFFFU)
#define CSL_ICSSIEP_DIGIO_STATUS_REG_DIGIO_STAT_SHIFT           (0U)
#define CSL_ICSSIEP_DIGIO_STATUS_REG_DIGIO_STAT_RESETVAL        (0x00000000U)
#define CSL_ICSSIEP_DIGIO_STATUS_REG_DIGIO_STAT_MAX             (0xffffffffU)

#define CSL_ICSSIEP_DIGIO_STATUS_REG_RESETVAL                   (0x00000000U)

/* DIGIO_DATA_IN_REG */

#define CSL_ICSSIEP_DIGIO_DATA_IN_REG_DATA_IN_MASK              (0xFFFFFFFFU)
#define CSL_ICSSIEP_DIGIO_DATA_IN_REG_DATA_IN_SHIFT             (0U)
#define CSL_ICSSIEP_DIGIO_DATA_IN_REG_DATA_IN_RESETVAL          (0x00000000U)
#define CSL_ICSSIEP_DIGIO_DATA_IN_REG_DATA_IN_MAX               (0xffffffffU)

#define CSL_ICSSIEP_DIGIO_DATA_IN_REG_RESETVAL                  (0x00000000U)

/* DIGIO_DATA_IN_RAW_REG */

#define CSL_ICSSIEP_DIGIO_DATA_IN_RAW_REG_DATA_IN_RAW_MASK      (0xFFFFFFFFU)
#define CSL_ICSSIEP_DIGIO_DATA_IN_RAW_REG_DATA_IN_RAW_SHIFT     (0U)
#define CSL_ICSSIEP_DIGIO_DATA_IN_RAW_REG_DATA_IN_RAW_RESETVAL  (0x00000000U)
#define CSL_ICSSIEP_DIGIO_DATA_IN_RAW_REG_DATA_IN_RAW_MAX       (0xffffffffU)

#define CSL_ICSSIEP_DIGIO_DATA_IN_RAW_REG_RESETVAL              (0x00000000U)

/* DIGIO_DATA_OUT_REG */

#define CSL_ICSSIEP_DIGIO_DATA_OUT_REG_DATA_OUT_MASK            (0xFFFFFFFFU)
#define CSL_ICSSIEP_DIGIO_DATA_OUT_REG_DATA_OUT_SHIFT           (0U)
#define CSL_ICSSIEP_DIGIO_DATA_OUT_REG_DATA_OUT_RESETVAL        (0x00000000U)
#define CSL_ICSSIEP_DIGIO_DATA_OUT_REG_DATA_OUT_MAX             (0xffffffffU)

#define CSL_ICSSIEP_DIGIO_DATA_OUT_REG_RESETVAL                 (0x00000000U)

/* DIGIO_DATA_OUT_EN_REG */

#define CSL_ICSSIEP_DIGIO_DATA_OUT_EN_REG_DATA_OUT_EN_MASK      (0xFFFFFFFFU)
#define CSL_ICSSIEP_DIGIO_DATA_OUT_EN_REG_DATA_OUT_EN_SHIFT     (0U)
#define CSL_ICSSIEP_DIGIO_DATA_OUT_EN_REG_DATA_OUT_EN_RESETVAL  (0x00000000U)
#define CSL_ICSSIEP_DIGIO_DATA_OUT_EN_REG_DATA_OUT_EN_MAX       (0xffffffffU)

#define CSL_ICSSIEP_DIGIO_DATA_OUT_EN_REG_RESETVAL              (0x00000000U)

/* DIGIO_EXP_REG */

#define CSL_ICSSIEP_DIGIO_EXP_REG_SW_DATA_OUT_UP_MASK           (0x00000001U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_SW_DATA_OUT_UP_SHIFT          (0U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_SW_DATA_OUT_UP_RESETVAL       (0x00000000U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_SW_DATA_OUT_UP_MAX            (0x00000001U)

#define CSL_ICSSIEP_DIGIO_EXP_REG_OUTVALID_OVR_EN_MASK          (0x00000002U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_OUTVALID_OVR_EN_SHIFT         (1U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_OUTVALID_OVR_EN_RESETVAL      (0x00000000U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_OUTVALID_OVR_EN_MAX           (0x00000001U)

#define CSL_ICSSIEP_DIGIO_EXP_REG_SW_OUTVALID_MASK              (0x00000004U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_SW_OUTVALID_SHIFT             (2U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_SW_OUTVALID_RESETVAL          (0x00000000U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_SW_OUTVALID_MAX               (0x00000001U)

#define CSL_ICSSIEP_DIGIO_EXP_REG_OUTVALID_DLY_MASK             (0x000000F0U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_OUTVALID_DLY_SHIFT            (4U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_OUTVALID_DLY_RESETVAL         (0x00000000U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_OUTVALID_DLY_MAX              (0x0000000fU)

#define CSL_ICSSIEP_DIGIO_EXP_REG_SOF_DLY_MASK                  (0x00000F00U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_SOF_DLY_SHIFT                 (8U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_SOF_DLY_RESETVAL              (0x00000000U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_SOF_DLY_MAX                   (0x0000000fU)

#define CSL_ICSSIEP_DIGIO_EXP_REG_SOF_SEL_MASK                  (0x00001000U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_SOF_SEL_SHIFT                 (12U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_SOF_SEL_RESETVAL              (0x00000000U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_SOF_SEL_MAX                   (0x00000001U)

#define CSL_ICSSIEP_DIGIO_EXP_REG_EOF_SEL_MASK                  (0x00002000U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_EOF_SEL_SHIFT                 (13U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_EOF_SEL_RESETVAL              (0x00000000U)
#define CSL_ICSSIEP_DIGIO_EXP_REG_EOF_SEL_MAX                   (0x00000001U)

#define CSL_ICSSIEP_DIGIO_EXP_REG_RESETVAL                      (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
