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
 *  Name        : cslr_mcu_pll_mmr.h
*/
#ifndef CSLR_MCU_PLL_MMR_H_
#define CSLR_MCU_PLL_MMR_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Module Base Offset Values
**************************************************************************/

#define CSL_MCU_PLL_MMR_CFG_REGS_BASE                                     (0x00000000U)


/**************************************************************************
* Hardware Region  : PLLCTRL MMRs
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/

typedef struct {
    volatile uint32_t PLL0_PID;                  /* Peripheral Identification Register */
    volatile uint8_t  Resv_8[4];
    volatile uint32_t PLL0_CFG;                  /* pll MMR Configuration */
    volatile uint8_t  Resv_16[4];
    volatile uint32_t PLL0_LOCKKEY0;             /* PLL0_LOCKKEY0 - PLL0 Lock Key 0 Register */
    volatile uint32_t PLL0_LOCKKEY1;             /* PLL0_LOCKKEY1 - PLL0 Lock Key 1 RegisterAddr */
    volatile uint8_t  Resv_32[8];
    volatile uint32_t PLL0_CTRL;                 /* PLL0_CTRL - PLL0 Control */
    volatile uint32_t PLL0_STAT;                 /* PLL0_STAT - PLL0 Status */
    volatile uint8_t  Resv_48[8];
    volatile uint32_t PLL0_FREQ_CTRL0;           /* PLL0_FREQ_CTRL0 - PLL0 Frequency Control 0 Register */
    volatile uint32_t PLL0_FREQ_CTRL1;           /* PLL0_FREQ_CTRL1 - PLL0 Frequency Control 1 Register */
    volatile uint32_t PLL0_DIV_CTRL;             /* PLL0_DIV_CTRL - PLL0 Output Clock Divider Register */
    volatile uint8_t  Resv_64[4];
    volatile uint32_t PLL0_SS_CTRL;              /* PLL_SS_CTRL register for pll0 */
    volatile uint32_t PLL0_SS_SPREAD;            /* PLL_SS_SPREAD register for pll0 */
    volatile uint8_t  Resv_96[24];
    volatile uint32_t PLL0_CAL_CTRL;             /* PLL0_CAL_CTRL - PLL0 Calibration Control Register */
    volatile uint32_t PLL0_CAL_STAT;             /* PLL0_CAL_STAT - PLL0 Calibration Status Register */
    volatile uint8_t  Resv_128[24];
    volatile uint32_t PLL0_HSDIV_CTRL0;          /* HSDIV_CTRL0 register for pll0 */
    volatile uint32_t PLL0_HSDIV_CTRL1;          /* HSDIV_CTRL1 register for pll0 */
    volatile uint32_t PLL0_HSDIV_CTRL2;          /* HSDIV_CTRL2 register for pll0 */
    volatile uint32_t PLL0_HSDIV_CTRL3;          /* HSDIV_CTRL3 register for pll0 */
    volatile uint32_t PLL0_HSDIV_CTRL4;          /* HSDIV_CTRL4 register for pll0 */
} CSL_mcu_pll_mmr_cfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_MCU_PLL_MMR_CFG_PLL0_PID                                      (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CFG                                      (0x00000008U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_LOCKKEY0                                 (0x00000010U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_LOCKKEY1                                 (0x00000014U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL                                     (0x00000020U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_STAT                                     (0x00000024U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_FREQ_CTRL0                               (0x00000030U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_FREQ_CTRL1                               (0x00000034U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_DIV_CTRL                                 (0x00000038U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_CTRL                                  (0x00000040U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_SPREAD                                (0x00000044U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_CTRL                                 (0x00000060U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_STAT                                 (0x00000064U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL0                              (0x00000080U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL1                              (0x00000084U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL2                              (0x00000088U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL3                              (0x0000008CU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL4                              (0x00000090U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PLL0_PID */

#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_MINOR_MASK                           (0x0000003FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_MINOR_SHIFT                          (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_MINOR_MAX                            (0x0000003FU)

#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_CUSTOM_MASK                          (0x000000C0U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_CUSTOM_SHIFT                         (0x00000006U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_CUSTOM_MAX                           (0x00000003U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_MAJOR_MASK                           (0x00000700U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_MAJOR_SHIFT                          (0x00000008U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_MAJOR_MAX                            (0x00000007U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_MISC_MASK                            (0x0000F800U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_MISC_SHIFT                           (0x0000000BU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_MISC_MAX                             (0x0000001FU)

#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_MODULE_MASK                          (0x0FFF0000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_MODULE_SHIFT                         (0x00000010U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_MODULE_MAX                           (0x00000FFFU)

#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_BU_MASK                              (0x30000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_BU_SHIFT                             (0x0000001CU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_BU_MAX                               (0x00000003U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_SCHEME_MASK                          (0xC0000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_SCHEME_SHIFT                         (0x0000001EU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_PID_SCHEME_MAX                           (0x00000003U)

/* PLL0_CFG */

#define CSL_MCU_PLL_MMR_CFG_PLL0_CFG_PLL_TYPE_MASK                        (0x00000003U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CFG_PLL_TYPE_SHIFT                       (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CFG_PLL_TYPE_MAX                         (0x00000003U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_CFG_SSM_WVTBL_MASK                       (0x00000100U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CFG_SSM_WVTBL_SHIFT                      (0x00000008U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CFG_SSM_WVTBL_MAX                        (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_CFG_SSM_TYPE_MASK                        (0x00001800U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CFG_SSM_TYPE_SHIFT                       (0x0000000BU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CFG_SSM_TYPE_MAX                         (0x00000003U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_CFG_HSDIV_PRSNT_MASK                     (0xFFFF0000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CFG_HSDIV_PRSNT_SHIFT                    (0x00000010U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CFG_HSDIV_PRSNT_MAX                      (0x0000FFFFU)

/* PLL0_LOCKKEY0 */

#define CSL_MCU_PLL_MMR_CFG_PLL0_LOCKKEY0_KEY_MASK                        (0xFFFFFFFEU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_LOCKKEY0_KEY_SHIFT                       (0x00000001U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_LOCKKEY0_KEY_MAX                         (0x7FFFFFFFU)

#define CSL_MCU_PLL_MMR_CFG_PLL0_LOCKKEY0_UNLOCKED_MASK                   (0x00000001U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_LOCKKEY0_UNLOCKED_SHIFT                  (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_LOCKKEY0_UNLOCKED_MAX                    (0x00000001U)

/* PLL0_LOCKKEY1 */

#define CSL_MCU_PLL_MMR_CFG_PLL0_LOCKKEY1_LOCKKEY1_VAL_MASK               (0xFFFFFFFFU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_LOCKKEY1_LOCKKEY1_VAL_SHIFT              (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_LOCKKEY1_LOCKKEY1_VAL_MAX                (0xFFFFFFFFU)

/* PLL0_CTRL */

#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_DAC_EN_MASK                         (0x00000001U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_DAC_EN_SHIFT                        (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_DAC_EN_MAX                          (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_DSM_EN_MASK                         (0x00000002U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_DSM_EN_SHIFT                        (0x00000001U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_DSM_EN_MAX                          (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_CLK_POSTDIV_EN_MASK                 (0x00000010U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_CLK_POSTDIV_EN_SHIFT                (0x00000004U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_CLK_POSTDIV_EN_MAX                  (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_CLK_4PH_EN_MASK                     (0x00000020U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_CLK_4PH_EN_SHIFT                    (0x00000005U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_CLK_4PH_EN_MAX                      (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_INTL_BYP_EN_MASK                    (0x00000100U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_INTL_BYP_EN_SHIFT                   (0x00000008U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_INTL_BYP_EN_MAX                     (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_PLL_EN_MASK                         (0x00008000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_PLL_EN_SHIFT                        (0x0000000FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_PLL_EN_MAX                          (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_BYP_ON_LOCKLOSS_MASK                (0x00010000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_BYP_ON_LOCKLOSS_SHIFT               (0x00000010U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_BYP_ON_LOCKLOSS_MAX                 (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_BYPASS_EN_MASK                      (0x80000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_BYPASS_EN_SHIFT                     (0x0000001FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CTRL_BYPASS_EN_MAX                       (0x00000001U)

/* PLL0_STAT */

#define CSL_MCU_PLL_MMR_CFG_PLL0_STAT_LOCK_MASK                           (0x00000001U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_STAT_LOCK_SHIFT                          (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_STAT_LOCK_MAX                            (0x00000001U)

/* PLL0_FREQ_CTRL0 */

#define CSL_MCU_PLL_MMR_CFG_PLL0_FREQ_CTRL0_FB_DIV_INT_MASK               (0x00000FFFU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_FREQ_CTRL0_FB_DIV_INT_SHIFT              (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_FREQ_CTRL0_FB_DIV_INT_MAX                (0x00000FFFU)

/* PLL0_FREQ_CTRL1 */

#define CSL_MCU_PLL_MMR_CFG_PLL0_FREQ_CTRL1_FB_DIV_FRAC_MASK              (0x00FFFFFFU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_FREQ_CTRL1_FB_DIV_FRAC_SHIFT             (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_FREQ_CTRL1_FB_DIV_FRAC_MAX               (0x00FFFFFFU)

/* PLL0_DIV_CTRL */

#define CSL_MCU_PLL_MMR_CFG_PLL0_DIV_CTRL_REF_DIV_MASK                    (0x0000003FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_DIV_CTRL_REF_DIV_SHIFT                   (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_DIV_CTRL_REF_DIV_MAX                     (0x0000003FU)

#define CSL_MCU_PLL_MMR_CFG_PLL0_DIV_CTRL_POST_DIV1_MASK                  (0x00070000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_DIV_CTRL_POST_DIV1_SHIFT                 (0x00000010U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_DIV_CTRL_POST_DIV1_MAX                   (0x00000007U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_DIV_CTRL_POST_DIV2_MASK                  (0x07000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_DIV_CTRL_POST_DIV2_SHIFT                 (0x00000018U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_DIV_CTRL_POST_DIV2_MAX                   (0x00000007U)

/* PLL0_SS_CTRL */

#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_CTRL_WAVE_SEL_MASK                    (0x00000001U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_CTRL_WAVE_SEL_SHIFT                   (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_CTRL_WAVE_SEL_MAX                     (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_CTRL_DOWNSPREAD_EN_MASK               (0x00000010U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_CTRL_DOWNSPREAD_EN_SHIFT              (0x00000004U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_CTRL_DOWNSPREAD_EN_MAX                (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_CTRL_RESET_MASK                       (0x00008000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_CTRL_RESET_SHIFT                      (0x0000000FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_CTRL_RESET_MAX                        (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_CTRL_WV_TBLE_MAXADDR_MASK             (0x03FC0000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_CTRL_WV_TBLE_MAXADDR_SHIFT            (0x00000012U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_CTRL_WV_TBLE_MAXADDR_MAX              (0x000000FFU)

#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_CTRL_BYPASS_EN_MASK                   (0x80000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_CTRL_BYPASS_EN_SHIFT                  (0x0000001FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_CTRL_BYPASS_EN_MAX                    (0x00000001U)

/* PLL0_SS_SPREAD */

#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_SPREAD_SPREAD_MASK                    (0x0000001FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_SPREAD_SPREAD_SHIFT                   (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_SPREAD_SPREAD_MAX                     (0x0000001FU)

#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_SPREAD_MOD_DIV_MASK                   (0x000F0000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_SPREAD_MOD_DIV_SHIFT                  (0x00000010U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_SS_SPREAD_MOD_DIV_MAX                    (0x0000000FU)

/* PLL0_CAL_CTRL */

#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_IN_MASK                     (0x00000FFFU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_IN_SHIFT                    (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_IN_MAX                      (0x00000FFFU)

#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_BYP_MASK                    (0x00008000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_BYP_SHIFT                   (0x0000000FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_BYP_MAX                     (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_CNT_MASK                    (0x00070000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_CNT_SHIFT                   (0x00000010U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_CNT_MAX                     (0x00000007U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_CTRL_FAST_CAL_MASK                   (0x00100000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_CTRL_FAST_CAL_SHIFT                  (0x00000014U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_CTRL_FAST_CAL_MAX                    (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_EN_MASK                     (0x80000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_EN_SHIFT                    (0x0000001FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_EN_MAX                      (0x00000001U)

/* PLL0_CAL_STAT */

#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_STAT_CAL_OUT_MASK                    (0x00000FFFU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_STAT_CAL_OUT_SHIFT                   (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_STAT_CAL_OUT_MAX                     (0x00000FFFU)

#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_STAT_LOCK_CNT_MASK                   (0x000F0000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_STAT_LOCK_CNT_SHIFT                  (0x00000010U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_STAT_LOCK_CNT_MAX                    (0x0000000FU)

#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_STAT_CAL_LOCK_MASK                   (0x80000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_STAT_CAL_LOCK_SHIFT                  (0x0000001FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_CAL_STAT_CAL_LOCK_MAX                    (0x00000001U)

/* PLL0_HSDIV_CTRL0 */

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_HSDIV_MASK                   (0x0000007FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_HSDIV_SHIFT                  (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_HSDIV_MAX                    (0x0000007FU)

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_SYNC_DIS_MASK                (0x00000100U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_SYNC_DIS_SHIFT               (0x00000008U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_SYNC_DIS_MAX                 (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_CLKOUT_EN_MASK               (0x00008000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_CLKOUT_EN_SHIFT              (0x0000000FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_CLKOUT_EN_MAX                (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_RESET_MASK                   (0x80000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_RESET_SHIFT                  (0x0000001FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_RESET_MAX                    (0x00000001U)

/* PLL0_HSDIV_CTRL1 */

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_HSDIV_MASK                   (0x0000007FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_HSDIV_SHIFT                  (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_HSDIV_MAX                    (0x0000007FU)

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_SYNC_DIS_MASK                (0x00000100U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_SYNC_DIS_SHIFT               (0x00000008U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_SYNC_DIS_MAX                 (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_CLKOUT_EN_MASK               (0x00008000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_CLKOUT_EN_SHIFT              (0x0000000FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_CLKOUT_EN_MAX                (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_RESET_MASK                   (0x80000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_RESET_SHIFT                  (0x0000001FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_RESET_MAX                    (0x00000001U)

/* PLL0_HSDIV_CTRL2 */

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_HSDIV_MASK                   (0x0000007FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_HSDIV_SHIFT                  (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_HSDIV_MAX                    (0x0000007FU)

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_SYNC_DIS_MASK                (0x00000100U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_SYNC_DIS_SHIFT               (0x00000008U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_SYNC_DIS_MAX                 (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_CLKOUT_EN_MASK               (0x00008000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_CLKOUT_EN_SHIFT              (0x0000000FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_CLKOUT_EN_MAX                (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_RESET_MASK                   (0x80000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_RESET_SHIFT                  (0x0000001FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_RESET_MAX                    (0x00000001U)

/* PLL0_HSDIV_CTRL3 */

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_HSDIV_MASK                   (0x0000007FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_HSDIV_SHIFT                  (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_HSDIV_MAX                    (0x0000007FU)

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_SYNC_DIS_MASK                (0x00000100U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_SYNC_DIS_SHIFT               (0x00000008U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_SYNC_DIS_MAX                 (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_CLKOUT_EN_MASK               (0x00008000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_CLKOUT_EN_SHIFT              (0x0000000FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_CLKOUT_EN_MAX                (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_RESET_MASK                   (0x80000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_RESET_SHIFT                  (0x0000001FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_RESET_MAX                    (0x00000001U)

/* PLL0_HSDIV_CTRL4 */

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_HSDIV_MASK                   (0x0000007FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_HSDIV_SHIFT                  (0x00000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_HSDIV_MAX                    (0x0000007FU)

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_SYNC_DIS_MASK                (0x00000100U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_SYNC_DIS_SHIFT               (0x00000008U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_SYNC_DIS_MAX                 (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_CLKOUT_EN_MASK               (0x00008000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_CLKOUT_EN_SHIFT              (0x0000000FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_CLKOUT_EN_MAX                (0x00000001U)

#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_RESET_MASK                   (0x80000000U)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_RESET_SHIFT                  (0x0000001FU)
#define CSL_MCU_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_RESET_MAX                    (0x00000001U)

#ifdef __cplusplus
}
#endif
#endif
