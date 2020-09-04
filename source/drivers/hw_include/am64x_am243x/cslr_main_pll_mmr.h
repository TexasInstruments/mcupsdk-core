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
 *  Name        : cslr_main_pll_mmr.h
*/
#ifndef CSLR_MAIN_PLL_MMR_H_
#define CSLR_MAIN_PLL_MMR_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <drivers/hw_include/cslr.h>
#include <stdint.h>

/**************************************************************************
* Module Base Offset Values
**************************************************************************/

#define CSL_MAIN_PLL_MMR_CFG_REGS_BASE                                    (0x00000000U)


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
    volatile uint32_t PLL0_HSDIV_CTRL5;          /* HSDIV_CTRL5 register for pll0 */
    volatile uint32_t PLL0_HSDIV_CTRL6;          /* HSDIV_CTRL6 register for pll0 */
    volatile uint32_t PLL0_HSDIV_CTRL7;          /* HSDIV_CTRL7 register for pll0 */
    volatile uint32_t PLL0_HSDIV_CTRL8;          /* HSDIV_CTRL8 register for pll0 */
    volatile uint32_t PLL0_HSDIV_CTRL9;          /* HSDIV_CTRL9 register for pll0 */
    volatile uint8_t  Resv_4096[3928];
    volatile uint32_t PLL1_PID;                  /* Peripheral Identification Register */
    volatile uint8_t  Resv_4104[4];
    volatile uint32_t PLL1_CFG;                  /* pll MMR Configuration */
    volatile uint8_t  Resv_4112[4];
    volatile uint32_t PLL1_LOCKKEY0;             /* PLL1_LOCKKEY0 - PLL1 Lock Key 0 Register */
    volatile uint32_t PLL1_LOCKKEY1;             /* PLL1_LOCKKEY1 - PLL1 Lock Key 1 RegisterAddr */
    volatile uint8_t  Resv_4128[8];
    volatile uint32_t PLL1_CTRL;                 /* PLL1_CTRL - PLL1 Control */
    volatile uint32_t PLL1_STAT;                 /* PLL1_STAT - PLL1 Status */
    volatile uint8_t  Resv_4144[8];
    volatile uint32_t PLL1_FREQ_CTRL0;           /* PLL0_FREQ_CTRL0 - PLL1 Frequency Control 1 Register */
    volatile uint32_t PLL1_FREQ_CTRL1;           /* PLL1_FREQ_CTRL1 - PLL0 Frequency Control 1 Register */
    volatile uint32_t PLL1_DIV_CTRL;             /* PLL1_DIV_CTRL - PLL1 Output Clock Divider Register */
    volatile uint8_t  Resv_4160[4];
    volatile uint32_t PLL1_SS_CTRL;              /* PLL_SS_CTRL register for pll1 */
    volatile uint32_t PLL1_SS_SPREAD;            /* PLL_SS_SPREAD register for pll1 */
    volatile uint8_t  Resv_4192[24];
    volatile uint32_t PLL1_CAL_CTRL;             /* PLL1_CAL_CTRL - PLL1 Calibration Control Register */
    volatile uint32_t PLL1_CAL_STAT;             /* PLL1_CAL_STAT - PLL1 Calibration Status Register */
    volatile uint8_t  Resv_4224[24];
    volatile uint32_t PLL1_HSDIV_CTRL0;          /* HSDIV_CTRL0 register for pll1 */
    volatile uint32_t PLL1_HSDIV_CTRL1;          /* HSDIV_CTRL1 register for pll1 */
    volatile uint32_t PLL1_HSDIV_CTRL2;          /* HSDIV_CTRL2 register for pll1 */
    volatile uint32_t PLL1_HSDIV_CTRL3;          /* HSDIV_CTRL3 register for pll1 */
    volatile uint32_t PLL1_HSDIV_CTRL4;          /* HSDIV_CTRL4 register for pll1 */
    volatile uint32_t PLL1_HSDIV_CTRL5;          /* HSDIV_CTRL5 register for pll1 */
    volatile uint32_t PLL1_HSDIV_CTRL6;          /* HSDIV_CTRL6 register for pll1 */
    volatile uint8_t  Resv_8192[3940];
    volatile uint32_t PLL2_PID;                  /* Peripheral Identification Register */
    volatile uint8_t  Resv_8200[4];
    volatile uint32_t PLL2_CFG;                  /* pll MMR Configuration */
    volatile uint8_t  Resv_8208[4];
    volatile uint32_t PLL2_LOCKKEY0;             /* PLL2_LOCKKEY0 - PLL2 Lock Key 0 Register */
    volatile uint32_t PLL2_LOCKKEY1;             /* PLL2_LOCKKEY1 - PLL2 Lock Key 1 RegisterAddr */
    volatile uint8_t  Resv_8224[8];
    volatile uint32_t PLL2_CTRL;                 /* PLL2_CTRL - PLL2 Control */
    volatile uint32_t PLL2_STAT;                 /* PLL2_STAT - PLL2 Status */
    volatile uint8_t  Resv_8240[8];
    volatile uint32_t PLL2_FREQ_CTRL0;           /* PLL0_FREQ_CTRL0 - PLL2 Frequency Control 2 Register */
    volatile uint32_t PLL2_FREQ_CTRL1;           /* PLL2_FREQ_CTRL1 - PLL0 Frequency Control 1 Register */
    volatile uint32_t PLL2_DIV_CTRL;             /* PLL2_DIV_CTRL - PLL2 Output Clock Divider Register */
    volatile uint8_t  Resv_8256[4];
    volatile uint32_t PLL2_SS_CTRL;              /* PLL_SS_CTRL register for pll2 */
    volatile uint32_t PLL2_SS_SPREAD;            /* PLL_SS_SPREAD register for pll2 */
    volatile uint8_t  Resv_8288[24];
    volatile uint32_t PLL2_CAL_CTRL;             /* PLL2_CAL_CTRL - PLL2 Calibration Control Register */
    volatile uint32_t PLL2_CAL_STAT;             /* PLL2_CAL_STAT - PLL2 Calibration Status Register */
    volatile uint8_t  Resv_8320[24];
    volatile uint32_t PLL2_HSDIV_CTRL0;          /* HSDIV_CTRL0 register for pll2 */
    volatile uint32_t PLL2_HSDIV_CTRL1;          /* HSDIV_CTRL1 register for pll2 */
    volatile uint32_t PLL2_HSDIV_CTRL2;          /* HSDIV_CTRL2 register for pll2 */
    volatile uint32_t PLL2_HSDIV_CTRL3;          /* HSDIV_CTRL3 register for pll2 */
    volatile uint32_t PLL2_HSDIV_CTRL4;          /* HSDIV_CTRL4 register for pll2 */
    volatile uint32_t PLL2_HSDIV_CTRL5;          /* HSDIV_CTRL5 register for pll2 */
    volatile uint32_t PLL2_HSDIV_CTRL6;          /* HSDIV_CTRL6 register for pll2 */
    volatile uint32_t PLL2_HSDIV_CTRL7;          /* HSDIV_CTRL7 register for pll2 */
    volatile uint32_t PLL2_HSDIV_CTRL8;          /* HSDIV_CTRL8 register for pll2 */
    volatile uint32_t PLL2_HSDIV_CTRL9;          /* HSDIV_CTRL9 register for pll2 */
    volatile uint8_t  Resv_32768[24408];
    volatile uint32_t PLL8_PID;                  /* Peripheral Identification Register */
    volatile uint8_t  Resv_32776[4];
    volatile uint32_t PLL8_CFG;                  /* pll MMR Configuration */
    volatile uint8_t  Resv_32784[4];
    volatile uint32_t PLL8_LOCKKEY0;             /* PLL8_LOCKKEY0 - PLL8 Lock Key 0 Register */
    volatile uint32_t PLL8_LOCKKEY1;             /* PLL8_LOCKKEY1 - PLL8 Lock Key 1 RegisterAddr */
    volatile uint8_t  Resv_32800[8];
    volatile uint32_t PLL8_CTRL;                 /* PLL8_CTRL - PLL8 Control */
    volatile uint32_t PLL8_STAT;                 /* PLL8_STAT - PLL8 Status */
    volatile uint8_t  Resv_32816[8];
    volatile uint32_t PLL8_FREQ_CTRL0;           /* PLL0_FREQ_CTRL0 - PLL8 Frequency Control 8 Register */
    volatile uint32_t PLL8_FREQ_CTRL1;           /* PLL8_FREQ_CTRL1 - PLL0 Frequency Control 1 Register */
    volatile uint32_t PLL8_DIV_CTRL;             /* PLL8_DIV_CTRL - PLL8 Output Clock Divider Register */
    volatile uint8_t  Resv_32832[4];
    volatile uint32_t PLL8_SS_CTRL;              /* PLL_SS_CTRL register for pll8 */
    volatile uint32_t PLL8_SS_SPREAD;            /* PLL_SS_SPREAD register for pll8 */
    volatile uint8_t  Resv_32864[24];
    volatile uint32_t PLL8_CAL_CTRL;             /* PLL8_CAL_CTRL - PLL8 Calibration Control Register */
    volatile uint32_t PLL8_CAL_STAT;             /* PLL8_CAL_STAT - PLL8 Calibration Status Register */
    volatile uint8_t  Resv_32896[24];
    volatile uint32_t PLL8_HSDIV_CTRL0;          /* HSDIV_CTRL0 register for pll8 */
    volatile uint8_t  Resv_49152[16252];
    volatile uint32_t PLL12_PID;                 /* Peripheral Identification Register */
    volatile uint8_t  Resv_49160[4];
    volatile uint32_t PLL12_CFG;                 /* pll MMR Configuration */
    volatile uint8_t  Resv_49168[4];
    volatile uint32_t PLL12_LOCKKEY0;            /* PLL12_LOCKKEY0 - PLL12 Lock Key 0 Register */
    volatile uint32_t PLL12_LOCKKEY1;            /* PLL12_LOCKKEY1 - PLL12 Lock Key 1 RegisterAddr */
    volatile uint8_t  Resv_49184[8];
    volatile uint32_t PLL12_CTRL;                /* PLL12_CTRL - PLL12 Control */
    volatile uint32_t PLL12_STAT;                /* PLL12_STAT - PLL12 Status */
    volatile uint8_t  Resv_49200[8];
    volatile uint32_t PLL12_FREQ_CTRL0;          /* PLL0_FREQ_CTRL0 - PLL12 Frequency Control 12 Register */
    volatile uint32_t PLL12_FREQ_CTRL1;          /* PLL12_FREQ_CTRL1 - PLL0 Frequency Control 1 Register */
    volatile uint32_t PLL12_DIV_CTRL;            /* PLL12_DIV_CTRL - PLL12 Output Clock Divider Register */
    volatile uint8_t  Resv_49216[4];
    volatile uint32_t PLL12_SS_CTRL;             /* PLL_SS_CTRL register for pll12 */
    volatile uint32_t PLL12_SS_SPREAD;           /* PLL_SS_SPREAD register for pll12 */
    volatile uint8_t  Resv_49248[24];
    volatile uint32_t PLL12_CAL_CTRL;            /* PLL12_CAL_CTRL - PLL12 Calibration Control Register */
    volatile uint32_t PLL12_CAL_STAT;            /* PLL12_CAL_STAT - PLL12 Calibration Status Register */
    volatile uint8_t  Resv_49280[24];
    volatile uint32_t PLL12_HSDIV_CTRL0;         /* HSDIV_CTRL0 register for pll12 */
    volatile uint8_t  Resv_57344[8060];
    volatile uint32_t PLL14_PID;                 /* Peripheral Identification Register */
    volatile uint8_t  Resv_57352[4];
    volatile uint32_t PLL14_CFG;                 /* pll MMR Configuration */
    volatile uint8_t  Resv_57360[4];
    volatile uint32_t PLL14_LOCKKEY0;            /* PLL14_LOCKKEY0 - PLL14 Lock Key 0 Register */
    volatile uint32_t PLL14_LOCKKEY1;            /* PLL14_LOCKKEY1 - PLL14 Lock Key 1 RegisterAddr */
    volatile uint8_t  Resv_57376[8];
    volatile uint32_t PLL14_CTRL;                /* PLL14_CTRL - PLL14 Control */
    volatile uint32_t PLL14_STAT;                /* PLL14_STAT - PLL14 Status */
    volatile uint8_t  Resv_57392[8];
    volatile uint32_t PLL14_FREQ_CTRL0;          /* PLL0_FREQ_CTRL0 - PLL14 Frequency Control 14 Register */
    volatile uint32_t PLL14_FREQ_CTRL1;          /* PLL14_FREQ_CTRL1 - PLL0 Frequency Control 1 Register */
    volatile uint32_t PLL14_DIV_CTRL;            /* PLL14_DIV_CTRL - PLL14 Output Clock Divider Register */
    volatile uint8_t  Resv_57408[4];
    volatile uint32_t PLL14_SS_CTRL;             /* PLL_SS_CTRL register for pll14 */
    volatile uint32_t PLL14_SS_SPREAD;           /* PLL_SS_SPREAD register for pll14 */
    volatile uint8_t  Resv_57440[24];
    volatile uint32_t PLL14_CAL_CTRL;            /* PLL14_CAL_CTRL - PLL14 Calibration Control Register */
    volatile uint32_t PLL14_CAL_STAT;            /* PLL14_CAL_STAT - PLL14 Calibration Status Register */
    volatile uint8_t  Resv_57472[24];
    volatile uint32_t PLL14_HSDIV_CTRL0;         /* HSDIV_CTRL0 register for pll14 */
    volatile uint32_t PLL14_HSDIV_CTRL1;         /* HSDIV_CTRL1 register for pll14 */
} CSL_main_pll_mmr_cfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID                                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CFG                                     (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_LOCKKEY0                                (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_LOCKKEY1                                (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL                                    (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_STAT                                    (0x00000024U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_FREQ_CTRL0                              (0x00000030U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_FREQ_CTRL1                              (0x00000034U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_DIV_CTRL                                (0x00000038U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_CTRL                                 (0x00000040U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_SPREAD                               (0x00000044U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_CTRL                                (0x00000060U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_STAT                                (0x00000064U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL0                             (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL1                             (0x00000084U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL2                             (0x00000088U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL3                             (0x0000008CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL4                             (0x00000090U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL5                             (0x00000094U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL6                             (0x00000098U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL7                             (0x0000009CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL8                             (0x000000A0U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL9                             (0x000000A4U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID                                     (0x00001000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CFG                                     (0x00001008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_LOCKKEY0                                (0x00001010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_LOCKKEY1                                (0x00001014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL                                    (0x00001020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_STAT                                    (0x00001024U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_FREQ_CTRL0                              (0x00001030U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_FREQ_CTRL1                              (0x00001034U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_DIV_CTRL                                (0x00001038U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_CTRL                                 (0x00001040U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_SPREAD                               (0x00001044U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_CTRL                                (0x00001060U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_STAT                                (0x00001064U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL0                             (0x00001080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL1                             (0x00001084U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL2                             (0x00001088U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL3                             (0x0000108CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL4                             (0x00001090U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL5                             (0x00001094U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL6                             (0x00001098U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID                                     (0x00002000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CFG                                     (0x00002008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_LOCKKEY0                                (0x00002010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_LOCKKEY1                                (0x00002014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL                                    (0x00002020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_STAT                                    (0x00002024U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_FREQ_CTRL0                              (0x00002030U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_FREQ_CTRL1                              (0x00002034U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_DIV_CTRL                                (0x00002038U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_CTRL                                 (0x00002040U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_SPREAD                               (0x00002044U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_CTRL                                (0x00002060U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_STAT                                (0x00002064U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL0                             (0x00002080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL1                             (0x00002084U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL2                             (0x00002088U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL3                             (0x0000208CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL4                             (0x00002090U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL5                             (0x00002094U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL6                             (0x00002098U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL7                             (0x0000209CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL8                             (0x000020A0U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL9                             (0x000020A4U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID                                     (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CFG                                     (0x00008008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_LOCKKEY0                                (0x00008010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_LOCKKEY1                                (0x00008014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL                                    (0x00008020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_STAT                                    (0x00008024U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_FREQ_CTRL0                              (0x00008030U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_FREQ_CTRL1                              (0x00008034U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_DIV_CTRL                                (0x00008038U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_CTRL                                 (0x00008040U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_SPREAD                               (0x00008044U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_CTRL                                (0x00008060U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_STAT                                (0x00008064U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_HSDIV_CTRL0                             (0x00008080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID                                    (0x0000C000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CFG                                    (0x0000C008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_LOCKKEY0                               (0x0000C010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_LOCKKEY1                               (0x0000C014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL                                   (0x0000C020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_STAT                                   (0x0000C024U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_FREQ_CTRL0                             (0x0000C030U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_FREQ_CTRL1                             (0x0000C034U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_DIV_CTRL                               (0x0000C038U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_CTRL                                (0x0000C040U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_SPREAD                              (0x0000C044U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_CTRL                               (0x0000C060U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_STAT                               (0x0000C064U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_HSDIV_CTRL0                            (0x0000C080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID                                    (0x0000E000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CFG                                    (0x0000E008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_LOCKKEY0                               (0x0000E010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_LOCKKEY1                               (0x0000E014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL                                   (0x0000E020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_STAT                                   (0x0000E024U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_FREQ_CTRL0                             (0x0000E030U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_FREQ_CTRL1                             (0x0000E034U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_DIV_CTRL                               (0x0000E038U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_CTRL                                (0x0000E040U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_SPREAD                              (0x0000E044U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_CTRL                               (0x0000E060U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_STAT                               (0x0000E064U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL0                            (0x0000E080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL1                            (0x0000E084U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PLL0_PID */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_MINOR_MASK                          (0x0000003FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_MINOR_SHIFT                         (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_MINOR_MAX                           (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_CUSTOM_MASK                         (0x000000C0U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_CUSTOM_SHIFT                        (0x00000006U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_CUSTOM_MAX                          (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_MAJOR_MASK                          (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_MAJOR_SHIFT                         (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_MAJOR_MAX                           (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_MISC_MASK                           (0x0000F800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_MISC_SHIFT                          (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_MISC_MAX                            (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_MODULE_MASK                         (0x0FFF0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_MODULE_SHIFT                        (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_MODULE_MAX                          (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_BU_MASK                             (0x30000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_BU_SHIFT                            (0x0000001CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_BU_MAX                              (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_SCHEME_MASK                         (0xC0000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_SCHEME_SHIFT                        (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PID_SCHEME_MAX                          (0x00000003U)

/* PLL0_CFG */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CFG_PLL_TYPE_MASK                       (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CFG_PLL_TYPE_SHIFT                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CFG_PLL_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CFG_SSM_WVTBL_MASK                      (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CFG_SSM_WVTBL_SHIFT                     (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CFG_SSM_WVTBL_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CFG_SSM_TYPE_MASK                       (0x00001800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CFG_SSM_TYPE_SHIFT                      (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CFG_SSM_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CFG_HSDIV_PRSNT_MASK                    (0xFFFF0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CFG_HSDIV_PRSNT_SHIFT                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CFG_HSDIV_PRSNT_MAX                     (0x0000FFFFU)

/* PLL0_LOCKKEY0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_LOCKKEY0_KEY_MASK                       (0xFFFFFFFEU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_LOCKKEY0_KEY_SHIFT                      (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_LOCKKEY0_KEY_MAX                        (0x7FFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_LOCKKEY0_UNLOCKED_MASK                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_LOCKKEY0_UNLOCKED_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_LOCKKEY0_UNLOCKED_MAX                   (0x00000001U)

/* PLL0_LOCKKEY1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_LOCKKEY1_LOCKKEY1_VAL_MASK              (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_LOCKKEY1_LOCKKEY1_VAL_SHIFT             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_LOCKKEY1_LOCKKEY1_VAL_MAX               (0xFFFFFFFFU)

/* PLL0_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_DAC_EN_MASK                        (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_DAC_EN_SHIFT                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_DAC_EN_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_DSM_EN_MASK                        (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_DSM_EN_SHIFT                       (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_DSM_EN_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_CLK_POSTDIV_EN_MASK                (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_CLK_POSTDIV_EN_SHIFT               (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_CLK_POSTDIV_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_CLK_4PH_EN_MASK                    (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_CLK_4PH_EN_SHIFT                   (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_CLK_4PH_EN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_INTL_BYP_EN_MASK                   (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_INTL_BYP_EN_SHIFT                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_INTL_BYP_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_PLL_EN_MASK                        (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_PLL_EN_SHIFT                       (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_PLL_EN_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_BYP_ON_LOCKLOSS_MASK               (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_BYP_ON_LOCKLOSS_SHIFT              (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_BYP_ON_LOCKLOSS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_BYPASS_EN_MASK                     (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_BYPASS_EN_SHIFT                    (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CTRL_BYPASS_EN_MAX                      (0x00000001U)

/* PLL0_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_STAT_LOCK_MASK                          (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_STAT_LOCK_SHIFT                         (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_STAT_LOCK_MAX                           (0x00000001U)

/* PLL0_FREQ_CTRL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_FREQ_CTRL0_FB_DIV_INT_MASK              (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_FREQ_CTRL0_FB_DIV_INT_SHIFT             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_FREQ_CTRL0_FB_DIV_INT_MAX               (0x00000FFFU)

/* PLL0_FREQ_CTRL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_FREQ_CTRL1_FB_DIV_FRAC_MASK             (0x00FFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_FREQ_CTRL1_FB_DIV_FRAC_SHIFT            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_FREQ_CTRL1_FB_DIV_FRAC_MAX              (0x00FFFFFFU)

/* PLL0_DIV_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_DIV_CTRL_REF_DIV_MASK                   (0x0000003FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_DIV_CTRL_REF_DIV_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_DIV_CTRL_REF_DIV_MAX                    (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_DIV_CTRL_POST_DIV1_MASK                 (0x00070000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_DIV_CTRL_POST_DIV1_SHIFT                (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_DIV_CTRL_POST_DIV1_MAX                  (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_DIV_CTRL_POST_DIV2_MASK                 (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_DIV_CTRL_POST_DIV2_SHIFT                (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_DIV_CTRL_POST_DIV2_MAX                  (0x00000007U)

/* PLL0_SS_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_CTRL_WAVE_SEL_MASK                   (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_CTRL_WAVE_SEL_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_CTRL_WAVE_SEL_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_CTRL_DOWNSPREAD_EN_MASK              (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_CTRL_DOWNSPREAD_EN_SHIFT             (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_CTRL_DOWNSPREAD_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_CTRL_RESET_MASK                      (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_CTRL_RESET_SHIFT                     (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_CTRL_RESET_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_CTRL_WV_TBLE_MAXADDR_MASK            (0x03FC0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_CTRL_WV_TBLE_MAXADDR_SHIFT           (0x00000012U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_CTRL_WV_TBLE_MAXADDR_MAX             (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_CTRL_BYPASS_EN_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_CTRL_BYPASS_EN_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_CTRL_BYPASS_EN_MAX                   (0x00000001U)

/* PLL0_SS_SPREAD */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_SPREAD_SPREAD_MASK                   (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_SPREAD_SPREAD_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_SPREAD_SPREAD_MAX                    (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_SPREAD_MOD_DIV_MASK                  (0x000F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_SPREAD_MOD_DIV_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_SS_SPREAD_MOD_DIV_MAX                   (0x0000000FU)

/* PLL0_CAL_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_IN_MASK                    (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_IN_SHIFT                   (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_IN_MAX                     (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_BYP_MASK                   (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_BYP_SHIFT                  (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_BYP_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_CNT_MASK                   (0x00070000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_CNT_SHIFT                  (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_CNT_MAX                    (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_CTRL_FAST_CAL_MASK                  (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_CTRL_FAST_CAL_SHIFT                 (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_CTRL_FAST_CAL_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_EN_MASK                    (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_EN_SHIFT                   (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_CTRL_CAL_EN_MAX                     (0x00000001U)

/* PLL0_CAL_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_STAT_CAL_OUT_MASK                   (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_STAT_CAL_OUT_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_STAT_CAL_OUT_MAX                    (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_STAT_LOCK_CNT_MASK                  (0x000F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_STAT_LOCK_CNT_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_STAT_LOCK_CNT_MAX                   (0x0000000FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_STAT_CAL_LOCK_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_STAT_CAL_LOCK_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_CAL_STAT_CAL_LOCK_MAX                   (0x00000001U)

/* PLL0_HSDIV_CTRL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL0_RESET_MAX                   (0x00000001U)

/* PLL0_HSDIV_CTRL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL1_RESET_MAX                   (0x00000001U)

/* PLL0_HSDIV_CTRL2 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL2_RESET_MAX                   (0x00000001U)

/* PLL0_HSDIV_CTRL3 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL3_RESET_MAX                   (0x00000001U)

/* PLL0_HSDIV_CTRL4 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL4_RESET_MAX                   (0x00000001U)

/* PLL0_HSDIV_CTRL5 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL5_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL5_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL5_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL5_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL5_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL5_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL5_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL5_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL5_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL5_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL5_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL5_RESET_MAX                   (0x00000001U)

/* PLL0_HSDIV_CTRL6 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL6_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL6_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL6_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL6_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL6_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL6_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL6_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL6_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL6_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL6_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL6_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL6_RESET_MAX                   (0x00000001U)

/* PLL0_HSDIV_CTRL7 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL7_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL7_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL7_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL7_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL7_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL7_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL7_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL7_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL7_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL7_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL7_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL7_RESET_MAX                   (0x00000001U)

/* PLL0_HSDIV_CTRL8 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL8_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL8_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL8_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL8_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL8_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL8_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL8_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL8_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL8_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL8_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL8_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL8_RESET_MAX                   (0x00000001U)

/* PLL0_HSDIV_CTRL9 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL9_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL9_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL9_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL9_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL9_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL9_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL9_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL9_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL9_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL9_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL9_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL9_RESET_MAX                   (0x00000001U)

/* PLL1_PID */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_MINOR_MASK                          (0x0000003FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_MINOR_SHIFT                         (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_MINOR_MAX                           (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_CUSTOM_MASK                         (0x000000C0U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_CUSTOM_SHIFT                        (0x00000006U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_CUSTOM_MAX                          (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_MAJOR_MASK                          (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_MAJOR_SHIFT                         (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_MAJOR_MAX                           (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_MISC_MASK                           (0x0000F800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_MISC_SHIFT                          (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_MISC_MAX                            (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_MODULE_MASK                         (0x0FFF0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_MODULE_SHIFT                        (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_MODULE_MAX                          (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_BU_MASK                             (0x30000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_BU_SHIFT                            (0x0000001CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_BU_MAX                              (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_SCHEME_MASK                         (0xC0000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_SCHEME_SHIFT                        (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PID_SCHEME_MAX                          (0x00000003U)

/* PLL1_CFG */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CFG_PLL_TYPE_MASK                       (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CFG_PLL_TYPE_SHIFT                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CFG_PLL_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CFG_SSM_WVTBL_MASK                      (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CFG_SSM_WVTBL_SHIFT                     (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CFG_SSM_WVTBL_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CFG_SSM_TYPE_MASK                       (0x00001800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CFG_SSM_TYPE_SHIFT                      (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CFG_SSM_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CFG_HSDIV_PRSNT_MASK                    (0xFFFF0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CFG_HSDIV_PRSNT_SHIFT                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CFG_HSDIV_PRSNT_MAX                     (0x0000FFFFU)

/* PLL1_LOCKKEY0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_LOCKKEY0_KEY_MASK                       (0xFFFFFFFEU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_LOCKKEY0_KEY_SHIFT                      (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_LOCKKEY0_KEY_MAX                        (0x7FFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_LOCKKEY0_UNLOCKED_MASK                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_LOCKKEY0_UNLOCKED_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_LOCKKEY0_UNLOCKED_MAX                   (0x00000001U)

/* PLL1_LOCKKEY1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_LOCKKEY1_LOCKKEY1_VAL_MASK              (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_LOCKKEY1_LOCKKEY1_VAL_SHIFT             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_LOCKKEY1_LOCKKEY1_VAL_MAX               (0xFFFFFFFFU)

/* PLL1_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_DAC_EN_MASK                        (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_DAC_EN_SHIFT                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_DAC_EN_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_DSM_EN_MASK                        (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_DSM_EN_SHIFT                       (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_DSM_EN_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_CLK_POSTDIV_EN_MASK                (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_CLK_POSTDIV_EN_SHIFT               (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_CLK_POSTDIV_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_CLK_4PH_EN_MASK                    (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_CLK_4PH_EN_SHIFT                   (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_CLK_4PH_EN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_INTL_BYP_EN_MASK                   (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_INTL_BYP_EN_SHIFT                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_INTL_BYP_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_PLL_EN_MASK                        (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_PLL_EN_SHIFT                       (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_PLL_EN_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_BYP_ON_LOCKLOSS_MASK               (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_BYP_ON_LOCKLOSS_SHIFT              (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_BYP_ON_LOCKLOSS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_BYPASS_EN_MASK                     (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_BYPASS_EN_SHIFT                    (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CTRL_BYPASS_EN_MAX                      (0x00000001U)

/* PLL1_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_STAT_LOCK_MASK                          (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_STAT_LOCK_SHIFT                         (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_STAT_LOCK_MAX                           (0x00000001U)

/* PLL1_FREQ_CTRL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_FREQ_CTRL0_FB_DIV_INT_MASK              (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_FREQ_CTRL0_FB_DIV_INT_SHIFT             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_FREQ_CTRL0_FB_DIV_INT_MAX               (0x00000FFFU)

/* PLL1_FREQ_CTRL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_FREQ_CTRL1_FB_DIV_FRAC_MASK             (0x00FFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_FREQ_CTRL1_FB_DIV_FRAC_SHIFT            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_FREQ_CTRL1_FB_DIV_FRAC_MAX              (0x00FFFFFFU)

/* PLL1_DIV_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_DIV_CTRL_REF_DIV_MASK                   (0x0000003FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_DIV_CTRL_REF_DIV_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_DIV_CTRL_REF_DIV_MAX                    (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_DIV_CTRL_POST_DIV1_MASK                 (0x00070000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_DIV_CTRL_POST_DIV1_SHIFT                (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_DIV_CTRL_POST_DIV1_MAX                  (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_DIV_CTRL_POST_DIV2_MASK                 (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_DIV_CTRL_POST_DIV2_SHIFT                (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_DIV_CTRL_POST_DIV2_MAX                  (0x00000007U)

/* PLL1_SS_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_CTRL_WAVE_SEL_MASK                   (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_CTRL_WAVE_SEL_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_CTRL_WAVE_SEL_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_CTRL_DOWNSPREAD_EN_MASK              (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_CTRL_DOWNSPREAD_EN_SHIFT             (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_CTRL_DOWNSPREAD_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_CTRL_RESET_MASK                      (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_CTRL_RESET_SHIFT                     (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_CTRL_RESET_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_CTRL_WV_TBLE_MAXADDR_MASK            (0x03FC0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_CTRL_WV_TBLE_MAXADDR_SHIFT           (0x00000012U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_CTRL_WV_TBLE_MAXADDR_MAX             (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_CTRL_BYPASS_EN_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_CTRL_BYPASS_EN_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_CTRL_BYPASS_EN_MAX                   (0x00000001U)

/* PLL1_SS_SPREAD */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_SPREAD_SPREAD_MASK                   (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_SPREAD_SPREAD_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_SPREAD_SPREAD_MAX                    (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_SPREAD_MOD_DIV_MASK                  (0x000F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_SPREAD_MOD_DIV_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_SS_SPREAD_MOD_DIV_MAX                   (0x0000000FU)

/* PLL1_CAL_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_CTRL_CAL_IN_MASK                    (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_CTRL_CAL_IN_SHIFT                   (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_CTRL_CAL_IN_MAX                     (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_CTRL_CAL_BYP_MASK                   (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_CTRL_CAL_BYP_SHIFT                  (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_CTRL_CAL_BYP_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_CTRL_CAL_CNT_MASK                   (0x00070000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_CTRL_CAL_CNT_SHIFT                  (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_CTRL_CAL_CNT_MAX                    (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_CTRL_FAST_CAL_MASK                  (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_CTRL_FAST_CAL_SHIFT                 (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_CTRL_FAST_CAL_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_CTRL_CAL_EN_MASK                    (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_CTRL_CAL_EN_SHIFT                   (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_CTRL_CAL_EN_MAX                     (0x00000001U)

/* PLL1_CAL_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_STAT_CAL_OUT_MASK                   (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_STAT_CAL_OUT_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_STAT_CAL_OUT_MAX                    (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_STAT_LOCK_CNT_MASK                  (0x000F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_STAT_LOCK_CNT_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_STAT_LOCK_CNT_MAX                   (0x0000000FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_STAT_CAL_LOCK_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_STAT_CAL_LOCK_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_CAL_STAT_CAL_LOCK_MAX                   (0x00000001U)

/* PLL1_HSDIV_CTRL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL0_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL0_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL0_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL0_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL0_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL0_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL0_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL0_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL0_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL0_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL0_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL0_RESET_MAX                   (0x00000001U)

/* PLL1_HSDIV_CTRL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL1_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL1_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL1_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL1_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL1_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL1_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL1_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL1_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL1_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL1_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL1_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL1_RESET_MAX                   (0x00000001U)

/* PLL1_HSDIV_CTRL2 */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL2_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL2_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL2_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL2_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL2_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL2_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL2_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL2_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL2_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL2_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL2_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL2_RESET_MAX                   (0x00000001U)

/* PLL1_HSDIV_CTRL3 */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL3_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL3_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL3_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL3_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL3_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL3_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL3_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL3_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL3_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL3_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL3_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL3_RESET_MAX                   (0x00000001U)

/* PLL1_HSDIV_CTRL4 */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL4_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL4_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL4_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL4_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL4_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL4_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL4_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL4_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL4_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL4_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL4_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL4_RESET_MAX                   (0x00000001U)

/* PLL1_HSDIV_CTRL5 */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL5_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL5_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL5_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL5_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL5_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL5_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL5_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL5_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL5_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL5_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL5_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL5_RESET_MAX                   (0x00000001U)

/* PLL1_HSDIV_CTRL6 */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL6_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL6_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL6_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL6_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL6_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL6_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL6_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL6_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL6_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL6_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL6_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_HSDIV_CTRL6_RESET_MAX                   (0x00000001U)

/* PLL2_PID */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_MINOR_MASK                          (0x0000003FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_MINOR_SHIFT                         (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_MINOR_MAX                           (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_CUSTOM_MASK                         (0x000000C0U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_CUSTOM_SHIFT                        (0x00000006U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_CUSTOM_MAX                          (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_MAJOR_MASK                          (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_MAJOR_SHIFT                         (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_MAJOR_MAX                           (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_MISC_MASK                           (0x0000F800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_MISC_SHIFT                          (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_MISC_MAX                            (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_MODULE_MASK                         (0x0FFF0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_MODULE_SHIFT                        (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_MODULE_MAX                          (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_BU_MASK                             (0x30000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_BU_SHIFT                            (0x0000001CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_BU_MAX                              (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_SCHEME_MASK                         (0xC0000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_SCHEME_SHIFT                        (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PID_SCHEME_MAX                          (0x00000003U)

/* PLL2_CFG */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CFG_PLL_TYPE_MASK                       (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CFG_PLL_TYPE_SHIFT                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CFG_PLL_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CFG_SSM_WVTBL_MASK                      (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CFG_SSM_WVTBL_SHIFT                     (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CFG_SSM_WVTBL_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CFG_SSM_TYPE_MASK                       (0x00001800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CFG_SSM_TYPE_SHIFT                      (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CFG_SSM_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CFG_HSDIV_PRSNT_MASK                    (0xFFFF0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CFG_HSDIV_PRSNT_SHIFT                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CFG_HSDIV_PRSNT_MAX                     (0x0000FFFFU)

/* PLL2_LOCKKEY0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_LOCKKEY0_KEY_MASK                       (0xFFFFFFFEU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_LOCKKEY0_KEY_SHIFT                      (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_LOCKKEY0_KEY_MAX                        (0x7FFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_LOCKKEY0_UNLOCKED_MASK                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_LOCKKEY0_UNLOCKED_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_LOCKKEY0_UNLOCKED_MAX                   (0x00000001U)

/* PLL2_LOCKKEY1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_LOCKKEY1_LOCKKEY1_VAL_MASK              (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_LOCKKEY1_LOCKKEY1_VAL_SHIFT             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_LOCKKEY1_LOCKKEY1_VAL_MAX               (0xFFFFFFFFU)

/* PLL2_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_DAC_EN_MASK                        (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_DAC_EN_SHIFT                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_DAC_EN_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_DSM_EN_MASK                        (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_DSM_EN_SHIFT                       (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_DSM_EN_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_CLK_POSTDIV_EN_MASK                (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_CLK_POSTDIV_EN_SHIFT               (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_CLK_POSTDIV_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_CLK_4PH_EN_MASK                    (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_CLK_4PH_EN_SHIFT                   (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_CLK_4PH_EN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_INTL_BYP_EN_MASK                   (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_INTL_BYP_EN_SHIFT                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_INTL_BYP_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_PLL_EN_MASK                        (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_PLL_EN_SHIFT                       (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_PLL_EN_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_BYP_ON_LOCKLOSS_MASK               (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_BYP_ON_LOCKLOSS_SHIFT              (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_BYP_ON_LOCKLOSS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_BYPASS_EN_MASK                     (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_BYPASS_EN_SHIFT                    (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CTRL_BYPASS_EN_MAX                      (0x00000001U)

/* PLL2_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_STAT_LOCK_MASK                          (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_STAT_LOCK_SHIFT                         (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_STAT_LOCK_MAX                           (0x00000001U)

/* PLL2_FREQ_CTRL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_FREQ_CTRL0_FB_DIV_INT_MASK              (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_FREQ_CTRL0_FB_DIV_INT_SHIFT             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_FREQ_CTRL0_FB_DIV_INT_MAX               (0x00000FFFU)

/* PLL2_FREQ_CTRL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_FREQ_CTRL1_FB_DIV_FRAC_MASK             (0x00FFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_FREQ_CTRL1_FB_DIV_FRAC_SHIFT            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_FREQ_CTRL1_FB_DIV_FRAC_MAX              (0x00FFFFFFU)

/* PLL2_DIV_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_DIV_CTRL_REF_DIV_MASK                   (0x0000003FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_DIV_CTRL_REF_DIV_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_DIV_CTRL_REF_DIV_MAX                    (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_DIV_CTRL_POST_DIV1_MASK                 (0x00070000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_DIV_CTRL_POST_DIV1_SHIFT                (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_DIV_CTRL_POST_DIV1_MAX                  (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_DIV_CTRL_POST_DIV2_MASK                 (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_DIV_CTRL_POST_DIV2_SHIFT                (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_DIV_CTRL_POST_DIV2_MAX                  (0x00000007U)

/* PLL2_SS_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_CTRL_WAVE_SEL_MASK                   (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_CTRL_WAVE_SEL_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_CTRL_WAVE_SEL_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_CTRL_DOWNSPREAD_EN_MASK              (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_CTRL_DOWNSPREAD_EN_SHIFT             (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_CTRL_DOWNSPREAD_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_CTRL_RESET_MASK                      (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_CTRL_RESET_SHIFT                     (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_CTRL_RESET_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_CTRL_WV_TBLE_MAXADDR_MASK            (0x03FC0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_CTRL_WV_TBLE_MAXADDR_SHIFT           (0x00000012U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_CTRL_WV_TBLE_MAXADDR_MAX             (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_CTRL_BYPASS_EN_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_CTRL_BYPASS_EN_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_CTRL_BYPASS_EN_MAX                   (0x00000001U)

/* PLL2_SS_SPREAD */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_SPREAD_SPREAD_MASK                   (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_SPREAD_SPREAD_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_SPREAD_SPREAD_MAX                    (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_SPREAD_MOD_DIV_MASK                  (0x000F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_SPREAD_MOD_DIV_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_SS_SPREAD_MOD_DIV_MAX                   (0x0000000FU)

/* PLL2_CAL_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_CTRL_CAL_IN_MASK                    (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_CTRL_CAL_IN_SHIFT                   (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_CTRL_CAL_IN_MAX                     (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_CTRL_CAL_BYP_MASK                   (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_CTRL_CAL_BYP_SHIFT                  (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_CTRL_CAL_BYP_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_CTRL_CAL_CNT_MASK                   (0x00070000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_CTRL_CAL_CNT_SHIFT                  (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_CTRL_CAL_CNT_MAX                    (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_CTRL_FAST_CAL_MASK                  (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_CTRL_FAST_CAL_SHIFT                 (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_CTRL_FAST_CAL_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_CTRL_CAL_EN_MASK                    (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_CTRL_CAL_EN_SHIFT                   (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_CTRL_CAL_EN_MAX                     (0x00000001U)

/* PLL2_CAL_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_STAT_CAL_OUT_MASK                   (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_STAT_CAL_OUT_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_STAT_CAL_OUT_MAX                    (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_STAT_LOCK_CNT_MASK                  (0x000F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_STAT_LOCK_CNT_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_STAT_LOCK_CNT_MAX                   (0x0000000FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_STAT_CAL_LOCK_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_STAT_CAL_LOCK_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_CAL_STAT_CAL_LOCK_MAX                   (0x00000001U)

/* PLL2_HSDIV_CTRL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL0_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL0_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL0_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL0_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL0_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL0_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL0_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL0_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL0_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL0_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL0_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL0_RESET_MAX                   (0x00000001U)

/* PLL2_HSDIV_CTRL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL1_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL1_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL1_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL1_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL1_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL1_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL1_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL1_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL1_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL1_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL1_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL1_RESET_MAX                   (0x00000001U)

/* PLL2_HSDIV_CTRL2 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL2_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL2_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL2_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL2_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL2_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL2_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL2_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL2_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL2_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL2_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL2_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL2_RESET_MAX                   (0x00000001U)

/* PLL2_HSDIV_CTRL3 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL3_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL3_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL3_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL3_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL3_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL3_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL3_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL3_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL3_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL3_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL3_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL3_RESET_MAX                   (0x00000001U)

/* PLL2_HSDIV_CTRL4 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL4_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL4_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL4_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL4_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL4_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL4_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL4_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL4_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL4_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL4_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL4_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL4_RESET_MAX                   (0x00000001U)

/* PLL2_HSDIV_CTRL5 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL5_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL5_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL5_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL5_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL5_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL5_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL5_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL5_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL5_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL5_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL5_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL5_RESET_MAX                   (0x00000001U)

/* PLL2_HSDIV_CTRL6 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL6_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL6_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL6_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL6_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL6_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL6_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL6_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL6_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL6_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL6_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL6_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL6_RESET_MAX                   (0x00000001U)

/* PLL2_HSDIV_CTRL7 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL7_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL7_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL7_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL7_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL7_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL7_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL7_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL7_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL7_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL7_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL7_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL7_RESET_MAX                   (0x00000001U)

/* PLL2_HSDIV_CTRL8 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL8_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL8_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL8_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL8_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL8_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL8_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL8_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL8_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL8_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL8_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL8_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL8_RESET_MAX                   (0x00000001U)

/* PLL2_HSDIV_CTRL9 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL9_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL9_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL9_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL9_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL9_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL9_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL9_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL9_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL9_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL9_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL9_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL9_RESET_MAX                   (0x00000001U)

/* PLL8_PID */

#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_MINOR_MASK                          (0x0000003FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_MINOR_SHIFT                         (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_MINOR_MAX                           (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_CUSTOM_MASK                         (0x000000C0U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_CUSTOM_SHIFT                        (0x00000006U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_CUSTOM_MAX                          (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_MAJOR_MASK                          (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_MAJOR_SHIFT                         (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_MAJOR_MAX                           (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_MISC_MASK                           (0x0000F800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_MISC_SHIFT                          (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_MISC_MAX                            (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_MODULE_MASK                         (0x0FFF0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_MODULE_SHIFT                        (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_MODULE_MAX                          (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_BU_MASK                             (0x30000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_BU_SHIFT                            (0x0000001CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_BU_MAX                              (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_SCHEME_MASK                         (0xC0000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_SCHEME_SHIFT                        (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_PID_SCHEME_MAX                          (0x00000003U)

/* PLL8_CFG */

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CFG_PLL_TYPE_MASK                       (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CFG_PLL_TYPE_SHIFT                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CFG_PLL_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CFG_SSM_WVTBL_MASK                      (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CFG_SSM_WVTBL_SHIFT                     (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CFG_SSM_WVTBL_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CFG_SSM_TYPE_MASK                       (0x00001800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CFG_SSM_TYPE_SHIFT                      (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CFG_SSM_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CFG_HSDIV_PRSNT_MASK                    (0xFFFF0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CFG_HSDIV_PRSNT_SHIFT                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CFG_HSDIV_PRSNT_MAX                     (0x0000FFFFU)

/* PLL8_LOCKKEY0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL8_LOCKKEY0_KEY_MASK                       (0xFFFFFFFEU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_LOCKKEY0_KEY_SHIFT                      (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_LOCKKEY0_KEY_MAX                        (0x7FFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_LOCKKEY0_UNLOCKED_MASK                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_LOCKKEY0_UNLOCKED_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_LOCKKEY0_UNLOCKED_MAX                   (0x00000001U)

/* PLL8_LOCKKEY1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL8_LOCKKEY1_LOCKKEY1_VAL_MASK              (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_LOCKKEY1_LOCKKEY1_VAL_SHIFT             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_LOCKKEY1_LOCKKEY1_VAL_MAX               (0xFFFFFFFFU)

/* PLL8_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_DAC_EN_MASK                        (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_DAC_EN_SHIFT                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_DAC_EN_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_DSM_EN_MASK                        (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_DSM_EN_SHIFT                       (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_DSM_EN_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_CLK_POSTDIV_EN_MASK                (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_CLK_POSTDIV_EN_SHIFT               (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_CLK_POSTDIV_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_CLK_4PH_EN_MASK                    (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_CLK_4PH_EN_SHIFT                   (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_CLK_4PH_EN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_INTL_BYP_EN_MASK                   (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_INTL_BYP_EN_SHIFT                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_INTL_BYP_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_PLL_EN_MASK                        (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_PLL_EN_SHIFT                       (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_PLL_EN_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_BYP_ON_LOCKLOSS_MASK               (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_BYP_ON_LOCKLOSS_SHIFT              (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_BYP_ON_LOCKLOSS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_BYPASS_EN_MASK                     (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_BYPASS_EN_SHIFT                    (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CTRL_BYPASS_EN_MAX                      (0x00000001U)

/* PLL8_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL8_STAT_LOCK_MASK                          (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_STAT_LOCK_SHIFT                         (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_STAT_LOCK_MAX                           (0x00000001U)

/* PLL8_FREQ_CTRL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL8_FREQ_CTRL0_FB_DIV_INT_MASK              (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_FREQ_CTRL0_FB_DIV_INT_SHIFT             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_FREQ_CTRL0_FB_DIV_INT_MAX               (0x00000FFFU)

/* PLL8_FREQ_CTRL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL8_FREQ_CTRL1_FB_DIV_FRAC_MASK             (0x00FFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_FREQ_CTRL1_FB_DIV_FRAC_SHIFT            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_FREQ_CTRL1_FB_DIV_FRAC_MAX              (0x00FFFFFFU)

/* PLL8_DIV_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL8_DIV_CTRL_REF_DIV_MASK                   (0x0000003FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_DIV_CTRL_REF_DIV_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_DIV_CTRL_REF_DIV_MAX                    (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_DIV_CTRL_POST_DIV1_MASK                 (0x00070000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_DIV_CTRL_POST_DIV1_SHIFT                (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_DIV_CTRL_POST_DIV1_MAX                  (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_DIV_CTRL_POST_DIV2_MASK                 (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_DIV_CTRL_POST_DIV2_SHIFT                (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_DIV_CTRL_POST_DIV2_MAX                  (0x00000007U)

/* PLL8_SS_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_CTRL_WAVE_SEL_MASK                   (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_CTRL_WAVE_SEL_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_CTRL_WAVE_SEL_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_CTRL_DOWNSPREAD_EN_MASK              (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_CTRL_DOWNSPREAD_EN_SHIFT             (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_CTRL_DOWNSPREAD_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_CTRL_RESET_MASK                      (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_CTRL_RESET_SHIFT                     (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_CTRL_RESET_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_CTRL_WV_TBLE_MAXADDR_MASK            (0x03FC0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_CTRL_WV_TBLE_MAXADDR_SHIFT           (0x00000012U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_CTRL_WV_TBLE_MAXADDR_MAX             (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_CTRL_BYPASS_EN_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_CTRL_BYPASS_EN_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_CTRL_BYPASS_EN_MAX                   (0x00000001U)

/* PLL8_SS_SPREAD */

#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_SPREAD_SPREAD_MASK                   (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_SPREAD_SPREAD_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_SPREAD_SPREAD_MAX                    (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_SPREAD_MOD_DIV_MASK                  (0x000F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_SPREAD_MOD_DIV_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_SS_SPREAD_MOD_DIV_MAX                   (0x0000000FU)

/* PLL8_CAL_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_CTRL_CAL_IN_MASK                    (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_CTRL_CAL_IN_SHIFT                   (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_CTRL_CAL_IN_MAX                     (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_CTRL_CAL_BYP_MASK                   (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_CTRL_CAL_BYP_SHIFT                  (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_CTRL_CAL_BYP_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_CTRL_CAL_CNT_MASK                   (0x00070000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_CTRL_CAL_CNT_SHIFT                  (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_CTRL_CAL_CNT_MAX                    (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_CTRL_FAST_CAL_MASK                  (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_CTRL_FAST_CAL_SHIFT                 (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_CTRL_FAST_CAL_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_CTRL_CAL_EN_MASK                    (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_CTRL_CAL_EN_SHIFT                   (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_CTRL_CAL_EN_MAX                     (0x00000001U)

/* PLL8_CAL_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_STAT_CAL_OUT_MASK                   (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_STAT_CAL_OUT_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_STAT_CAL_OUT_MAX                    (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_STAT_LOCK_CNT_MASK                  (0x000F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_STAT_LOCK_CNT_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_STAT_LOCK_CNT_MAX                   (0x0000000FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_STAT_CAL_LOCK_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_STAT_CAL_LOCK_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_CAL_STAT_CAL_LOCK_MAX                   (0x00000001U)

/* PLL8_HSDIV_CTRL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL8_HSDIV_CTRL0_HSDIV_MASK                  (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_HSDIV_CTRL0_HSDIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_HSDIV_CTRL0_HSDIV_MAX                   (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_HSDIV_CTRL0_SYNC_DIS_MASK               (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_HSDIV_CTRL0_SYNC_DIS_SHIFT              (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_HSDIV_CTRL0_SYNC_DIS_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_HSDIV_CTRL0_CLKOUT_EN_MASK              (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_HSDIV_CTRL0_CLKOUT_EN_SHIFT             (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_HSDIV_CTRL0_CLKOUT_EN_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL8_HSDIV_CTRL0_RESET_MASK                  (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_HSDIV_CTRL0_RESET_SHIFT                 (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL8_HSDIV_CTRL0_RESET_MAX                   (0x00000001U)

/* PLL12_PID */

#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_MINOR_MASK                         (0x0000003FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_MINOR_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_MINOR_MAX                          (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_CUSTOM_MASK                        (0x000000C0U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_CUSTOM_SHIFT                       (0x00000006U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_CUSTOM_MAX                         (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_MAJOR_MASK                         (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_MAJOR_SHIFT                        (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_MAJOR_MAX                          (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_MISC_MASK                          (0x0000F800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_MISC_SHIFT                         (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_MISC_MAX                           (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_MODULE_MASK                        (0x0FFF0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_MODULE_SHIFT                       (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_MODULE_MAX                         (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_BU_MASK                            (0x30000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_BU_SHIFT                           (0x0000001CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_BU_MAX                             (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_SCHEME_MASK                        (0xC0000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_SCHEME_SHIFT                       (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_PID_SCHEME_MAX                         (0x00000003U)

/* PLL12_CFG */

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CFG_PLL_TYPE_MASK                      (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CFG_PLL_TYPE_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CFG_PLL_TYPE_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CFG_SSM_WVTBL_MASK                     (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CFG_SSM_WVTBL_SHIFT                    (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CFG_SSM_WVTBL_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CFG_SSM_TYPE_MASK                      (0x00001800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CFG_SSM_TYPE_SHIFT                     (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CFG_SSM_TYPE_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CFG_HSDIV_PRSNT_MASK                   (0xFFFF0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CFG_HSDIV_PRSNT_SHIFT                  (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CFG_HSDIV_PRSNT_MAX                    (0x0000FFFFU)

/* PLL12_LOCKKEY0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL12_LOCKKEY0_KEY_MASK                      (0xFFFFFFFEU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_LOCKKEY0_KEY_SHIFT                     (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_LOCKKEY0_KEY_MAX                       (0x7FFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_LOCKKEY0_UNLOCKED_MASK                 (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_LOCKKEY0_UNLOCKED_SHIFT                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_LOCKKEY0_UNLOCKED_MAX                  (0x00000001U)

/* PLL12_LOCKKEY1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL12_LOCKKEY1_LOCKKEY1_VAL_MASK             (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_LOCKKEY1_LOCKKEY1_VAL_SHIFT            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_LOCKKEY1_LOCKKEY1_VAL_MAX              (0xFFFFFFFFU)

/* PLL12_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_DAC_EN_MASK                       (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_DAC_EN_SHIFT                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_DAC_EN_MAX                        (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_DSM_EN_MASK                       (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_DSM_EN_SHIFT                      (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_DSM_EN_MAX                        (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_CLK_POSTDIV_EN_MASK               (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_CLK_POSTDIV_EN_SHIFT              (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_CLK_POSTDIV_EN_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_CLK_4PH_EN_MASK                   (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_CLK_4PH_EN_SHIFT                  (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_CLK_4PH_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_INTL_BYP_EN_MASK                  (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_INTL_BYP_EN_SHIFT                 (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_INTL_BYP_EN_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_PLL_EN_MASK                       (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_PLL_EN_SHIFT                      (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_PLL_EN_MAX                        (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_BYP_ON_LOCKLOSS_MASK              (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_BYP_ON_LOCKLOSS_SHIFT             (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_BYP_ON_LOCKLOSS_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_BYPASS_EN_MASK                    (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_BYPASS_EN_SHIFT                   (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CTRL_BYPASS_EN_MAX                     (0x00000001U)

/* PLL12_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL12_STAT_LOCK_MASK                         (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_STAT_LOCK_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_STAT_LOCK_MAX                          (0x00000001U)

/* PLL12_FREQ_CTRL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL12_FREQ_CTRL0_FB_DIV_INT_MASK             (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_FREQ_CTRL0_FB_DIV_INT_SHIFT            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_FREQ_CTRL0_FB_DIV_INT_MAX              (0x00000FFFU)

/* PLL12_FREQ_CTRL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL12_FREQ_CTRL1_FB_DIV_FRAC_MASK            (0x00FFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_FREQ_CTRL1_FB_DIV_FRAC_SHIFT           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_FREQ_CTRL1_FB_DIV_FRAC_MAX             (0x00FFFFFFU)

/* PLL12_DIV_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL12_DIV_CTRL_REF_DIV_MASK                  (0x0000003FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_DIV_CTRL_REF_DIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_DIV_CTRL_REF_DIV_MAX                   (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_DIV_CTRL_POST_DIV1_MASK                (0x00070000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_DIV_CTRL_POST_DIV1_SHIFT               (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_DIV_CTRL_POST_DIV1_MAX                 (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_DIV_CTRL_POST_DIV2_MASK                (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_DIV_CTRL_POST_DIV2_SHIFT               (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_DIV_CTRL_POST_DIV2_MAX                 (0x00000007U)

/* PLL12_SS_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_CTRL_WAVE_SEL_MASK                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_CTRL_WAVE_SEL_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_CTRL_WAVE_SEL_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_CTRL_DOWNSPREAD_EN_MASK             (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_CTRL_DOWNSPREAD_EN_SHIFT            (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_CTRL_DOWNSPREAD_EN_MAX              (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_CTRL_RESET_MASK                     (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_CTRL_RESET_SHIFT                    (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_CTRL_RESET_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_CTRL_WV_TBLE_MAXADDR_MASK           (0x03FC0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_CTRL_WV_TBLE_MAXADDR_SHIFT          (0x00000012U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_CTRL_WV_TBLE_MAXADDR_MAX            (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_CTRL_BYPASS_EN_MASK                 (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_CTRL_BYPASS_EN_SHIFT                (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_CTRL_BYPASS_EN_MAX                  (0x00000001U)

/* PLL12_SS_SPREAD */

#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_SPREAD_SPREAD_MASK                  (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_SPREAD_SPREAD_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_SPREAD_SPREAD_MAX                   (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_SPREAD_MOD_DIV_MASK                 (0x000F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_SPREAD_MOD_DIV_SHIFT                (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_SS_SPREAD_MOD_DIV_MAX                  (0x0000000FU)

/* PLL12_CAL_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_CTRL_CAL_IN_MASK                   (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_CTRL_CAL_IN_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_CTRL_CAL_IN_MAX                    (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_CTRL_CAL_BYP_MASK                  (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_CTRL_CAL_BYP_SHIFT                 (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_CTRL_CAL_BYP_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_CTRL_CAL_CNT_MASK                  (0x00070000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_CTRL_CAL_CNT_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_CTRL_CAL_CNT_MAX                   (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_CTRL_FAST_CAL_MASK                 (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_CTRL_FAST_CAL_SHIFT                (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_CTRL_FAST_CAL_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_CTRL_CAL_EN_MASK                   (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_CTRL_CAL_EN_SHIFT                  (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_CTRL_CAL_EN_MAX                    (0x00000001U)

/* PLL12_CAL_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_STAT_CAL_OUT_MASK                  (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_STAT_CAL_OUT_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_STAT_CAL_OUT_MAX                   (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_STAT_LOCK_CNT_MASK                 (0x000F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_STAT_LOCK_CNT_SHIFT                (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_STAT_LOCK_CNT_MAX                  (0x0000000FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_STAT_CAL_LOCK_MASK                 (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_STAT_CAL_LOCK_SHIFT                (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_CAL_STAT_CAL_LOCK_MAX                  (0x00000001U)

/* PLL12_HSDIV_CTRL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL12_HSDIV_CTRL0_HSDIV_MASK                 (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_HSDIV_CTRL0_HSDIV_SHIFT                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_HSDIV_CTRL0_HSDIV_MAX                  (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_HSDIV_CTRL0_SYNC_DIS_MASK              (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_HSDIV_CTRL0_SYNC_DIS_SHIFT             (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_HSDIV_CTRL0_SYNC_DIS_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_HSDIV_CTRL0_CLKOUT_EN_MASK             (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_HSDIV_CTRL0_CLKOUT_EN_SHIFT            (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_HSDIV_CTRL0_CLKOUT_EN_MAX              (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL12_HSDIV_CTRL0_RESET_MASK                 (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_HSDIV_CTRL0_RESET_SHIFT                (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL12_HSDIV_CTRL0_RESET_MAX                  (0x00000001U)

/* PLL14_PID */

#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_MINOR_MASK                         (0x0000003FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_MINOR_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_MINOR_MAX                          (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_CUSTOM_MASK                        (0x000000C0U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_CUSTOM_SHIFT                       (0x00000006U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_CUSTOM_MAX                         (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_MAJOR_MASK                         (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_MAJOR_SHIFT                        (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_MAJOR_MAX                          (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_MISC_MASK                          (0x0000F800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_MISC_SHIFT                         (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_MISC_MAX                           (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_MODULE_MASK                        (0x0FFF0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_MODULE_SHIFT                       (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_MODULE_MAX                         (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_BU_MASK                            (0x30000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_BU_SHIFT                           (0x0000001CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_BU_MAX                             (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_SCHEME_MASK                        (0xC0000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_SCHEME_SHIFT                       (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_PID_SCHEME_MAX                         (0x00000003U)

/* PLL14_CFG */

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CFG_PLL_TYPE_MASK                      (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CFG_PLL_TYPE_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CFG_PLL_TYPE_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CFG_SSM_WVTBL_MASK                     (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CFG_SSM_WVTBL_SHIFT                    (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CFG_SSM_WVTBL_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CFG_SSM_TYPE_MASK                      (0x00001800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CFG_SSM_TYPE_SHIFT                     (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CFG_SSM_TYPE_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CFG_HSDIV_PRSNT_MASK                   (0xFFFF0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CFG_HSDIV_PRSNT_SHIFT                  (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CFG_HSDIV_PRSNT_MAX                    (0x0000FFFFU)

/* PLL14_LOCKKEY0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL14_LOCKKEY0_KEY_MASK                      (0xFFFFFFFEU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_LOCKKEY0_KEY_SHIFT                     (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_LOCKKEY0_KEY_MAX                       (0x7FFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_LOCKKEY0_UNLOCKED_MASK                 (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_LOCKKEY0_UNLOCKED_SHIFT                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_LOCKKEY0_UNLOCKED_MAX                  (0x00000001U)

/* PLL14_LOCKKEY1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL14_LOCKKEY1_LOCKKEY1_VAL_MASK             (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_LOCKKEY1_LOCKKEY1_VAL_SHIFT            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_LOCKKEY1_LOCKKEY1_VAL_MAX              (0xFFFFFFFFU)

/* PLL14_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_DAC_EN_MASK                       (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_DAC_EN_SHIFT                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_DAC_EN_MAX                        (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_DSM_EN_MASK                       (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_DSM_EN_SHIFT                      (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_DSM_EN_MAX                        (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_CLK_POSTDIV_EN_MASK               (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_CLK_POSTDIV_EN_SHIFT              (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_CLK_POSTDIV_EN_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_CLK_4PH_EN_MASK                   (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_CLK_4PH_EN_SHIFT                  (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_CLK_4PH_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_INTL_BYP_EN_MASK                  (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_INTL_BYP_EN_SHIFT                 (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_INTL_BYP_EN_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_PLL_EN_MASK                       (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_PLL_EN_SHIFT                      (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_PLL_EN_MAX                        (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_BYP_ON_LOCKLOSS_MASK              (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_BYP_ON_LOCKLOSS_SHIFT             (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_BYP_ON_LOCKLOSS_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_BYPASS_EN_MASK                    (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_BYPASS_EN_SHIFT                   (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CTRL_BYPASS_EN_MAX                     (0x00000001U)

/* PLL14_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL14_STAT_LOCK_MASK                         (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_STAT_LOCK_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_STAT_LOCK_MAX                          (0x00000001U)

/* PLL14_FREQ_CTRL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL14_FREQ_CTRL0_FB_DIV_INT_MASK             (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_FREQ_CTRL0_FB_DIV_INT_SHIFT            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_FREQ_CTRL0_FB_DIV_INT_MAX              (0x00000FFFU)

/* PLL14_FREQ_CTRL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL14_FREQ_CTRL1_FB_DIV_FRAC_MASK            (0x00FFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_FREQ_CTRL1_FB_DIV_FRAC_SHIFT           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_FREQ_CTRL1_FB_DIV_FRAC_MAX             (0x00FFFFFFU)

/* PLL14_DIV_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL14_DIV_CTRL_REF_DIV_MASK                  (0x0000003FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_DIV_CTRL_REF_DIV_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_DIV_CTRL_REF_DIV_MAX                   (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_DIV_CTRL_POST_DIV1_MASK                (0x00070000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_DIV_CTRL_POST_DIV1_SHIFT               (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_DIV_CTRL_POST_DIV1_MAX                 (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_DIV_CTRL_POST_DIV2_MASK                (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_DIV_CTRL_POST_DIV2_SHIFT               (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_DIV_CTRL_POST_DIV2_MAX                 (0x00000007U)

/* PLL14_SS_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_CTRL_WAVE_SEL_MASK                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_CTRL_WAVE_SEL_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_CTRL_WAVE_SEL_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_CTRL_DOWNSPREAD_EN_MASK             (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_CTRL_DOWNSPREAD_EN_SHIFT            (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_CTRL_DOWNSPREAD_EN_MAX              (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_CTRL_RESET_MASK                     (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_CTRL_RESET_SHIFT                    (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_CTRL_RESET_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_CTRL_WV_TBLE_MAXADDR_MASK           (0x03FC0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_CTRL_WV_TBLE_MAXADDR_SHIFT          (0x00000012U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_CTRL_WV_TBLE_MAXADDR_MAX            (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_CTRL_BYPASS_EN_MASK                 (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_CTRL_BYPASS_EN_SHIFT                (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_CTRL_BYPASS_EN_MAX                  (0x00000001U)

/* PLL14_SS_SPREAD */

#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_SPREAD_SPREAD_MASK                  (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_SPREAD_SPREAD_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_SPREAD_SPREAD_MAX                   (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_SPREAD_MOD_DIV_MASK                 (0x000F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_SPREAD_MOD_DIV_SHIFT                (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_SS_SPREAD_MOD_DIV_MAX                  (0x0000000FU)

/* PLL14_CAL_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_CTRL_CAL_IN_MASK                   (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_CTRL_CAL_IN_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_CTRL_CAL_IN_MAX                    (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_CTRL_CAL_BYP_MASK                  (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_CTRL_CAL_BYP_SHIFT                 (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_CTRL_CAL_BYP_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_CTRL_CAL_CNT_MASK                  (0x00070000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_CTRL_CAL_CNT_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_CTRL_CAL_CNT_MAX                   (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_CTRL_FAST_CAL_MASK                 (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_CTRL_FAST_CAL_SHIFT                (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_CTRL_FAST_CAL_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_CTRL_CAL_EN_MASK                   (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_CTRL_CAL_EN_SHIFT                  (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_CTRL_CAL_EN_MAX                    (0x00000001U)

/* PLL14_CAL_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_STAT_CAL_OUT_MASK                  (0x00000FFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_STAT_CAL_OUT_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_STAT_CAL_OUT_MAX                   (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_STAT_LOCK_CNT_MASK                 (0x000F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_STAT_LOCK_CNT_SHIFT                (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_STAT_LOCK_CNT_MAX                  (0x0000000FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_STAT_CAL_LOCK_MASK                 (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_STAT_CAL_LOCK_SHIFT                (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_CAL_STAT_CAL_LOCK_MAX                  (0x00000001U)

/* PLL14_HSDIV_CTRL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL0_HSDIV_MASK                 (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL0_HSDIV_SHIFT                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL0_HSDIV_MAX                  (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL0_SYNC_DIS_MASK              (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL0_SYNC_DIS_SHIFT             (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL0_SYNC_DIS_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL0_CLKOUT_EN_MASK             (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL0_CLKOUT_EN_SHIFT            (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL0_CLKOUT_EN_MAX              (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL0_RESET_MASK                 (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL0_RESET_SHIFT                (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL0_RESET_MAX                  (0x00000001U)

/* PLL14_HSDIV_CTRL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL1_HSDIV_MASK                 (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL1_HSDIV_SHIFT                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL1_HSDIV_MAX                  (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL1_SYNC_DIS_MASK              (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL1_SYNC_DIS_SHIFT             (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL1_SYNC_DIS_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL1_CLKOUT_EN_MASK             (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL1_CLKOUT_EN_SHIFT            (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL1_CLKOUT_EN_MAX              (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL1_RESET_MASK                 (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL1_RESET_SHIFT                (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL14_HSDIV_CTRL1_RESET_MAX                  (0x00000001U)

#ifdef __cplusplus
}
#endif
#endif
