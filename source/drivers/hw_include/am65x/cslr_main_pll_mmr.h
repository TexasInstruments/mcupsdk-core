/********************************************************************
* Copyright (C) 2024 Texas Instruments Incorporated.
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

#define CSL_MAIN_PLL_MMR_CFG_REGS_BASE                                         (0x00000000U)


/**************************************************************************
* Hardware Region  : PLLCTLR MMRs
**************************************************************************/


/**************************************************************************
* Register Overlay Structure
**************************************************************************/


typedef struct {
    volatile uint32_t PID;
    volatile uint8_t  Resv_8[4];
    volatile uint32_t PLL_MMR_CFG0;
    volatile uint32_t PLL_MMR_CFG1;
    volatile uint32_t PLL0_KICK0;                /* LOCK register - KICK0 component */
    volatile uint32_t PLL0_KICK1;                /* LOCK register - KICK1 component */
    volatile uint8_t  Resv_32[8];
    volatile uint32_t PLL0_PLL_FREQ_CTL0;        /* PLL_FREQ_CTL0 register for PLL0 */
    volatile uint32_t PLL0_PLL_FREQ_CTL1;        /* PLL_FREQ_CTL1 register for PLL0 */
    volatile uint32_t PLL0_PLL_CLK_DIV;          /* PLL_CLK_DIV register for PLL0 */
    volatile uint32_t PLL0_PLL_PROG;             /* PLL_PROG register for PLL0 */
    volatile uint32_t PLL0_PLL_CTRL;             /* PLL_CTRL register for PLL0 */
    volatile uint32_t PLL0_PLL_STAT;             /* PLL_STAT register for PLL0 */
    volatile uint32_t PLL0_PLL_PWR_CTRL;         /* PLL_PWR_CTRL register for PLL0 */
    volatile uint32_t PLL0_PLL_PWR_STAT;         /* PLL_PWR_STAT register for PLL0 */
    volatile uint32_t PLL0_PLL_SS_SPREAD;        /* PLL_SS_SPREAD register for PLL0 */
    volatile uint32_t PLL0_PLL_SS_MODFREQ;       /* PLL_SS_MODFREQ register for PLL0 */
    volatile uint8_t  Resv_288[216];
    volatile uint32_t PLL0_HSDIV_CLKDIV;         /* HSDIV_CLKDIV register for PLL0 */
    volatile uint32_t PLL0_HSDIV_CTRL;           /* HSDIV_CTRL register for PLL0 */
    volatile uint32_t PLL0_HSDIV_STAT;           /* HSDIV_STAT register for PLL0 */
    volatile uint32_t PLL0_HSDIV_PWR_CTRL;       /* HSDIV_PWR_CTRL register for PLL0 */
    volatile uint32_t PLL0_HSDIV_PWR_STAT;       /* HSDIV_PWR_STAT register for PLL0 */
    volatile uint8_t  Resv_4112[3804];
    volatile uint32_t PLL1_KICK0;                /* LOCK register - KICK0 component */
    volatile uint32_t PLL1_KICK1;                /* LOCK register - KICK1 component */
    volatile uint8_t  Resv_4128[8];
    volatile uint32_t PLL1_PLL_FREQ_CTL0;        /* PLL_FREQ_CTL0 register for PLL1 */
    volatile uint32_t PLL1_PLL_FREQ_CTL1;        /* PLL_FREQ_CTL1 register for PLL1 */
    volatile uint32_t PLL1_PLL_CLK_DIV;          /* PLL_CLK_DIV register for PLL1 */
    volatile uint32_t PLL1_PLL_PROG;             /* PLL_PROG register for PLL1 */
    volatile uint32_t PLL1_PLL_CTRL;             /* PLL_CTRL register for PLL1 */
    volatile uint32_t PLL1_PLL_STAT;             /* PLL_STAT register for PLL1 */
    volatile uint32_t PLL1_PLL_PWR_CTRL;         /* PLL_PWR_CTRL register for PLL1 */
    volatile uint32_t PLL1_PLL_PWR_STAT;         /* PLL_PWR_STAT register for PLL1 */
    volatile uint32_t PLL1_PLL_SS_SPREAD;        /* PLL_SS_SPREAD register for PLL1 */
    volatile uint32_t PLL1_PLL_SS_MODFREQ;       /* PLL_SS_MODFREQ register for PLL1 */
    volatile uint8_t  Resv_8208[4040];
    volatile uint32_t PLL2_KICK0;                /* LOCK register - KICK0 component */
    volatile uint32_t PLL2_KICK1;                /* LOCK register - KICK1 component */
    volatile uint8_t  Resv_8224[8];
    volatile uint32_t PLL2_PLL_FREQ_CTL0;        /* PLL_FREQ_CTL0 register for PLL2 */
    volatile uint32_t PLL2_PLL_FREQ_CTL1;        /* PLL_FREQ_CTL1 register for PLL2 */
    volatile uint32_t PLL2_PLL_CLK_DIV;          /* PLL_CLK_DIV register for PLL2 */
    volatile uint32_t PLL2_PLL_PROG;             /* PLL_PROG register for PLL2 */
    volatile uint32_t PLL2_PLL_CTRL;             /* PLL_CTRL register for PLL2 */
    volatile uint32_t PLL2_PLL_STAT;             /* PLL_STAT register for PLL2 */
    volatile uint32_t PLL2_PLL_PWR_CTRL;         /* PLL_PWR_CTRL register for PLL2 */
    volatile uint32_t PLL2_PLL_PWR_STAT;         /* PLL_PWR_STAT register for PLL2 */
    volatile uint32_t PLL2_PLL_SS_SPREAD;        /* PLL_SS_SPREAD register for PLL2 */
    volatile uint32_t PLL2_PLL_SS_MODFREQ;       /* PLL_SS_MODFREQ register for PLL2 */
    volatile uint8_t  Resv_8480[216];
    volatile uint32_t PLL2_HSDIV_CLKDIV;         /* HSDIV_CLKDIV register for PLL2 */
    volatile uint32_t PLL2_HSDIV_CTRL;           /* HSDIV_CTRL register for PLL2 */
    volatile uint32_t PLL2_HSDIV_STAT;           /* HSDIV_STAT register for PLL2 */
    volatile uint32_t PLL2_HSDIV_PWR_CTRL;       /* HSDIV_PWR_CTRL register for PLL2 */
    volatile uint32_t PLL2_HSDIV_PWR_STAT;       /* HSDIV_PWR_STAT register for PLL2 */
    volatile uint8_t  Resv_12304[3804];
    volatile uint32_t PLL3_KICK0;                /* LOCK register - KICK0 component */
    volatile uint32_t PLL3_KICK1;                /* LOCK register - KICK1 component */
    volatile uint8_t  Resv_12320[8];
    volatile uint32_t PLL3_PLL_FREQ_CTL0;        /* PLL_FREQ_CTL0 register for PLL3 */
    volatile uint32_t PLL3_PLL_FREQ_CTL1;        /* PLL_FREQ_CTL1 register for PLL3 */
    volatile uint32_t PLL3_PLL_CLK_DIV;          /* PLL_CLK_DIV register for PLL3 */
    volatile uint32_t PLL3_PLL_PROG;             /* PLL_PROG register for PLL3 */
    volatile uint32_t PLL3_PLL_CTRL;             /* PLL_CTRL register for PLL3 */
    volatile uint32_t PLL3_PLL_STAT;             /* PLL_STAT register for PLL3 */
    volatile uint32_t PLL3_PLL_PWR_CTRL;         /* PLL_PWR_CTRL register for PLL3 */
    volatile uint32_t PLL3_PLL_PWR_STAT;         /* PLL_PWR_STAT register for PLL3 */
    volatile uint32_t PLL3_PLL_SS_SPREAD;        /* PLL_SS_SPREAD register for PLL3 */
    volatile uint32_t PLL3_PLL_SS_MODFREQ;       /* PLL_SS_MODFREQ register for PLL3 */
    volatile uint8_t  Resv_16400[4040];
    volatile uint32_t PLL4_KICK0;                /* LOCK register - KICK0 component */
    volatile uint32_t PLL4_KICK1;                /* LOCK register - KICK1 component */
    volatile uint8_t  Resv_16416[8];
    volatile uint32_t PLL4_PLL_FREQ_CTL0;        /* PLL_FREQ_CTL0 register for PLL4 */
    volatile uint32_t PLL4_PLL_FREQ_CTL1;        /* PLL_FREQ_CTL1 register for PLL4 */
    volatile uint32_t PLL4_PLL_CLK_DIV;          /* PLL_CLK_DIV register for PLL4 */
    volatile uint32_t PLL4_PLL_PROG;             /* PLL_PROG register for PLL4 */
    volatile uint32_t PLL4_PLL_CTRL;             /* PLL_CTRL register for PLL4 */
    volatile uint32_t PLL4_PLL_STAT;             /* PLL_STAT register for PLL4 */
    volatile uint32_t PLL4_PLL_PWR_CTRL;         /* PLL_PWR_CTRL register for PLL4 */
    volatile uint32_t PLL4_PLL_PWR_STAT;         /* PLL_PWR_STAT register for PLL4 */
    volatile uint32_t PLL4_PLL_SS_SPREAD;        /* PLL_SS_SPREAD register for PLL4 */
    volatile uint32_t PLL4_PLL_SS_MODFREQ;       /* PLL_SS_MODFREQ register for PLL4 */
    volatile uint8_t  Resv_20496[4040];
    volatile uint32_t PLL5_KICK0;                /* LOCK register - KICK0 component */
    volatile uint32_t PLL5_KICK1;                /* LOCK register - KICK1 component */
    volatile uint8_t  Resv_20512[8];
    volatile uint32_t PLL5_PLL_FREQ_CTL0;        /* PLL_FREQ_CTL0 register for PLL5 */
    volatile uint32_t PLL5_PLL_FREQ_CTL1;        /* PLL_FREQ_CTL1 register for PLL5 */
    volatile uint32_t PLL5_PLL_CLK_DIV;          /* PLL_CLK_DIV register for PLL5 */
    volatile uint32_t PLL5_PLL_PROG;             /* PLL_PROG register for PLL5 */
    volatile uint32_t PLL5_PLL_CTRL;             /* PLL_CTRL register for PLL5 */
    volatile uint32_t PLL5_PLL_STAT;             /* PLL_STAT register for PLL5 */
    volatile uint32_t PLL5_PLL_PWR_CTRL;         /* PLL_PWR_CTRL register for PLL5 */
    volatile uint32_t PLL5_PLL_PWR_STAT;         /* PLL_PWR_STAT register for PLL5 */
    volatile uint32_t PLL5_PLL_SS_SPREAD;        /* PLL_SS_SPREAD register for PLL5 */
    volatile uint32_t PLL5_PLL_SS_MODFREQ;       /* PLL_SS_MODFREQ register for PLL5 */
    volatile uint8_t  Resv_24592[4040];
    volatile uint32_t PLL6_KICK0;                /* LOCK register - KICK0 component */
    volatile uint32_t PLL6_KICK1;                /* LOCK register - KICK1 component */
    volatile uint8_t  Resv_24608[8];
    volatile uint32_t PLL6_PLL_FREQ_CTL0;        /* PLL_FREQ_CTL0 register for PLL6 */
    volatile uint32_t PLL6_PLL_FREQ_CTL1;        /* PLL_FREQ_CTL1 register for PLL6 */
    volatile uint32_t PLL6_PLL_CLK_DIV;          /* PLL_CLK_DIV register for PLL6 */
    volatile uint32_t PLL6_PLL_PROG;             /* PLL_PROG register for PLL6 */
    volatile uint32_t PLL6_PLL_CTRL;             /* PLL_CTRL register for PLL6 */
    volatile uint32_t PLL6_PLL_STAT;             /* PLL_STAT register for PLL6 */
    volatile uint32_t PLL6_PLL_PWR_CTRL;         /* PLL_PWR_CTRL register for PLL6 */
    volatile uint32_t PLL6_PLL_PWR_STAT;         /* PLL_PWR_STAT register for PLL6 */
    volatile uint32_t PLL6_PLL_SS_SPREAD;        /* PLL_SS_SPREAD register for PLL6 */
    volatile uint32_t PLL6_PLL_SS_MODFREQ;       /* PLL_SS_MODFREQ register for PLL6 */
    volatile uint8_t  Resv_28688[4040];
    volatile uint32_t PLL7_KICK0;                /* LOCK register - KICK0 component */
    volatile uint32_t PLL7_KICK1;                /* LOCK register - KICK1 component */
    volatile uint8_t  Resv_28704[8];
    volatile uint32_t PLL7_PLL_FREQ_CTL0;        /* PLL_FREQ_CTL0 register for PLL7 */
    volatile uint32_t PLL7_PLL_FREQ_CTL1;        /* PLL_FREQ_CTL1 register for PLL7 */
    volatile uint32_t PLL7_PLL_CLK_DIV;          /* PLL_CLK_DIV register for PLL7 */
    volatile uint32_t PLL7_PLL_PROG;             /* PLL_PROG register for PLL7 */
    volatile uint32_t PLL7_PLL_CTRL;             /* PLL_CTRL register for PLL7 */
    volatile uint32_t PLL7_PLL_STAT;             /* PLL_STAT register for PLL7 */
    volatile uint32_t PLL7_PLL_PWR_CTRL;         /* PLL_PWR_CTRL register for PLL7 */
    volatile uint32_t PLL7_PLL_PWR_STAT;         /* PLL_PWR_STAT register for PLL7 */
    volatile uint32_t PLL7_PLL_SS_SPREAD;        /* PLL_SS_SPREAD register for PLL7 */
    volatile uint32_t PLL7_PLL_SS_MODFREQ;       /* PLL_SS_MODFREQ register for PLL7 */
} CSL_main_pll_mmr_cfgRegs;


/**************************************************************************
* Register Macros
**************************************************************************/

#define CSL_MAIN_PLL_MMR_CFG_PID                                               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0                                      (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1                                      (0x0000000CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_KICK0                                        (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_KICK1                                        (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL0                                (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL1                                (0x00000024U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CLK_DIV                                  (0x00000028U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PROG                                     (0x0000002CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL                                     (0x00000030U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT                                     (0x00000034U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL                                 (0x00000038U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT                                 (0x0000003CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_SPREAD                                (0x00000040U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_MODFREQ                               (0x00000044U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV                                 (0x00000120U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL                                   (0x00000124U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT                                   (0x00000128U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL                               (0x0000012CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_STAT                               (0x00000130U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_KICK0                                        (0x00001010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_KICK1                                        (0x00001014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL0                                (0x00001020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL1                                (0x00001024U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CLK_DIV                                  (0x00001028U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PROG                                     (0x0000102CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL                                     (0x00001030U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT                                     (0x00001034U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL                                 (0x00001038U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT                                 (0x0000103CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_SPREAD                                (0x00001040U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_MODFREQ                               (0x00001044U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_KICK0                                        (0x00002010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_KICK1                                        (0x00002014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL0                                (0x00002020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL1                                (0x00002024U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CLK_DIV                                  (0x00002028U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PROG                                     (0x0000202CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL                                     (0x00002030U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT                                     (0x00002034U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL                                 (0x00002038U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT                                 (0x0000203CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_SPREAD                                (0x00002040U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_MODFREQ                               (0x00002044U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV                                 (0x00002120U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL                                   (0x00002124U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT                                   (0x00002128U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL                               (0x0000212CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_STAT                               (0x00002130U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_KICK0                                        (0x00003010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_KICK1                                        (0x00003014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL0                                (0x00003020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL1                                (0x00003024U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CLK_DIV                                  (0x00003028U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PROG                                     (0x0000302CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL                                     (0x00003030U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT                                     (0x00003034U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL                                 (0x00003038U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT                                 (0x0000303CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_SPREAD                                (0x00003040U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_MODFREQ                               (0x00003044U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_KICK0                                        (0x00004010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_KICK1                                        (0x00004014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL0                                (0x00004020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL1                                (0x00004024U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CLK_DIV                                  (0x00004028U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PROG                                     (0x0000402CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL                                     (0x00004030U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT                                     (0x00004034U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL                                 (0x00004038U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT                                 (0x0000403CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_SPREAD                                (0x00004040U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_MODFREQ                               (0x00004044U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_KICK0                                        (0x00005010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_KICK1                                        (0x00005014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL0                                (0x00005020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL1                                (0x00005024U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CLK_DIV                                  (0x00005028U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PROG                                     (0x0000502CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL                                     (0x00005030U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT                                     (0x00005034U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL                                 (0x00005038U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT                                 (0x0000503CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_SPREAD                                (0x00005040U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_MODFREQ                               (0x00005044U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_KICK0                                        (0x00006010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_KICK1                                        (0x00006014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL0                                (0x00006020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL1                                (0x00006024U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CLK_DIV                                  (0x00006028U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PROG                                     (0x0000602CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL                                     (0x00006030U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT                                     (0x00006034U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL                                 (0x00006038U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT                                 (0x0000603CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_SPREAD                                (0x00006040U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_MODFREQ                               (0x00006044U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_KICK0                                        (0x00007010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_KICK1                                        (0x00007014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL0                                (0x00007020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL1                                (0x00007024U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CLK_DIV                                  (0x00007028U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PROG                                     (0x0000702CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL                                     (0x00007030U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT                                     (0x00007034U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL                                 (0x00007038U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT                                 (0x0000703CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_SPREAD                                (0x00007040U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_MODFREQ                               (0x00007044U)

/**************************************************************************
* Field Definition Macros
**************************************************************************/


/* PID */

#define CSL_MAIN_PLL_MMR_CFG_PID_MINOR_MASK                                    (0x0000003FU)
#define CSL_MAIN_PLL_MMR_CFG_PID_MINOR_SHIFT                                   (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PID_MINOR_RESETVAL                                (0x00000006U)
#define CSL_MAIN_PLL_MMR_CFG_PID_MINOR_MAX                                     (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PID_CUSTOM_MASK                                   (0x000000C0U)
#define CSL_MAIN_PLL_MMR_CFG_PID_CUSTOM_SHIFT                                  (0x00000006U)
#define CSL_MAIN_PLL_MMR_CFG_PID_CUSTOM_RESETVAL                               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PID_CUSTOM_MAX                                    (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PID_MAJOR_MASK                                    (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PID_MAJOR_SHIFT                                   (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PID_MAJOR_RESETVAL                                (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PID_MAJOR_MAX                                     (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PID_MISC_MASK                                     (0x0000F800U)
#define CSL_MAIN_PLL_MMR_CFG_PID_MISC_SHIFT                                    (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PID_MISC_RESETVAL                                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PID_MISC_MAX                                      (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PID_MODULE_MASK                                   (0x0FFF0000U)
#define CSL_MAIN_PLL_MMR_CFG_PID_MODULE_SHIFT                                  (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PID_MODULE_RESETVAL                               (0x00000182U)
#define CSL_MAIN_PLL_MMR_CFG_PID_MODULE_MAX                                    (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PID_BU_MASK                                       (0x30000000U)
#define CSL_MAIN_PLL_MMR_CFG_PID_BU_SHIFT                                      (0x0000001CU)
#define CSL_MAIN_PLL_MMR_CFG_PID_BU_RESETVAL                                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PID_BU_MAX                                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PID_SCHEME_MASK                                   (0xC0000000U)
#define CSL_MAIN_PLL_MMR_CFG_PID_SCHEME_SHIFT                                  (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PID_SCHEME_RESETVAL                               (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PID_SCHEME_MAX                                    (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PID_RESETVAL                                      (0x61820206U)

/* PLL_MMR_CFG0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL0_TYPE_MASK                       (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL0_TYPE_SHIFT                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL0_TYPE_RESETVAL                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL0_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL1_TYPE_MASK                       (0x0000000CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL1_TYPE_SHIFT                      (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL1_TYPE_RESETVAL                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL1_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL2_TYPE_MASK                       (0x00000030U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL2_TYPE_SHIFT                      (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL2_TYPE_RESETVAL                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL2_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL3_TYPE_MASK                       (0x000000C0U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL3_TYPE_SHIFT                      (0x00000006U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL3_TYPE_RESETVAL                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL3_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL4_TYPE_MASK                       (0x00000300U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL4_TYPE_SHIFT                      (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL4_TYPE_RESETVAL                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL4_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL5_TYPE_MASK                       (0x00000C00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL5_TYPE_SHIFT                      (0x0000000AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL5_TYPE_RESETVAL                   (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL5_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL6_TYPE_MASK                       (0x00003000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL6_TYPE_SHIFT                      (0x0000000CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL6_TYPE_RESETVAL                   (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL6_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL7_TYPE_MASK                       (0x0000C000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL7_TYPE_SHIFT                      (0x0000000EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL7_TYPE_RESETVAL                   (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL7_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL8_TYPE_MASK                       (0x00030000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL8_TYPE_SHIFT                      (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL8_TYPE_RESETVAL                   (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL8_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL9_TYPE_MASK                       (0x000C0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL9_TYPE_SHIFT                      (0x00000012U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL9_TYPE_RESETVAL                   (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL9_TYPE_MAX                        (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL10_TYPE_MASK                      (0x00300000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL10_TYPE_SHIFT                     (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL10_TYPE_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL10_TYPE_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL11_TYPE_MASK                      (0x00C00000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL11_TYPE_SHIFT                     (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL11_TYPE_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL11_TYPE_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL12_TYPE_MASK                      (0x03000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL12_TYPE_SHIFT                     (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL12_TYPE_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL12_TYPE_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL13_TYPE_MASK                      (0x0C000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL13_TYPE_SHIFT                     (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL13_TYPE_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL13_TYPE_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL14_TYPE_MASK                      (0x30000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL14_TYPE_SHIFT                     (0x0000001CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL14_TYPE_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL14_TYPE_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL15_TYPE_MASK                      (0xC0000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL15_TYPE_SHIFT                     (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL15_TYPE_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_PLL15_TYPE_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG0_RESETVAL                             (0x000052AAU)

/* PLL_MMR_CFG1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL0_HSDIV_MASK                      (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL0_HSDIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL0_HSDIV_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL0_HSDIV_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL1_HSDIV_MASK                      (0x0000000CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL1_HSDIV_SHIFT                     (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL1_HSDIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL1_HSDIV_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL2_HSDIV_MASK                      (0x00000030U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL2_HSDIV_SHIFT                     (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL2_HSDIV_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL2_HSDIV_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL3_HSDIV_MASK                      (0x000000C0U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL3_HSDIV_SHIFT                     (0x00000006U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL3_HSDIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL3_HSDIV_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL4_HSDIV_MASK                      (0x00000300U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL4_HSDIV_SHIFT                     (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL4_HSDIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL4_HSDIV_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL5_HSDIV_MASK                      (0x00000C00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL5_HSDIV_SHIFT                     (0x0000000AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL5_HSDIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL5_HSDIV_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL6_HSDIV_MASK                      (0x00003000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL6_HSDIV_SHIFT                     (0x0000000CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL6_HSDIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL6_HSDIV_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL7_HSDIV_MASK                      (0x0000C000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL7_HSDIV_SHIFT                     (0x0000000EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL7_HSDIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL7_HSDIV_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL8_HSDIV_MASK                      (0x00030000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL8_HSDIV_SHIFT                     (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL8_HSDIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL8_HSDIV_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL9_HSDIV_MASK                      (0x000C0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL9_HSDIV_SHIFT                     (0x00000012U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL9_HSDIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL9_HSDIV_MAX                       (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL10_HSDIV_MASK                     (0x00300000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL10_HSDIV_SHIFT                    (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL10_HSDIV_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL10_HSDIV_MAX                      (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL11_HSDIV_MASK                     (0x00C00000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL11_HSDIV_SHIFT                    (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL11_HSDIV_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL11_HSDIV_MAX                      (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL12_HSDIV_MASK                     (0x03000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL12_HSDIV_SHIFT                    (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL12_HSDIV_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL12_HSDIV_MAX                      (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL13_HSDIV_MASK                     (0x0C000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL13_HSDIV_SHIFT                    (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL13_HSDIV_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL13_HSDIV_MAX                      (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL14_HSDIV_MASK                     (0x30000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL14_HSDIV_SHIFT                    (0x0000001CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL14_HSDIV_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL14_HSDIV_MAX                      (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL15_HSDIV_MASK                     (0xC0000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL15_HSDIV_SHIFT                    (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL15_HSDIV_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_PLL15_HSDIV_MAX                      (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL_MMR_CFG1_RESETVAL                             (0x00000011U)

/* PLL0_KICK0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_KICK0_KICK0_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_KICK0_KICK0_VAL_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_KICK0_KICK0_VAL_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_KICK0_KICK0_VAL_MAX                          (0xFFFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_KICK0_RESETVAL                               (0x00000000U)

/* PLL0_KICK1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_KICK1_KICK1_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_KICK1_KICK1_VAL_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_KICK1_KICK1_VAL_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_KICK1_KICK1_VAL_MAX                          (0xFFFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_KICK1_RESETVAL                               (0x00000000U)

/* PLL0_PLL_FREQ_CTL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL0_NDIV_MASK                      (0x000000FFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL0_NDIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL0_NDIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL0_NDIV_MAX                       (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL0_M_INT_MULT_MASK                (0x000FFF00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL0_M_INT_MULT_SHIFT               (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL0_M_INT_MULT_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL0_M_INT_MULT_MAX                 (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL0_SD_DIV_MASK                    (0xFF000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL0_SD_DIV_SHIFT                   (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL0_SD_DIV_RESETVAL                (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL0_SD_DIV_MAX                     (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL0_RESETVAL                       (0x02000000U)

/* PLL0_PLL_FREQ_CTL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL1_M_FRAC_MULT_MASK               (0x0003FFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL1_M_FRAC_MULT_SHIFT              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL1_M_FRAC_MULT_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL1_M_FRAC_MULT_MAX                (0x0003FFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL1_SEL_FREQ_DCO_MASK              (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL1_SEL_FREQ_DCO_SHIFT             (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL1_SEL_FREQ_DCO_RESETVAL          (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL1_SEL_FREQ_DCO_MAX               (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_FREQ_CTL1_RESETVAL                       (0x02000000U)

/* PLL0_PLL_CLK_DIV */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CLK_DIV_M1_DIV_MASK                      (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CLK_DIV_M1_DIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CLK_DIV_M1_DIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CLK_DIV_M1_DIV_MAX                       (0x0000000FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CLK_DIV_M2_DIV_MASK                      (0x00007F00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CLK_DIV_M2_DIV_SHIFT                     (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CLK_DIV_M2_DIV_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CLK_DIV_M2_DIV_MAX                       (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CLK_DIV_M3_DIV_MASK                      (0x001F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CLK_DIV_M3_DIV_SHIFT                     (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CLK_DIV_M3_DIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CLK_DIV_M3_DIV_MAX                       (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CLK_DIV_RESETVAL                         (0x00000100U)

/* PLL0_PLL_PROG */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PROG_TINIT_Z_MASK                        (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PROG_TINIT_Z_SHIFT                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PROG_TINIT_Z_RESETVAL                    (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PROG_TINIT_Z_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PROG_TENABLE_MASK                        (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PROG_TENABLE_SHIFT                       (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PROG_TENABLE_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PROG_TENABLE_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PROG_TENABLEDIV_MASK                     (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PROG_TENABLEDIV_SHIFT                    (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PROG_TENABLEDIV_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PROG_TENABLEDIV_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PROG_RESETVAL                            (0x00000001U)

/* PLL0_PLL_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUTBYPASS_EN_MASK                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUTBYPASS_EN_SHIFT               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUTBYPASS_EN_RESETVAL            (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUTBYPASS_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUTHIF_EN_MASK                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUTHIF_EN_SHIFT                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUTHIF_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUTHIF_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUTX2_EN_MASK                    (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUTX2_EN_SHIFT                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUTX2_EN_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUTX2_EN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUT_EN_MASK                      (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUT_EN_SHIFT                     (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUT_EN_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUT_EN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKDCOLDO_EN_MASK                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKDCOLDO_EN_SHIFT                  (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKDCOLDO_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKDCOLDO_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUTLDO_EN_MASK                   (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUTLDO_EN_SHIFT                  (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUTLDO_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKOUTLDO_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_IDLE_MASK                           (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_IDLE_SHIFT                          (0x00000007U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_IDLE_RESETVAL                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_IDLE_MAX                            (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKRAMP_RATE_MASK                   (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKRAMP_RATE_SHIFT                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKRAMP_RATE_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKRAMP_RATE_MAX                    (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKRAMP_LEVEL_MASK                  (0x00003000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKRAMP_LEVEL_SHIFT                 (0x0000000CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKRAMP_LEVEL_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKRAMP_LEVEL_MAX                   (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_ULOWCLK_EN_MASK                     (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_ULOWCLK_EN_SHIFT                    (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_ULOWCLK_EN_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_ULOWCLK_EN_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_DRIFTGUARD_EN_MASK                  (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_DRIFTGUARD_EN_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_DRIFTGUARD_EN_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_DRIFTGUARD_EN_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_REGM4X_EN_MASK                      (0x00020000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_REGM4X_EN_SHIFT                     (0x00000011U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_REGM4X_EN_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_REGM4X_EN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKINPHIF_SEL_MASK                  (0x00040000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKINPHIF_SEL_SHIFT                 (0x00000012U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKINPHIF_SEL_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CLKINPHIF_SEL_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_RELOCK_RAMP_EN_MASK                 (0x00080000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_RELOCK_RAMP_EN_SHIFT                (0x00000013U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_RELOCK_RAMP_EN_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_RELOCK_RAMP_EN_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CYCLESLIP_EN_MASK                   (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CYCLESLIP_EN_SHIFT                  (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CYCLESLIP_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_CYCLESLIP_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_LOWCURRSTBY_MASK                    (0x00200000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_LOWCURRSTBY_SHIFT                   (0x00000015U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_LOWCURRSTBY_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_LOWCURRSTBY_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_RELAXED_LOCK_MASK                   (0x00400000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_RELAXED_LOCK_SHIFT                  (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_RELAXED_LOCK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_RELAXED_LOCK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_LPMODE_MASK                         (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_LPMODE_SHIFT                        (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_LPMODE_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_LPMODE_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_BW_CONTROL_MASK                     (0x03000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_BW_CONTROL_SHIFT                    (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_BW_CONTROL_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_BW_CONTROL_MAX                      (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_BW_INCR_DECRZ_MASK                  (0x04000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_BW_INCR_DECRZ_SHIFT                 (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_BW_INCR_DECRZ_RESETVAL              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_BW_INCR_DECRZ_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_DC_CORRECTOR_EN_MASK                (0x08000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_DC_CORRECTOR_EN_SHIFT               (0x0000001BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_DC_CORRECTOR_EN_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_DC_CORRECTOR_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_DOWNSPREAD_MASK                     (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_DOWNSPREAD_SHIFT                    (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_DOWNSPREAD_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_DOWNSPREAD_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_SSC_EN_MASK                         (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_SSC_EN_SHIFT                        (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_SSC_EN_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_SSC_EN_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_CTRL_RESETVAL                            (0x04000009U)

/* PLL0_PLL_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUTBYPASS_EN_ACK_MASK            (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUTBYPASS_EN_ACK_SHIFT           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUTBYPASS_EN_ACK_RESETVAL        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUTBYPASS_EN_ACK_MAX             (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUTHIF_EN_ACK_MASK               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUTHIF_EN_ACK_SHIFT              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUTHIF_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUTHIF_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUTX2_EN_ACK_MASK                (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUTX2_EN_ACK_SHIFT               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUTX2_EN_ACK_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUTX2_EN_ACK_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUT_EN_ACK_MASK                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUT_EN_ACK_SHIFT                 (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUT_EN_ACK_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUT_EN_ACK_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKDCOLDO_EN_ACK_MASK               (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKDCOLDO_EN_ACK_SHIFT              (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKDCOLDO_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKDCOLDO_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUTLDO_EN_ACK_MASK               (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUTLDO_EN_ACK_SHIFT              (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUTLDO_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_CLKOUTLDO_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_BYPASS_ACK_MASK                     (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_BYPASS_ACK_SHIFT                    (0x00000007U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_BYPASS_ACK_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_BYPASS_ACK_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_M1CHANGE_ACK_MASK                   (0x00000200U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_M1CHANGE_ACK_SHIFT                  (0x00000009U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_M1CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_M1CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_M2CHANGE_ACK_MASK                   (0x00000400U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_M2CHANGE_ACK_SHIFT                  (0x0000000AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_M2CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_M2CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_M3CHANGE_ACK_MASK                   (0x00000800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_M3CHANGE_ACK_SHIFT                  (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_M3CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_M3CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_BYPASS_CLKTYPE_MASK                 (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_BYPASS_CLKTYPE_SHIFT                (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_BYPASS_CLKTYPE_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_BYPASS_CLKTYPE_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_TVALID_MASK                         (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_TVALID_SHIFT                        (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_TVALID_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_TVALID_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_LOCK2_MASK                          (0x00400000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_LOCK2_SHIFT                         (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_LOCK2_RESETVAL                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_LOCK2_MAX                           (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_HIGHJITTER_MASK                     (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_HIGHJITTER_SHIFT                    (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_HIGHJITTER_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_HIGHJITTER_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_PHASELOCK_MASK                      (0x01000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_PHASELOCK_SHIFT                     (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_PHASELOCK_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_PHASELOCK_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_FREQLOCK_MASK                       (0x02000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_FREQLOCK_SHIFT                      (0x00000019U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_FREQLOCK_RESETVAL                   (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_FREQLOCK_MAX                        (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_LOSSREF_MASK                        (0x04000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_LOSSREF_SHIFT                       (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_LOSSREF_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_LOSSREF_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_RECAL_MASK                          (0x08000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_RECAL_SHIFT                         (0x0000001BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_RECAL_RESETVAL                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_RECAL_MAX                           (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_SSC_ACK_MASK                        (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_SSC_ACK_SHIFT                       (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_SSC_ACK_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_SSC_ACK_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_STAT_RESETVAL                            (0x00000000U)

/* PLL0_PLL_PWR_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISO_PRE_Z_MASK                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISO_PRE_Z_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISO_PRE_Z_RESETVAL              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISO_PRE_Z_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISOSCAN_PRE_Z_MASK              (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISOSCAN_PRE_Z_SHIFT             (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISOSCAN_PRE_Z_RESETVAL          (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISOSCAN_PRE_Z_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_OFFMODE_MASK                    (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_OFFMODE_SHIFT                   (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_OFFMODE_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_OFFMODE_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISO_CLR_MASK                    (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISO_CLR_SHIFT                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISO_CLR_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISO_CLR_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISOSCAN_CLR_MASK                (0x00020000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISOSCAN_CLR_SHIFT               (0x00000011U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISOSCAN_CLR_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISOSCAN_CLR_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISORET_MASK                     (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISORET_SHIFT                    (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISORET_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISORET_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISO_SCAN_MASK                   (0x00200000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISO_SCAN_SHIFT                  (0x00000015U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISO_SCAN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_ISO_SCAN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_RET_MASK                        (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_RET_SHIFT                       (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_RET_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_RET_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_PGOODIN_MASK                    (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_PGOODIN_SHIFT                   (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_PGOODIN_RESETVAL                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_PGOODIN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_PONIN_MASK                      (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_PONIN_SHIFT                     (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_PONIN_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_PONIN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_CTRL_RESETVAL                        (0xC0000003U)

/* PLL0_PLL_PWR_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_LDOPWDN_MASK                    (0x00004000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_LDOPWDN_SHIFT                   (0x0000000EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_LDOPWDN_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_LDOPWDN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_TICOPWDN_MASK                   (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_TICOPWDN_SHIFT                  (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_TICOPWDN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_TICOPWDN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_PGOODOUT_MASK                   (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_PGOODOUT_SHIFT                  (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_PGOODOUT_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_PGOODOUT_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_PONOUT_MASK                     (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_PONOUT_SHIFT                    (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_PONOUT_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_PONOUT_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_PWR_STAT_RESETVAL                        (0x00000000U)

/* PLL0_PLL_SS_SPREAD */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_MASK          (0x0003FFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_SHIFT         (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_RESETVAL      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_MAX           (0x0003FFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_SPREAD_DELTA_MSTEP_INT_MASK           (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_SPREAD_DELTA_MSTEP_INT_SHIFT          (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_SPREAD_DELTA_MSTEP_INT_RESETVAL       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_SPREAD_DELTA_MSTEP_INT_MAX            (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_SPREAD_RESETVAL                       (0x00000000U)

/* PLL0_PLL_SS_MODFREQ */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_MASK         (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_SHIFT        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_RESETVAL     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_MAX          (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_MASK          (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_SHIFT         (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_RESETVAL      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_MAX           (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_PLL_SS_MODFREQ_RESETVAL                      (0x00000000U)

/* PLL0_HSDIV_CLKDIV */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_HSDIV1_MASK                     (0x0000003FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_HSDIV1_SHIFT                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_HSDIV1_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_HSDIV1_MAX                      (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_HSDIV2_MASK                     (0x00003F00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_HSDIV2_SHIFT                    (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_HSDIV2_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_HSDIV2_MAX                      (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_HSDIV3_MASK                     (0x003F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_HSDIV3_SHIFT                    (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_HSDIV3_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_HSDIV3_MAX                      (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_HSDIV4_MASK                     (0x3F000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_HSDIV4_SHIFT                    (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_HSDIV4_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_HSDIV4_MAX                      (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CLKDIV_RESETVAL                        (0x00000000U)

/* PLL0_HSDIV_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_CLKOUT1_EN_MASK                   (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_CLKOUT1_EN_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_CLKOUT1_EN_RESETVAL               (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_CLKOUT1_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_CLKOUT2_EN_MASK                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_CLKOUT2_EN_SHIFT                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_CLKOUT2_EN_RESETVAL               (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_CLKOUT2_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_CLKOUT3_EN_MASK                   (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_CLKOUT3_EN_SHIFT                  (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_CLKOUT3_EN_RESETVAL               (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_CLKOUT3_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_CLKOUT4_EN_MASK                   (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_CLKOUT4_EN_SHIFT                  (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_CLKOUT4_EN_RESETVAL               (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_CLKOUT4_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_SPAREIN_MASK                      (0x001F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_SPAREIN_SHIFT                     (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_SPAREIN_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_SPAREIN_MAX                       (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_TENABLEDIV_MASK                   (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_TENABLEDIV_SHIFT                  (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_TENABLEDIV_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_TENABLEDIV_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_CTRL_RESETVAL                          (0x0000000FU)

/* PLL0_HSDIV_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_CLKOUT1_EN_ACK_MASK               (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_CLKOUT1_EN_ACK_SHIFT              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_CLKOUT1_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_CLKOUT1_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_CLKOUT2_EN_ACK_MASK               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_CLKOUT2_EN_ACK_SHIFT              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_CLKOUT2_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_CLKOUT2_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_CLKOUT3_EN_ACK_MASK               (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_CLKOUT3_EN_ACK_SHIFT              (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_CLKOUT3_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_CLKOUT3_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_CLKOUT4_EN_ACK_MASK               (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_CLKOUT4_EN_ACK_SHIFT              (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_CLKOUT4_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_CLKOUT4_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_DIV1_CHANGE_ACK_MASK              (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_DIV1_CHANGE_ACK_SHIFT             (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_DIV1_CHANGE_ACK_RESETVAL          (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_DIV1_CHANGE_ACK_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_DIV2_CHANGE_ACK_MASK              (0x00000200U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_DIV2_CHANGE_ACK_SHIFT             (0x00000009U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_DIV2_CHANGE_ACK_RESETVAL          (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_DIV2_CHANGE_ACK_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_DIV3_CHANGE_ACK_MASK              (0x00000400U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_DIV3_CHANGE_ACK_SHIFT             (0x0000000AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_DIV3_CHANGE_ACK_RESETVAL          (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_DIV3_CHANGE_ACK_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_DIV4_CHANGE_ACK_MASK              (0x00000800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_DIV4_CHANGE_ACK_SHIFT             (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_DIV4_CHANGE_ACK_RESETVAL          (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_DIV4_CHANGE_ACK_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_LOCK_MASK                         (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_LOCK_SHIFT                        (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_LOCK_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_LOCK_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_SPAREOUT_MASK                     (0x001F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_SPAREOUT_SHIFT                    (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_SPAREOUT_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_SPAREOUT_MAX                      (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_STAT_RESETVAL                          (0x00000000U)

/* PLL0_HSDIV_PWR_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_ISO_PRE_Z_MASK                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_ISO_PRE_Z_SHIFT               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_ISO_PRE_Z_RESETVAL            (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_ISO_PRE_Z_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_AIPOFF_MASK                   (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_AIPOFF_SHIFT                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_AIPOFF_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_AIPOFF_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_ISO_CLR_MASK                  (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_ISO_CLR_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_ISO_CLR_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_ISO_CLR_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_ISO_MASK                      (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_ISO_SHIFT                     (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_ISO_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_ISO_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_RET_MASK                      (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_RET_SHIFT                     (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_RET_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_RET_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_PGOODIN_MASK                  (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_PGOODIN_SHIFT                 (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_PGOODIN_RESETVAL              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_PGOODIN_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_PONIN_MASK                    (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_PONIN_SHIFT                   (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_PONIN_RESETVAL                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_PONIN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_CTRL_RESETVAL                      (0xC0000001U)

/* PLL0_HSDIV_PWR_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_STAT_LDOPWDN_MASK                  (0x00004000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_STAT_LDOPWDN_SHIFT                 (0x0000000EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_STAT_LDOPWDN_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_STAT_LDOPWDN_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_STAT_PGOODOUT_MASK                 (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_STAT_PGOODOUT_SHIFT                (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_STAT_PGOODOUT_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_STAT_PGOODOUT_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_STAT_PONOUT_MASK                   (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_STAT_PONOUT_SHIFT                  (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_STAT_PONOUT_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_STAT_PONOUT_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL0_HSDIV_PWR_STAT_RESETVAL                      (0x00000000U)

/* PLL1_KICK0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_KICK0_KICK0_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_KICK0_KICK0_VAL_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_KICK0_KICK0_VAL_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_KICK0_KICK0_VAL_MAX                          (0xFFFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_KICK0_RESETVAL                               (0x00000000U)

/* PLL1_KICK1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_KICK1_KICK1_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_KICK1_KICK1_VAL_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_KICK1_KICK1_VAL_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_KICK1_KICK1_VAL_MAX                          (0xFFFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_KICK1_RESETVAL                               (0x00000000U)

/* PLL1_PLL_FREQ_CTL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL0_NDIV_MASK                      (0x000000FFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL0_NDIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL0_NDIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL0_NDIV_MAX                       (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL0_M_INT_MULT_MASK                (0x000FFF00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL0_M_INT_MULT_SHIFT               (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL0_M_INT_MULT_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL0_M_INT_MULT_MAX                 (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL0_SD_DIV_MASK                    (0xFF000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL0_SD_DIV_SHIFT                   (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL0_SD_DIV_RESETVAL                (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL0_SD_DIV_MAX                     (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL0_RESETVAL                       (0x02000000U)

/* PLL1_PLL_FREQ_CTL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL1_M_FRAC_MULT_MASK               (0x0003FFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL1_M_FRAC_MULT_SHIFT              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL1_M_FRAC_MULT_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL1_M_FRAC_MULT_MAX                (0x0003FFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL1_SEL_FREQ_DCO_MASK              (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL1_SEL_FREQ_DCO_SHIFT             (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL1_SEL_FREQ_DCO_RESETVAL          (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL1_SEL_FREQ_DCO_MAX               (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_FREQ_CTL1_RESETVAL                       (0x02000000U)

/* PLL1_PLL_CLK_DIV */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CLK_DIV_M1_DIV_MASK                      (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CLK_DIV_M1_DIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CLK_DIV_M1_DIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CLK_DIV_M1_DIV_MAX                       (0x0000000FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CLK_DIV_M2_DIV_MASK                      (0x00007F00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CLK_DIV_M2_DIV_SHIFT                     (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CLK_DIV_M2_DIV_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CLK_DIV_M2_DIV_MAX                       (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CLK_DIV_M3_DIV_MASK                      (0x001F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CLK_DIV_M3_DIV_SHIFT                     (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CLK_DIV_M3_DIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CLK_DIV_M3_DIV_MAX                       (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CLK_DIV_RESETVAL                         (0x00000100U)

/* PLL1_PLL_PROG */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PROG_TINIT_Z_MASK                        (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PROG_TINIT_Z_SHIFT                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PROG_TINIT_Z_RESETVAL                    (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PROG_TINIT_Z_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PROG_TENABLE_MASK                        (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PROG_TENABLE_SHIFT                       (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PROG_TENABLE_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PROG_TENABLE_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PROG_TENABLEDIV_MASK                     (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PROG_TENABLEDIV_SHIFT                    (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PROG_TENABLEDIV_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PROG_TENABLEDIV_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PROG_RESETVAL                            (0x00000001U)

/* PLL1_PLL_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUTBYPASS_EN_MASK                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUTBYPASS_EN_SHIFT               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUTBYPASS_EN_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUTBYPASS_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUTHIF_EN_MASK                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUTHIF_EN_SHIFT                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUTHIF_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUTHIF_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUTX2_EN_MASK                    (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUTX2_EN_SHIFT                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUTX2_EN_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUTX2_EN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUT_EN_MASK                      (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUT_EN_SHIFT                     (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUT_EN_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUT_EN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKDCOLDO_EN_MASK                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKDCOLDO_EN_SHIFT                  (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKDCOLDO_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKDCOLDO_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUTLDO_EN_MASK                   (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUTLDO_EN_SHIFT                  (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUTLDO_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKOUTLDO_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_IDLE_MASK                           (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_IDLE_SHIFT                          (0x00000007U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_IDLE_RESETVAL                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_IDLE_MAX                            (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKRAMP_RATE_MASK                   (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKRAMP_RATE_SHIFT                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKRAMP_RATE_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKRAMP_RATE_MAX                    (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKRAMP_LEVEL_MASK                  (0x00003000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKRAMP_LEVEL_SHIFT                 (0x0000000CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKRAMP_LEVEL_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKRAMP_LEVEL_MAX                   (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_ULOWCLK_EN_MASK                     (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_ULOWCLK_EN_SHIFT                    (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_ULOWCLK_EN_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_ULOWCLK_EN_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_DRIFTGUARD_EN_MASK                  (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_DRIFTGUARD_EN_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_DRIFTGUARD_EN_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_DRIFTGUARD_EN_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_REGM4X_EN_MASK                      (0x00020000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_REGM4X_EN_SHIFT                     (0x00000011U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_REGM4X_EN_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_REGM4X_EN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKINPHIF_SEL_MASK                  (0x00040000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKINPHIF_SEL_SHIFT                 (0x00000012U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKINPHIF_SEL_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CLKINPHIF_SEL_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_RELOCK_RAMP_EN_MASK                 (0x00080000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_RELOCK_RAMP_EN_SHIFT                (0x00000013U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_RELOCK_RAMP_EN_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_RELOCK_RAMP_EN_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CYCLESLIP_EN_MASK                   (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CYCLESLIP_EN_SHIFT                  (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CYCLESLIP_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_CYCLESLIP_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_LOWCURRSTBY_MASK                    (0x00200000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_LOWCURRSTBY_SHIFT                   (0x00000015U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_LOWCURRSTBY_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_LOWCURRSTBY_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_RELAXED_LOCK_MASK                   (0x00400000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_RELAXED_LOCK_SHIFT                  (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_RELAXED_LOCK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_RELAXED_LOCK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_LPMODE_MASK                         (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_LPMODE_SHIFT                        (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_LPMODE_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_LPMODE_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_BW_CONTROL_MASK                     (0x03000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_BW_CONTROL_SHIFT                    (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_BW_CONTROL_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_BW_CONTROL_MAX                      (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_BW_INCR_DECRZ_MASK                  (0x04000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_BW_INCR_DECRZ_SHIFT                 (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_BW_INCR_DECRZ_RESETVAL              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_BW_INCR_DECRZ_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_DC_CORRECTOR_EN_MASK                (0x08000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_DC_CORRECTOR_EN_SHIFT               (0x0000001BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_DC_CORRECTOR_EN_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_DC_CORRECTOR_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_DOWNSPREAD_MASK                     (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_DOWNSPREAD_SHIFT                    (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_DOWNSPREAD_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_DOWNSPREAD_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_SSC_EN_MASK                         (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_SSC_EN_SHIFT                        (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_SSC_EN_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_SSC_EN_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_CTRL_RESETVAL                            (0x04000000U)

/* PLL1_PLL_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUTBYPASS_EN_ACK_MASK            (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUTBYPASS_EN_ACK_SHIFT           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUTBYPASS_EN_ACK_RESETVAL        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUTBYPASS_EN_ACK_MAX             (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUTHIF_EN_ACK_MASK               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUTHIF_EN_ACK_SHIFT              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUTHIF_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUTHIF_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUTX2_EN_ACK_MASK                (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUTX2_EN_ACK_SHIFT               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUTX2_EN_ACK_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUTX2_EN_ACK_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUT_EN_ACK_MASK                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUT_EN_ACK_SHIFT                 (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUT_EN_ACK_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUT_EN_ACK_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKDCOLDO_EN_ACK_MASK               (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKDCOLDO_EN_ACK_SHIFT              (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKDCOLDO_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKDCOLDO_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUTLDO_EN_ACK_MASK               (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUTLDO_EN_ACK_SHIFT              (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUTLDO_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_CLKOUTLDO_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_BYPASS_ACK_MASK                     (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_BYPASS_ACK_SHIFT                    (0x00000007U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_BYPASS_ACK_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_BYPASS_ACK_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_M1CHANGE_ACK_MASK                   (0x00000200U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_M1CHANGE_ACK_SHIFT                  (0x00000009U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_M1CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_M1CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_M2CHANGE_ACK_MASK                   (0x00000400U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_M2CHANGE_ACK_SHIFT                  (0x0000000AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_M2CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_M2CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_M3CHANGE_ACK_MASK                   (0x00000800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_M3CHANGE_ACK_SHIFT                  (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_M3CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_M3CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_BYPASS_CLKTYPE_MASK                 (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_BYPASS_CLKTYPE_SHIFT                (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_BYPASS_CLKTYPE_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_BYPASS_CLKTYPE_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_TVALID_MASK                         (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_TVALID_SHIFT                        (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_TVALID_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_TVALID_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_LOCK2_MASK                          (0x00400000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_LOCK2_SHIFT                         (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_LOCK2_RESETVAL                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_LOCK2_MAX                           (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_HIGHJITTER_MASK                     (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_HIGHJITTER_SHIFT                    (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_HIGHJITTER_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_HIGHJITTER_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_PHASELOCK_MASK                      (0x01000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_PHASELOCK_SHIFT                     (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_PHASELOCK_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_PHASELOCK_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_FREQLOCK_MASK                       (0x02000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_FREQLOCK_SHIFT                      (0x00000019U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_FREQLOCK_RESETVAL                   (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_FREQLOCK_MAX                        (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_LOSSREF_MASK                        (0x04000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_LOSSREF_SHIFT                       (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_LOSSREF_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_LOSSREF_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_RECAL_MASK                          (0x08000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_RECAL_SHIFT                         (0x0000001BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_RECAL_RESETVAL                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_RECAL_MAX                           (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_SSC_ACK_MASK                        (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_SSC_ACK_SHIFT                       (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_SSC_ACK_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_SSC_ACK_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_STAT_RESETVAL                            (0x00000000U)

/* PLL1_PLL_PWR_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISO_PRE_Z_MASK                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISO_PRE_Z_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISO_PRE_Z_RESETVAL              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISO_PRE_Z_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISOSCAN_PRE_Z_MASK              (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISOSCAN_PRE_Z_SHIFT             (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISOSCAN_PRE_Z_RESETVAL          (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISOSCAN_PRE_Z_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_OFFMODE_MASK                    (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_OFFMODE_SHIFT                   (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_OFFMODE_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_OFFMODE_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISO_CLR_MASK                    (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISO_CLR_SHIFT                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISO_CLR_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISO_CLR_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISOSCAN_CLR_MASK                (0x00020000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISOSCAN_CLR_SHIFT               (0x00000011U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISOSCAN_CLR_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISOSCAN_CLR_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISORET_MASK                     (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISORET_SHIFT                    (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISORET_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISORET_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISO_SCAN_MASK                   (0x00200000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISO_SCAN_SHIFT                  (0x00000015U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISO_SCAN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_ISO_SCAN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_RET_MASK                        (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_RET_SHIFT                       (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_RET_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_RET_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_PGOODIN_MASK                    (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_PGOODIN_SHIFT                   (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_PGOODIN_RESETVAL                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_PGOODIN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_PONIN_MASK                      (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_PONIN_SHIFT                     (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_PONIN_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_PONIN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_CTRL_RESETVAL                        (0xC0000003U)

/* PLL1_PLL_PWR_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_LDOPWDN_MASK                    (0x00004000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_LDOPWDN_SHIFT                   (0x0000000EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_LDOPWDN_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_LDOPWDN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_TICOPWDN_MASK                   (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_TICOPWDN_SHIFT                  (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_TICOPWDN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_TICOPWDN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_PGOODOUT_MASK                   (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_PGOODOUT_SHIFT                  (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_PGOODOUT_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_PGOODOUT_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_PONOUT_MASK                     (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_PONOUT_SHIFT                    (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_PONOUT_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_PONOUT_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_PWR_STAT_RESETVAL                        (0x00000000U)

/* PLL1_PLL_SS_SPREAD */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_MASK          (0x0003FFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_SHIFT         (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_RESETVAL      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_MAX           (0x0003FFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_SPREAD_DELTA_MSTEP_INT_MASK           (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_SPREAD_DELTA_MSTEP_INT_SHIFT          (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_SPREAD_DELTA_MSTEP_INT_RESETVAL       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_SPREAD_DELTA_MSTEP_INT_MAX            (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_SPREAD_RESETVAL                       (0x00000000U)

/* PLL1_PLL_SS_MODFREQ */

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_MASK         (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_SHIFT        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_RESETVAL     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_MAX          (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_MASK          (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_SHIFT         (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_RESETVAL      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_MAX           (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL1_PLL_SS_MODFREQ_RESETVAL                      (0x00000000U)

/* PLL2_KICK0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_KICK0_KICK0_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_KICK0_KICK0_VAL_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_KICK0_KICK0_VAL_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_KICK0_KICK0_VAL_MAX                          (0xFFFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_KICK0_RESETVAL                               (0x00000000U)

/* PLL2_KICK1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_KICK1_KICK1_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_KICK1_KICK1_VAL_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_KICK1_KICK1_VAL_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_KICK1_KICK1_VAL_MAX                          (0xFFFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_KICK1_RESETVAL                               (0x00000000U)

/* PLL2_PLL_FREQ_CTL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL0_NDIV_MASK                      (0x000000FFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL0_NDIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL0_NDIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL0_NDIV_MAX                       (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL0_M_INT_MULT_MASK                (0x000FFF00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL0_M_INT_MULT_SHIFT               (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL0_M_INT_MULT_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL0_M_INT_MULT_MAX                 (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL0_SD_DIV_MASK                    (0xFF000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL0_SD_DIV_SHIFT                   (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL0_SD_DIV_RESETVAL                (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL0_SD_DIV_MAX                     (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL0_RESETVAL                       (0x02000000U)

/* PLL2_PLL_FREQ_CTL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL1_M_FRAC_MULT_MASK               (0x0003FFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL1_M_FRAC_MULT_SHIFT              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL1_M_FRAC_MULT_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL1_M_FRAC_MULT_MAX                (0x0003FFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL1_SEL_FREQ_DCO_MASK              (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL1_SEL_FREQ_DCO_SHIFT             (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL1_SEL_FREQ_DCO_RESETVAL          (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL1_SEL_FREQ_DCO_MAX               (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_FREQ_CTL1_RESETVAL                       (0x02000000U)

/* PLL2_PLL_CLK_DIV */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CLK_DIV_M1_DIV_MASK                      (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CLK_DIV_M1_DIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CLK_DIV_M1_DIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CLK_DIV_M1_DIV_MAX                       (0x0000000FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CLK_DIV_M2_DIV_MASK                      (0x00007F00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CLK_DIV_M2_DIV_SHIFT                     (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CLK_DIV_M2_DIV_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CLK_DIV_M2_DIV_MAX                       (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CLK_DIV_M3_DIV_MASK                      (0x001F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CLK_DIV_M3_DIV_SHIFT                     (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CLK_DIV_M3_DIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CLK_DIV_M3_DIV_MAX                       (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CLK_DIV_RESETVAL                         (0x00000100U)

/* PLL2_PLL_PROG */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PROG_TINIT_Z_MASK                        (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PROG_TINIT_Z_SHIFT                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PROG_TINIT_Z_RESETVAL                    (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PROG_TINIT_Z_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PROG_TENABLE_MASK                        (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PROG_TENABLE_SHIFT                       (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PROG_TENABLE_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PROG_TENABLE_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PROG_TENABLEDIV_MASK                     (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PROG_TENABLEDIV_SHIFT                    (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PROG_TENABLEDIV_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PROG_TENABLEDIV_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PROG_RESETVAL                            (0x00000001U)

/* PLL2_PLL_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUTBYPASS_EN_MASK                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUTBYPASS_EN_SHIFT               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUTBYPASS_EN_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUTBYPASS_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUTHIF_EN_MASK                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUTHIF_EN_SHIFT                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUTHIF_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUTHIF_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUTX2_EN_MASK                    (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUTX2_EN_SHIFT                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUTX2_EN_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUTX2_EN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUT_EN_MASK                      (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUT_EN_SHIFT                     (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUT_EN_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUT_EN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKDCOLDO_EN_MASK                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKDCOLDO_EN_SHIFT                  (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKDCOLDO_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKDCOLDO_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUTLDO_EN_MASK                   (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUTLDO_EN_SHIFT                  (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUTLDO_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKOUTLDO_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_IDLE_MASK                           (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_IDLE_SHIFT                          (0x00000007U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_IDLE_RESETVAL                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_IDLE_MAX                            (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKRAMP_RATE_MASK                   (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKRAMP_RATE_SHIFT                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKRAMP_RATE_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKRAMP_RATE_MAX                    (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKRAMP_LEVEL_MASK                  (0x00003000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKRAMP_LEVEL_SHIFT                 (0x0000000CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKRAMP_LEVEL_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKRAMP_LEVEL_MAX                   (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_ULOWCLK_EN_MASK                     (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_ULOWCLK_EN_SHIFT                    (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_ULOWCLK_EN_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_ULOWCLK_EN_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_DRIFTGUARD_EN_MASK                  (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_DRIFTGUARD_EN_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_DRIFTGUARD_EN_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_DRIFTGUARD_EN_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_REGM4X_EN_MASK                      (0x00020000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_REGM4X_EN_SHIFT                     (0x00000011U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_REGM4X_EN_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_REGM4X_EN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKINPHIF_SEL_MASK                  (0x00040000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKINPHIF_SEL_SHIFT                 (0x00000012U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKINPHIF_SEL_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CLKINPHIF_SEL_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_RELOCK_RAMP_EN_MASK                 (0x00080000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_RELOCK_RAMP_EN_SHIFT                (0x00000013U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_RELOCK_RAMP_EN_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_RELOCK_RAMP_EN_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CYCLESLIP_EN_MASK                   (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CYCLESLIP_EN_SHIFT                  (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CYCLESLIP_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_CYCLESLIP_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_LOWCURRSTBY_MASK                    (0x00200000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_LOWCURRSTBY_SHIFT                   (0x00000015U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_LOWCURRSTBY_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_LOWCURRSTBY_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_RELAXED_LOCK_MASK                   (0x00400000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_RELAXED_LOCK_SHIFT                  (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_RELAXED_LOCK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_RELAXED_LOCK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_LPMODE_MASK                         (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_LPMODE_SHIFT                        (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_LPMODE_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_LPMODE_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_BW_CONTROL_MASK                     (0x03000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_BW_CONTROL_SHIFT                    (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_BW_CONTROL_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_BW_CONTROL_MAX                      (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_BW_INCR_DECRZ_MASK                  (0x04000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_BW_INCR_DECRZ_SHIFT                 (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_BW_INCR_DECRZ_RESETVAL              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_BW_INCR_DECRZ_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_DC_CORRECTOR_EN_MASK                (0x08000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_DC_CORRECTOR_EN_SHIFT               (0x0000001BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_DC_CORRECTOR_EN_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_DC_CORRECTOR_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_DOWNSPREAD_MASK                     (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_DOWNSPREAD_SHIFT                    (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_DOWNSPREAD_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_DOWNSPREAD_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_SSC_EN_MASK                         (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_SSC_EN_SHIFT                        (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_SSC_EN_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_SSC_EN_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_CTRL_RESETVAL                            (0x04000000U)

/* PLL2_PLL_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUTBYPASS_EN_ACK_MASK            (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUTBYPASS_EN_ACK_SHIFT           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUTBYPASS_EN_ACK_RESETVAL        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUTBYPASS_EN_ACK_MAX             (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUTHIF_EN_ACK_MASK               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUTHIF_EN_ACK_SHIFT              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUTHIF_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUTHIF_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUTX2_EN_ACK_MASK                (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUTX2_EN_ACK_SHIFT               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUTX2_EN_ACK_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUTX2_EN_ACK_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUT_EN_ACK_MASK                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUT_EN_ACK_SHIFT                 (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUT_EN_ACK_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUT_EN_ACK_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKDCOLDO_EN_ACK_MASK               (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKDCOLDO_EN_ACK_SHIFT              (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKDCOLDO_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKDCOLDO_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUTLDO_EN_ACK_MASK               (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUTLDO_EN_ACK_SHIFT              (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUTLDO_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_CLKOUTLDO_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_BYPASS_ACK_MASK                     (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_BYPASS_ACK_SHIFT                    (0x00000007U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_BYPASS_ACK_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_BYPASS_ACK_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_M1CHANGE_ACK_MASK                   (0x00000200U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_M1CHANGE_ACK_SHIFT                  (0x00000009U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_M1CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_M1CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_M2CHANGE_ACK_MASK                   (0x00000400U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_M2CHANGE_ACK_SHIFT                  (0x0000000AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_M2CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_M2CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_M3CHANGE_ACK_MASK                   (0x00000800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_M3CHANGE_ACK_SHIFT                  (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_M3CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_M3CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_BYPASS_CLKTYPE_MASK                 (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_BYPASS_CLKTYPE_SHIFT                (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_BYPASS_CLKTYPE_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_BYPASS_CLKTYPE_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_TVALID_MASK                         (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_TVALID_SHIFT                        (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_TVALID_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_TVALID_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_LOCK2_MASK                          (0x00400000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_LOCK2_SHIFT                         (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_LOCK2_RESETVAL                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_LOCK2_MAX                           (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_HIGHJITTER_MASK                     (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_HIGHJITTER_SHIFT                    (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_HIGHJITTER_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_HIGHJITTER_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_PHASELOCK_MASK                      (0x01000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_PHASELOCK_SHIFT                     (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_PHASELOCK_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_PHASELOCK_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_FREQLOCK_MASK                       (0x02000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_FREQLOCK_SHIFT                      (0x00000019U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_FREQLOCK_RESETVAL                   (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_FREQLOCK_MAX                        (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_LOSSREF_MASK                        (0x04000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_LOSSREF_SHIFT                       (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_LOSSREF_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_LOSSREF_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_RECAL_MASK                          (0x08000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_RECAL_SHIFT                         (0x0000001BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_RECAL_RESETVAL                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_RECAL_MAX                           (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_SSC_ACK_MASK                        (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_SSC_ACK_SHIFT                       (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_SSC_ACK_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_SSC_ACK_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_STAT_RESETVAL                            (0x00000000U)

/* PLL2_PLL_PWR_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISO_PRE_Z_MASK                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISO_PRE_Z_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISO_PRE_Z_RESETVAL              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISO_PRE_Z_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISOSCAN_PRE_Z_MASK              (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISOSCAN_PRE_Z_SHIFT             (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISOSCAN_PRE_Z_RESETVAL          (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISOSCAN_PRE_Z_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_OFFMODE_MASK                    (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_OFFMODE_SHIFT                   (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_OFFMODE_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_OFFMODE_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISO_CLR_MASK                    (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISO_CLR_SHIFT                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISO_CLR_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISO_CLR_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISOSCAN_CLR_MASK                (0x00020000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISOSCAN_CLR_SHIFT               (0x00000011U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISOSCAN_CLR_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISOSCAN_CLR_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISORET_MASK                     (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISORET_SHIFT                    (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISORET_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISORET_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISO_SCAN_MASK                   (0x00200000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISO_SCAN_SHIFT                  (0x00000015U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISO_SCAN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_ISO_SCAN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_RET_MASK                        (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_RET_SHIFT                       (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_RET_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_RET_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_PGOODIN_MASK                    (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_PGOODIN_SHIFT                   (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_PGOODIN_RESETVAL                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_PGOODIN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_PONIN_MASK                      (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_PONIN_SHIFT                     (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_PONIN_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_PONIN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_CTRL_RESETVAL                        (0xC0000003U)

/* PLL2_PLL_PWR_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_LDOPWDN_MASK                    (0x00004000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_LDOPWDN_SHIFT                   (0x0000000EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_LDOPWDN_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_LDOPWDN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_TICOPWDN_MASK                   (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_TICOPWDN_SHIFT                  (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_TICOPWDN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_TICOPWDN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_PGOODOUT_MASK                   (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_PGOODOUT_SHIFT                  (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_PGOODOUT_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_PGOODOUT_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_PONOUT_MASK                     (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_PONOUT_SHIFT                    (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_PONOUT_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_PONOUT_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_PWR_STAT_RESETVAL                        (0x00000000U)

/* PLL2_PLL_SS_SPREAD */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_MASK          (0x0003FFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_SHIFT         (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_RESETVAL      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_MAX           (0x0003FFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_SPREAD_DELTA_MSTEP_INT_MASK           (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_SPREAD_DELTA_MSTEP_INT_SHIFT          (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_SPREAD_DELTA_MSTEP_INT_RESETVAL       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_SPREAD_DELTA_MSTEP_INT_MAX            (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_SPREAD_RESETVAL                       (0x00000000U)

/* PLL2_PLL_SS_MODFREQ */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_MASK         (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_SHIFT        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_RESETVAL     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_MAX          (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_MASK          (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_SHIFT         (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_RESETVAL      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_MAX           (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_PLL_SS_MODFREQ_RESETVAL                      (0x00000000U)

/* PLL2_HSDIV_CLKDIV */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_HSDIV1_MASK                     (0x0000003FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_HSDIV1_SHIFT                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_HSDIV1_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_HSDIV1_MAX                      (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_HSDIV2_MASK                     (0x00003F00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_HSDIV2_SHIFT                    (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_HSDIV2_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_HSDIV2_MAX                      (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_HSDIV3_MASK                     (0x003F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_HSDIV3_SHIFT                    (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_HSDIV3_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_HSDIV3_MAX                      (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_HSDIV4_MASK                     (0x3F000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_HSDIV4_SHIFT                    (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_HSDIV4_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_HSDIV4_MAX                      (0x0000003FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CLKDIV_RESETVAL                        (0x00000000U)

/* PLL2_HSDIV_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_CLKOUT1_EN_MASK                   (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_CLKOUT1_EN_SHIFT                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_CLKOUT1_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_CLKOUT1_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_CLKOUT2_EN_MASK                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_CLKOUT2_EN_SHIFT                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_CLKOUT2_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_CLKOUT2_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_CLKOUT3_EN_MASK                   (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_CLKOUT3_EN_SHIFT                  (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_CLKOUT3_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_CLKOUT3_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_CLKOUT4_EN_MASK                   (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_CLKOUT4_EN_SHIFT                  (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_CLKOUT4_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_CLKOUT4_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_SPAREIN_MASK                      (0x001F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_SPAREIN_SHIFT                     (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_SPAREIN_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_SPAREIN_MAX                       (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_TENABLEDIV_MASK                   (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_TENABLEDIV_SHIFT                  (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_TENABLEDIV_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_TENABLEDIV_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_CTRL_RESETVAL                          (0x00000000U)

/* PLL2_HSDIV_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_CLKOUT1_EN_ACK_MASK               (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_CLKOUT1_EN_ACK_SHIFT              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_CLKOUT1_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_CLKOUT1_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_CLKOUT2_EN_ACK_MASK               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_CLKOUT2_EN_ACK_SHIFT              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_CLKOUT2_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_CLKOUT2_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_CLKOUT3_EN_ACK_MASK               (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_CLKOUT3_EN_ACK_SHIFT              (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_CLKOUT3_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_CLKOUT3_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_CLKOUT4_EN_ACK_MASK               (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_CLKOUT4_EN_ACK_SHIFT              (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_CLKOUT4_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_CLKOUT4_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_DIV1_CHANGE_ACK_MASK              (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_DIV1_CHANGE_ACK_SHIFT             (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_DIV1_CHANGE_ACK_RESETVAL          (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_DIV1_CHANGE_ACK_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_DIV2_CHANGE_ACK_MASK              (0x00000200U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_DIV2_CHANGE_ACK_SHIFT             (0x00000009U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_DIV2_CHANGE_ACK_RESETVAL          (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_DIV2_CHANGE_ACK_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_DIV3_CHANGE_ACK_MASK              (0x00000400U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_DIV3_CHANGE_ACK_SHIFT             (0x0000000AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_DIV3_CHANGE_ACK_RESETVAL          (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_DIV3_CHANGE_ACK_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_DIV4_CHANGE_ACK_MASK              (0x00000800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_DIV4_CHANGE_ACK_SHIFT             (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_DIV4_CHANGE_ACK_RESETVAL          (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_DIV4_CHANGE_ACK_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_LOCK_MASK                         (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_LOCK_SHIFT                        (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_LOCK_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_LOCK_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_SPAREOUT_MASK                     (0x001F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_SPAREOUT_SHIFT                    (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_SPAREOUT_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_SPAREOUT_MAX                      (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_STAT_RESETVAL                          (0x00000000U)

/* PLL2_HSDIV_PWR_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_ISO_PRE_Z_MASK                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_ISO_PRE_Z_SHIFT               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_ISO_PRE_Z_RESETVAL            (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_ISO_PRE_Z_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_AIPOFF_MASK                   (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_AIPOFF_SHIFT                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_AIPOFF_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_AIPOFF_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_ISO_CLR_MASK                  (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_ISO_CLR_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_ISO_CLR_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_ISO_CLR_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_ISO_MASK                      (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_ISO_SHIFT                     (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_ISO_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_ISO_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_RET_MASK                      (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_RET_SHIFT                     (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_RET_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_RET_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_PGOODIN_MASK                  (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_PGOODIN_SHIFT                 (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_PGOODIN_RESETVAL              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_PGOODIN_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_PONIN_MASK                    (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_PONIN_SHIFT                   (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_PONIN_RESETVAL                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_PONIN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_CTRL_RESETVAL                      (0xC0000001U)

/* PLL2_HSDIV_PWR_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_STAT_LDOPWDN_MASK                  (0x00004000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_STAT_LDOPWDN_SHIFT                 (0x0000000EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_STAT_LDOPWDN_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_STAT_LDOPWDN_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_STAT_PGOODOUT_MASK                 (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_STAT_PGOODOUT_SHIFT                (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_STAT_PGOODOUT_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_STAT_PGOODOUT_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_STAT_PONOUT_MASK                   (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_STAT_PONOUT_SHIFT                  (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_STAT_PONOUT_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_STAT_PONOUT_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL2_HSDIV_PWR_STAT_RESETVAL                      (0x00000000U)

/* PLL3_KICK0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL3_KICK0_KICK0_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_KICK0_KICK0_VAL_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_KICK0_KICK0_VAL_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_KICK0_KICK0_VAL_MAX                          (0xFFFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_KICK0_RESETVAL                               (0x00000000U)

/* PLL3_KICK1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL3_KICK1_KICK1_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_KICK1_KICK1_VAL_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_KICK1_KICK1_VAL_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_KICK1_KICK1_VAL_MAX                          (0xFFFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_KICK1_RESETVAL                               (0x00000000U)

/* PLL3_PLL_FREQ_CTL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL0_NDIV_MASK                      (0x000000FFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL0_NDIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL0_NDIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL0_NDIV_MAX                       (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL0_M_INT_MULT_MASK                (0x000FFF00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL0_M_INT_MULT_SHIFT               (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL0_M_INT_MULT_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL0_M_INT_MULT_MAX                 (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL0_SD_DIV_MASK                    (0xFF000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL0_SD_DIV_SHIFT                   (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL0_SD_DIV_RESETVAL                (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL0_SD_DIV_MAX                     (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL0_RESETVAL                       (0x02000000U)

/* PLL3_PLL_FREQ_CTL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL1_M_FRAC_MULT_MASK               (0x0003FFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL1_M_FRAC_MULT_SHIFT              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL1_M_FRAC_MULT_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL1_M_FRAC_MULT_MAX                (0x0003FFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL1_SEL_FREQ_DCO_MASK              (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL1_SEL_FREQ_DCO_SHIFT             (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL1_SEL_FREQ_DCO_RESETVAL          (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL1_SEL_FREQ_DCO_MAX               (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_FREQ_CTL1_RESETVAL                       (0x02000000U)

/* PLL3_PLL_CLK_DIV */

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CLK_DIV_M1_DIV_MASK                      (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CLK_DIV_M1_DIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CLK_DIV_M1_DIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CLK_DIV_M1_DIV_MAX                       (0x0000000FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CLK_DIV_M2_DIV_MASK                      (0x00007F00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CLK_DIV_M2_DIV_SHIFT                     (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CLK_DIV_M2_DIV_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CLK_DIV_M2_DIV_MAX                       (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CLK_DIV_M3_DIV_MASK                      (0x001F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CLK_DIV_M3_DIV_SHIFT                     (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CLK_DIV_M3_DIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CLK_DIV_M3_DIV_MAX                       (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CLK_DIV_RESETVAL                         (0x00000100U)

/* PLL3_PLL_PROG */

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PROG_TINIT_Z_MASK                        (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PROG_TINIT_Z_SHIFT                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PROG_TINIT_Z_RESETVAL                    (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PROG_TINIT_Z_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PROG_TENABLE_MASK                        (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PROG_TENABLE_SHIFT                       (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PROG_TENABLE_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PROG_TENABLE_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PROG_TENABLEDIV_MASK                     (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PROG_TENABLEDIV_SHIFT                    (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PROG_TENABLEDIV_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PROG_TENABLEDIV_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PROG_RESETVAL                            (0x00000001U)

/* PLL3_PLL_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUTBYPASS_EN_MASK                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUTBYPASS_EN_SHIFT               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUTBYPASS_EN_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUTBYPASS_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUTHIF_EN_MASK                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUTHIF_EN_SHIFT                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUTHIF_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUTHIF_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUTX2_EN_MASK                    (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUTX2_EN_SHIFT                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUTX2_EN_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUTX2_EN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUT_EN_MASK                      (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUT_EN_SHIFT                     (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUT_EN_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUT_EN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKDCOLDO_EN_MASK                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKDCOLDO_EN_SHIFT                  (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKDCOLDO_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKDCOLDO_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUTLDO_EN_MASK                   (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUTLDO_EN_SHIFT                  (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUTLDO_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKOUTLDO_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_IDLE_MASK                           (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_IDLE_SHIFT                          (0x00000007U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_IDLE_RESETVAL                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_IDLE_MAX                            (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKRAMP_RATE_MASK                   (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKRAMP_RATE_SHIFT                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKRAMP_RATE_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKRAMP_RATE_MAX                    (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKRAMP_LEVEL_MASK                  (0x00003000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKRAMP_LEVEL_SHIFT                 (0x0000000CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKRAMP_LEVEL_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKRAMP_LEVEL_MAX                   (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_ULOWCLK_EN_MASK                     (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_ULOWCLK_EN_SHIFT                    (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_ULOWCLK_EN_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_ULOWCLK_EN_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_DRIFTGUARD_EN_MASK                  (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_DRIFTGUARD_EN_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_DRIFTGUARD_EN_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_DRIFTGUARD_EN_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_REGM4X_EN_MASK                      (0x00020000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_REGM4X_EN_SHIFT                     (0x00000011U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_REGM4X_EN_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_REGM4X_EN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKINPHIF_SEL_MASK                  (0x00040000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKINPHIF_SEL_SHIFT                 (0x00000012U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKINPHIF_SEL_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CLKINPHIF_SEL_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_RELOCK_RAMP_EN_MASK                 (0x00080000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_RELOCK_RAMP_EN_SHIFT                (0x00000013U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_RELOCK_RAMP_EN_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_RELOCK_RAMP_EN_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CYCLESLIP_EN_MASK                   (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CYCLESLIP_EN_SHIFT                  (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CYCLESLIP_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_CYCLESLIP_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_LOWCURRSTBY_MASK                    (0x00200000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_LOWCURRSTBY_SHIFT                   (0x00000015U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_LOWCURRSTBY_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_LOWCURRSTBY_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_RELAXED_LOCK_MASK                   (0x00400000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_RELAXED_LOCK_SHIFT                  (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_RELAXED_LOCK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_RELAXED_LOCK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_LPMODE_MASK                         (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_LPMODE_SHIFT                        (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_LPMODE_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_LPMODE_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_BW_CONTROL_MASK                     (0x03000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_BW_CONTROL_SHIFT                    (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_BW_CONTROL_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_BW_CONTROL_MAX                      (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_BW_INCR_DECRZ_MASK                  (0x04000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_BW_INCR_DECRZ_SHIFT                 (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_BW_INCR_DECRZ_RESETVAL              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_BW_INCR_DECRZ_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_DC_CORRECTOR_EN_MASK                (0x08000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_DC_CORRECTOR_EN_SHIFT               (0x0000001BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_DC_CORRECTOR_EN_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_DC_CORRECTOR_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_DOWNSPREAD_MASK                     (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_DOWNSPREAD_SHIFT                    (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_DOWNSPREAD_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_DOWNSPREAD_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_SSC_EN_MASK                         (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_SSC_EN_SHIFT                        (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_SSC_EN_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_SSC_EN_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_CTRL_RESETVAL                            (0x04000000U)

/* PLL3_PLL_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUTBYPASS_EN_ACK_MASK            (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUTBYPASS_EN_ACK_SHIFT           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUTBYPASS_EN_ACK_RESETVAL        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUTBYPASS_EN_ACK_MAX             (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUTHIF_EN_ACK_MASK               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUTHIF_EN_ACK_SHIFT              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUTHIF_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUTHIF_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUTX2_EN_ACK_MASK                (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUTX2_EN_ACK_SHIFT               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUTX2_EN_ACK_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUTX2_EN_ACK_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUT_EN_ACK_MASK                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUT_EN_ACK_SHIFT                 (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUT_EN_ACK_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUT_EN_ACK_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKDCOLDO_EN_ACK_MASK               (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKDCOLDO_EN_ACK_SHIFT              (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKDCOLDO_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKDCOLDO_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUTLDO_EN_ACK_MASK               (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUTLDO_EN_ACK_SHIFT              (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUTLDO_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_CLKOUTLDO_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_BYPASS_ACK_MASK                     (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_BYPASS_ACK_SHIFT                    (0x00000007U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_BYPASS_ACK_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_BYPASS_ACK_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_M1CHANGE_ACK_MASK                   (0x00000200U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_M1CHANGE_ACK_SHIFT                  (0x00000009U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_M1CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_M1CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_M2CHANGE_ACK_MASK                   (0x00000400U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_M2CHANGE_ACK_SHIFT                  (0x0000000AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_M2CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_M2CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_M3CHANGE_ACK_MASK                   (0x00000800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_M3CHANGE_ACK_SHIFT                  (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_M3CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_M3CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_BYPASS_CLKTYPE_MASK                 (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_BYPASS_CLKTYPE_SHIFT                (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_BYPASS_CLKTYPE_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_BYPASS_CLKTYPE_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_TVALID_MASK                         (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_TVALID_SHIFT                        (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_TVALID_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_TVALID_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_LOCK2_MASK                          (0x00400000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_LOCK2_SHIFT                         (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_LOCK2_RESETVAL                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_LOCK2_MAX                           (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_HIGHJITTER_MASK                     (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_HIGHJITTER_SHIFT                    (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_HIGHJITTER_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_HIGHJITTER_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_PHASELOCK_MASK                      (0x01000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_PHASELOCK_SHIFT                     (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_PHASELOCK_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_PHASELOCK_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_FREQLOCK_MASK                       (0x02000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_FREQLOCK_SHIFT                      (0x00000019U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_FREQLOCK_RESETVAL                   (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_FREQLOCK_MAX                        (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_LOSSREF_MASK                        (0x04000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_LOSSREF_SHIFT                       (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_LOSSREF_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_LOSSREF_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_RECAL_MASK                          (0x08000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_RECAL_SHIFT                         (0x0000001BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_RECAL_RESETVAL                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_RECAL_MAX                           (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_SSC_ACK_MASK                        (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_SSC_ACK_SHIFT                       (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_SSC_ACK_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_SSC_ACK_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_STAT_RESETVAL                            (0x00000000U)

/* PLL3_PLL_PWR_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISO_PRE_Z_MASK                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISO_PRE_Z_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISO_PRE_Z_RESETVAL              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISO_PRE_Z_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISOSCAN_PRE_Z_MASK              (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISOSCAN_PRE_Z_SHIFT             (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISOSCAN_PRE_Z_RESETVAL          (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISOSCAN_PRE_Z_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_OFFMODE_MASK                    (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_OFFMODE_SHIFT                   (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_OFFMODE_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_OFFMODE_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISO_CLR_MASK                    (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISO_CLR_SHIFT                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISO_CLR_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISO_CLR_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISOSCAN_CLR_MASK                (0x00020000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISOSCAN_CLR_SHIFT               (0x00000011U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISOSCAN_CLR_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISOSCAN_CLR_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISORET_MASK                     (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISORET_SHIFT                    (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISORET_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISORET_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISO_SCAN_MASK                   (0x00200000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISO_SCAN_SHIFT                  (0x00000015U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISO_SCAN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_ISO_SCAN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_RET_MASK                        (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_RET_SHIFT                       (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_RET_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_RET_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_PGOODIN_MASK                    (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_PGOODIN_SHIFT                   (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_PGOODIN_RESETVAL                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_PGOODIN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_PONIN_MASK                      (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_PONIN_SHIFT                     (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_PONIN_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_PONIN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_CTRL_RESETVAL                        (0xC0000003U)

/* PLL3_PLL_PWR_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_LDOPWDN_MASK                    (0x00004000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_LDOPWDN_SHIFT                   (0x0000000EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_LDOPWDN_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_LDOPWDN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_TICOPWDN_MASK                   (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_TICOPWDN_SHIFT                  (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_TICOPWDN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_TICOPWDN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_PGOODOUT_MASK                   (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_PGOODOUT_SHIFT                  (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_PGOODOUT_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_PGOODOUT_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_PONOUT_MASK                     (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_PONOUT_SHIFT                    (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_PONOUT_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_PONOUT_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_PWR_STAT_RESETVAL                        (0x00000000U)

/* PLL3_PLL_SS_SPREAD */

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_MASK          (0x0003FFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_SHIFT         (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_RESETVAL      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_MAX           (0x0003FFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_SPREAD_DELTA_MSTEP_INT_MASK           (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_SPREAD_DELTA_MSTEP_INT_SHIFT          (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_SPREAD_DELTA_MSTEP_INT_RESETVAL       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_SPREAD_DELTA_MSTEP_INT_MAX            (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_SPREAD_RESETVAL                       (0x00000000U)

/* PLL3_PLL_SS_MODFREQ */

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_MASK         (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_SHIFT        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_RESETVAL     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_MAX          (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_MASK          (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_SHIFT         (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_RESETVAL      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_MAX           (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL3_PLL_SS_MODFREQ_RESETVAL                      (0x00000000U)

/* PLL4_KICK0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL4_KICK0_KICK0_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_KICK0_KICK0_VAL_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_KICK0_KICK0_VAL_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_KICK0_KICK0_VAL_MAX                          (0xFFFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_KICK0_RESETVAL                               (0x00000000U)

/* PLL4_KICK1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL4_KICK1_KICK1_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_KICK1_KICK1_VAL_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_KICK1_KICK1_VAL_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_KICK1_KICK1_VAL_MAX                          (0xFFFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_KICK1_RESETVAL                               (0x00000000U)

/* PLL4_PLL_FREQ_CTL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL0_NDIV_MASK                      (0x000000FFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL0_NDIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL0_NDIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL0_NDIV_MAX                       (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL0_M_INT_MULT_MASK                (0x000FFF00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL0_M_INT_MULT_SHIFT               (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL0_M_INT_MULT_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL0_M_INT_MULT_MAX                 (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL0_SD_DIV_MASK                    (0xFF000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL0_SD_DIV_SHIFT                   (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL0_SD_DIV_RESETVAL                (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL0_SD_DIV_MAX                     (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL0_RESETVAL                       (0x02000000U)

/* PLL4_PLL_FREQ_CTL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL1_M_FRAC_MULT_MASK               (0x0003FFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL1_M_FRAC_MULT_SHIFT              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL1_M_FRAC_MULT_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL1_M_FRAC_MULT_MAX                (0x0003FFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL1_SEL_FREQ_DCO_MASK              (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL1_SEL_FREQ_DCO_SHIFT             (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL1_SEL_FREQ_DCO_RESETVAL          (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL1_SEL_FREQ_DCO_MAX               (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_FREQ_CTL1_RESETVAL                       (0x02000000U)

/* PLL4_PLL_CLK_DIV */

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CLK_DIV_M1_DIV_MASK                      (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CLK_DIV_M1_DIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CLK_DIV_M1_DIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CLK_DIV_M1_DIV_MAX                       (0x0000000FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CLK_DIV_M2_DIV_MASK                      (0x00007F00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CLK_DIV_M2_DIV_SHIFT                     (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CLK_DIV_M2_DIV_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CLK_DIV_M2_DIV_MAX                       (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CLK_DIV_M3_DIV_MASK                      (0x001F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CLK_DIV_M3_DIV_SHIFT                     (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CLK_DIV_M3_DIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CLK_DIV_M3_DIV_MAX                       (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CLK_DIV_RESETVAL                         (0x00000100U)

/* PLL4_PLL_PROG */

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PROG_TINIT_Z_MASK                        (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PROG_TINIT_Z_SHIFT                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PROG_TINIT_Z_RESETVAL                    (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PROG_TINIT_Z_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PROG_TENABLE_MASK                        (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PROG_TENABLE_SHIFT                       (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PROG_TENABLE_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PROG_TENABLE_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PROG_TENABLEDIV_MASK                     (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PROG_TENABLEDIV_SHIFT                    (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PROG_TENABLEDIV_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PROG_TENABLEDIV_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PROG_RESETVAL                            (0x00000001U)

/* PLL4_PLL_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUTBYPASS_EN_MASK                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUTBYPASS_EN_SHIFT               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUTBYPASS_EN_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUTBYPASS_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUTHIF_EN_MASK                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUTHIF_EN_SHIFT                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUTHIF_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUTHIF_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUTX2_EN_MASK                    (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUTX2_EN_SHIFT                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUTX2_EN_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUTX2_EN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUT_EN_MASK                      (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUT_EN_SHIFT                     (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUT_EN_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUT_EN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKDCOLDO_EN_MASK                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKDCOLDO_EN_SHIFT                  (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKDCOLDO_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKDCOLDO_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUTLDO_EN_MASK                   (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUTLDO_EN_SHIFT                  (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUTLDO_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKOUTLDO_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_IDLE_MASK                           (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_IDLE_SHIFT                          (0x00000007U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_IDLE_RESETVAL                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_IDLE_MAX                            (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKRAMP_RATE_MASK                   (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKRAMP_RATE_SHIFT                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKRAMP_RATE_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKRAMP_RATE_MAX                    (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKRAMP_LEVEL_MASK                  (0x00003000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKRAMP_LEVEL_SHIFT                 (0x0000000CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKRAMP_LEVEL_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKRAMP_LEVEL_MAX                   (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_ULOWCLK_EN_MASK                     (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_ULOWCLK_EN_SHIFT                    (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_ULOWCLK_EN_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_ULOWCLK_EN_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_DRIFTGUARD_EN_MASK                  (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_DRIFTGUARD_EN_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_DRIFTGUARD_EN_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_DRIFTGUARD_EN_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_REGM4X_EN_MASK                      (0x00020000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_REGM4X_EN_SHIFT                     (0x00000011U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_REGM4X_EN_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_REGM4X_EN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKINPHIF_SEL_MASK                  (0x00040000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKINPHIF_SEL_SHIFT                 (0x00000012U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKINPHIF_SEL_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CLKINPHIF_SEL_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_RELOCK_RAMP_EN_MASK                 (0x00080000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_RELOCK_RAMP_EN_SHIFT                (0x00000013U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_RELOCK_RAMP_EN_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_RELOCK_RAMP_EN_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CYCLESLIP_EN_MASK                   (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CYCLESLIP_EN_SHIFT                  (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CYCLESLIP_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_CYCLESLIP_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_LOWCURRSTBY_MASK                    (0x00200000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_LOWCURRSTBY_SHIFT                   (0x00000015U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_LOWCURRSTBY_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_LOWCURRSTBY_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_RELAXED_LOCK_MASK                   (0x00400000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_RELAXED_LOCK_SHIFT                  (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_RELAXED_LOCK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_RELAXED_LOCK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_LPMODE_MASK                         (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_LPMODE_SHIFT                        (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_LPMODE_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_LPMODE_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_BW_CONTROL_MASK                     (0x03000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_BW_CONTROL_SHIFT                    (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_BW_CONTROL_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_BW_CONTROL_MAX                      (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_BW_INCR_DECRZ_MASK                  (0x04000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_BW_INCR_DECRZ_SHIFT                 (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_BW_INCR_DECRZ_RESETVAL              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_BW_INCR_DECRZ_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_DC_CORRECTOR_EN_MASK                (0x08000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_DC_CORRECTOR_EN_SHIFT               (0x0000001BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_DC_CORRECTOR_EN_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_DC_CORRECTOR_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_DOWNSPREAD_MASK                     (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_DOWNSPREAD_SHIFT                    (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_DOWNSPREAD_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_DOWNSPREAD_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_SSC_EN_MASK                         (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_SSC_EN_SHIFT                        (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_SSC_EN_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_SSC_EN_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_CTRL_RESETVAL                            (0x04000000U)

/* PLL4_PLL_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUTBYPASS_EN_ACK_MASK            (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUTBYPASS_EN_ACK_SHIFT           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUTBYPASS_EN_ACK_RESETVAL        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUTBYPASS_EN_ACK_MAX             (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUTHIF_EN_ACK_MASK               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUTHIF_EN_ACK_SHIFT              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUTHIF_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUTHIF_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUTX2_EN_ACK_MASK                (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUTX2_EN_ACK_SHIFT               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUTX2_EN_ACK_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUTX2_EN_ACK_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUT_EN_ACK_MASK                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUT_EN_ACK_SHIFT                 (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUT_EN_ACK_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUT_EN_ACK_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKDCOLDO_EN_ACK_MASK               (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKDCOLDO_EN_ACK_SHIFT              (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKDCOLDO_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKDCOLDO_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUTLDO_EN_ACK_MASK               (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUTLDO_EN_ACK_SHIFT              (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUTLDO_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_CLKOUTLDO_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_BYPASS_ACK_MASK                     (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_BYPASS_ACK_SHIFT                    (0x00000007U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_BYPASS_ACK_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_BYPASS_ACK_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_M1CHANGE_ACK_MASK                   (0x00000200U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_M1CHANGE_ACK_SHIFT                  (0x00000009U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_M1CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_M1CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_M2CHANGE_ACK_MASK                   (0x00000400U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_M2CHANGE_ACK_SHIFT                  (0x0000000AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_M2CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_M2CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_M3CHANGE_ACK_MASK                   (0x00000800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_M3CHANGE_ACK_SHIFT                  (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_M3CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_M3CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_BYPASS_CLKTYPE_MASK                 (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_BYPASS_CLKTYPE_SHIFT                (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_BYPASS_CLKTYPE_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_BYPASS_CLKTYPE_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_TVALID_MASK                         (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_TVALID_SHIFT                        (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_TVALID_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_TVALID_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_LOCK2_MASK                          (0x00400000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_LOCK2_SHIFT                         (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_LOCK2_RESETVAL                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_LOCK2_MAX                           (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_HIGHJITTER_MASK                     (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_HIGHJITTER_SHIFT                    (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_HIGHJITTER_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_HIGHJITTER_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_PHASELOCK_MASK                      (0x01000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_PHASELOCK_SHIFT                     (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_PHASELOCK_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_PHASELOCK_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_FREQLOCK_MASK                       (0x02000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_FREQLOCK_SHIFT                      (0x00000019U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_FREQLOCK_RESETVAL                   (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_FREQLOCK_MAX                        (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_LOSSREF_MASK                        (0x04000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_LOSSREF_SHIFT                       (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_LOSSREF_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_LOSSREF_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_RECAL_MASK                          (0x08000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_RECAL_SHIFT                         (0x0000001BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_RECAL_RESETVAL                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_RECAL_MAX                           (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_SSC_ACK_MASK                        (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_SSC_ACK_SHIFT                       (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_SSC_ACK_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_SSC_ACK_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_STAT_RESETVAL                            (0x00000000U)

/* PLL4_PLL_PWR_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISO_PRE_Z_MASK                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISO_PRE_Z_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISO_PRE_Z_RESETVAL              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISO_PRE_Z_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISOSCAN_PRE_Z_MASK              (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISOSCAN_PRE_Z_SHIFT             (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISOSCAN_PRE_Z_RESETVAL          (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISOSCAN_PRE_Z_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_OFFMODE_MASK                    (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_OFFMODE_SHIFT                   (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_OFFMODE_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_OFFMODE_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISO_CLR_MASK                    (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISO_CLR_SHIFT                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISO_CLR_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISO_CLR_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISOSCAN_CLR_MASK                (0x00020000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISOSCAN_CLR_SHIFT               (0x00000011U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISOSCAN_CLR_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISOSCAN_CLR_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISORET_MASK                     (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISORET_SHIFT                    (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISORET_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISORET_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISO_SCAN_MASK                   (0x00200000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISO_SCAN_SHIFT                  (0x00000015U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISO_SCAN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_ISO_SCAN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_RET_MASK                        (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_RET_SHIFT                       (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_RET_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_RET_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_PGOODIN_MASK                    (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_PGOODIN_SHIFT                   (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_PGOODIN_RESETVAL                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_PGOODIN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_PONIN_MASK                      (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_PONIN_SHIFT                     (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_PONIN_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_PONIN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_CTRL_RESETVAL                        (0xC0000003U)

/* PLL4_PLL_PWR_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_LDOPWDN_MASK                    (0x00004000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_LDOPWDN_SHIFT                   (0x0000000EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_LDOPWDN_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_LDOPWDN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_TICOPWDN_MASK                   (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_TICOPWDN_SHIFT                  (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_TICOPWDN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_TICOPWDN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_PGOODOUT_MASK                   (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_PGOODOUT_SHIFT                  (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_PGOODOUT_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_PGOODOUT_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_PONOUT_MASK                     (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_PONOUT_SHIFT                    (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_PONOUT_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_PONOUT_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_PWR_STAT_RESETVAL                        (0x00000000U)

/* PLL4_PLL_SS_SPREAD */

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_MASK          (0x0003FFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_SHIFT         (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_RESETVAL      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_MAX           (0x0003FFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_SPREAD_DELTA_MSTEP_INT_MASK           (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_SPREAD_DELTA_MSTEP_INT_SHIFT          (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_SPREAD_DELTA_MSTEP_INT_RESETVAL       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_SPREAD_DELTA_MSTEP_INT_MAX            (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_SPREAD_RESETVAL                       (0x00000000U)

/* PLL4_PLL_SS_MODFREQ */

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_MASK         (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_SHIFT        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_RESETVAL     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_MAX          (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_MASK          (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_SHIFT         (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_RESETVAL      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_MAX           (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL4_PLL_SS_MODFREQ_RESETVAL                      (0x00000000U)

/* PLL5_KICK0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL5_KICK0_KICK0_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_KICK0_KICK0_VAL_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_KICK0_KICK0_VAL_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_KICK0_KICK0_VAL_MAX                          (0xFFFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_KICK0_RESETVAL                               (0x00000000U)

/* PLL5_KICK1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL5_KICK1_KICK1_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_KICK1_KICK1_VAL_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_KICK1_KICK1_VAL_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_KICK1_KICK1_VAL_MAX                          (0xFFFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_KICK1_RESETVAL                               (0x00000000U)

/* PLL5_PLL_FREQ_CTL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL0_NDIV_MASK                      (0x000000FFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL0_NDIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL0_NDIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL0_NDIV_MAX                       (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL0_M_INT_MULT_MASK                (0x000FFF00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL0_M_INT_MULT_SHIFT               (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL0_M_INT_MULT_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL0_M_INT_MULT_MAX                 (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL0_SD_DIV_MASK                    (0xFF000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL0_SD_DIV_SHIFT                   (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL0_SD_DIV_RESETVAL                (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL0_SD_DIV_MAX                     (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL0_RESETVAL                       (0x02000000U)

/* PLL5_PLL_FREQ_CTL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL1_M_FRAC_MULT_MASK               (0x0003FFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL1_M_FRAC_MULT_SHIFT              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL1_M_FRAC_MULT_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL1_M_FRAC_MULT_MAX                (0x0003FFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL1_SEL_FREQ_DCO_MASK              (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL1_SEL_FREQ_DCO_SHIFT             (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL1_SEL_FREQ_DCO_RESETVAL          (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL1_SEL_FREQ_DCO_MAX               (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_FREQ_CTL1_RESETVAL                       (0x00000000U)

/* PLL5_PLL_CLK_DIV */

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CLK_DIV_M1_DIV_MASK                      (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CLK_DIV_M1_DIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CLK_DIV_M1_DIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CLK_DIV_M1_DIV_MAX                       (0x0000000FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CLK_DIV_M2_DIV_MASK                      (0x00007F00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CLK_DIV_M2_DIV_SHIFT                     (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CLK_DIV_M2_DIV_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CLK_DIV_M2_DIV_MAX                       (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CLK_DIV_M3_DIV_MASK                      (0x001F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CLK_DIV_M3_DIV_SHIFT                     (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CLK_DIV_M3_DIV_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CLK_DIV_M3_DIV_MAX                       (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CLK_DIV_RESETVAL                         (0x00010100U)

/* PLL5_PLL_PROG */

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PROG_TINIT_Z_MASK                        (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PROG_TINIT_Z_SHIFT                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PROG_TINIT_Z_RESETVAL                    (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PROG_TINIT_Z_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PROG_TENABLE_MASK                        (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PROG_TENABLE_SHIFT                       (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PROG_TENABLE_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PROG_TENABLE_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PROG_TENABLEDIV_MASK                     (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PROG_TENABLEDIV_SHIFT                    (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PROG_TENABLEDIV_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PROG_TENABLEDIV_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PROG_RESETVAL                            (0x00000001U)

/* PLL5_PLL_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUTBYPASS_EN_MASK                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUTBYPASS_EN_SHIFT               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUTBYPASS_EN_RESETVAL            (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUTBYPASS_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUTHIF_EN_MASK                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUTHIF_EN_SHIFT                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUTHIF_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUTHIF_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUTX2_EN_MASK                    (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUTX2_EN_SHIFT                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUTX2_EN_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUTX2_EN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUT_EN_MASK                      (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUT_EN_SHIFT                     (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUT_EN_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUT_EN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKDCOLDO_EN_MASK                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKDCOLDO_EN_SHIFT                  (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKDCOLDO_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKDCOLDO_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUTLDO_EN_MASK                   (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUTLDO_EN_SHIFT                  (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUTLDO_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKOUTLDO_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_IDLE_MASK                           (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_IDLE_SHIFT                          (0x00000007U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_IDLE_RESETVAL                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_IDLE_MAX                            (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKRAMP_RATE_MASK                   (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKRAMP_RATE_SHIFT                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKRAMP_RATE_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKRAMP_RATE_MAX                    (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKRAMP_LEVEL_MASK                  (0x00003000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKRAMP_LEVEL_SHIFT                 (0x0000000CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKRAMP_LEVEL_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKRAMP_LEVEL_MAX                   (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_ULOWCLK_EN_MASK                     (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_ULOWCLK_EN_SHIFT                    (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_ULOWCLK_EN_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_ULOWCLK_EN_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_DRIFTGUARD_EN_MASK                  (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_DRIFTGUARD_EN_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_DRIFTGUARD_EN_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_DRIFTGUARD_EN_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_REGM4X_EN_MASK                      (0x00020000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_REGM4X_EN_SHIFT                     (0x00000011U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_REGM4X_EN_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_REGM4X_EN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKINPHIF_SEL_MASK                  (0x00040000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKINPHIF_SEL_SHIFT                 (0x00000012U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKINPHIF_SEL_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CLKINPHIF_SEL_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_RELOCK_RAMP_EN_MASK                 (0x00080000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_RELOCK_RAMP_EN_SHIFT                (0x00000013U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_RELOCK_RAMP_EN_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_RELOCK_RAMP_EN_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CYCLESLIP_EN_MASK                   (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CYCLESLIP_EN_SHIFT                  (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CYCLESLIP_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_CYCLESLIP_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_LOWCURRSTBY_MASK                    (0x00200000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_LOWCURRSTBY_SHIFT                   (0x00000015U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_LOWCURRSTBY_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_LOWCURRSTBY_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_RELAXED_LOCK_MASK                   (0x00400000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_RELAXED_LOCK_SHIFT                  (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_RELAXED_LOCK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_RELAXED_LOCK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_LPMODE_MASK                         (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_LPMODE_SHIFT                        (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_LPMODE_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_LPMODE_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_BW_CONTROL_MASK                     (0x03000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_BW_CONTROL_SHIFT                    (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_BW_CONTROL_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_BW_CONTROL_MAX                      (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_BW_INCR_DECRZ_MASK                  (0x04000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_BW_INCR_DECRZ_SHIFT                 (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_BW_INCR_DECRZ_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_BW_INCR_DECRZ_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_DC_CORRECTOR_EN_MASK                (0x08000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_DC_CORRECTOR_EN_SHIFT               (0x0000001BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_DC_CORRECTOR_EN_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_DC_CORRECTOR_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_DOWNSPREAD_MASK                     (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_DOWNSPREAD_SHIFT                    (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_DOWNSPREAD_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_DOWNSPREAD_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_SSC_EN_MASK                         (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_SSC_EN_SHIFT                        (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_SSC_EN_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_SSC_EN_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_CTRL_RESETVAL                            (0x00000009U)

/* PLL5_PLL_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUTBYPASS_EN_ACK_MASK            (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUTBYPASS_EN_ACK_SHIFT           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUTBYPASS_EN_ACK_RESETVAL        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUTBYPASS_EN_ACK_MAX             (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUTHIF_EN_ACK_MASK               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUTHIF_EN_ACK_SHIFT              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUTHIF_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUTHIF_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUTX2_EN_ACK_MASK                (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUTX2_EN_ACK_SHIFT               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUTX2_EN_ACK_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUTX2_EN_ACK_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUT_EN_ACK_MASK                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUT_EN_ACK_SHIFT                 (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUT_EN_ACK_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUT_EN_ACK_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKDCOLDO_EN_ACK_MASK               (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKDCOLDO_EN_ACK_SHIFT              (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKDCOLDO_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKDCOLDO_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUTLDO_EN_ACK_MASK               (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUTLDO_EN_ACK_SHIFT              (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUTLDO_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_CLKOUTLDO_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_BYPASS_ACK_MASK                     (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_BYPASS_ACK_SHIFT                    (0x00000007U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_BYPASS_ACK_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_BYPASS_ACK_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_M1CHANGE_ACK_MASK                   (0x00000200U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_M1CHANGE_ACK_SHIFT                  (0x00000009U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_M1CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_M1CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_M2CHANGE_ACK_MASK                   (0x00000400U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_M2CHANGE_ACK_SHIFT                  (0x0000000AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_M2CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_M2CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_M3CHANGE_ACK_MASK                   (0x00000800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_M3CHANGE_ACK_SHIFT                  (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_M3CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_M3CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_BYPASS_CLKTYPE_MASK                 (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_BYPASS_CLKTYPE_SHIFT                (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_BYPASS_CLKTYPE_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_BYPASS_CLKTYPE_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_TVALID_MASK                         (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_TVALID_SHIFT                        (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_TVALID_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_TVALID_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_LOCK2_MASK                          (0x00400000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_LOCK2_SHIFT                         (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_LOCK2_RESETVAL                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_LOCK2_MAX                           (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_HIGHJITTER_MASK                     (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_HIGHJITTER_SHIFT                    (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_HIGHJITTER_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_HIGHJITTER_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_PHASELOCK_MASK                      (0x01000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_PHASELOCK_SHIFT                     (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_PHASELOCK_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_PHASELOCK_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_FREQLOCK_MASK                       (0x02000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_FREQLOCK_SHIFT                      (0x00000019U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_FREQLOCK_RESETVAL                   (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_FREQLOCK_MAX                        (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_LOSSREF_MASK                        (0x04000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_LOSSREF_SHIFT                       (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_LOSSREF_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_LOSSREF_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_RECAL_MASK                          (0x08000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_RECAL_SHIFT                         (0x0000001BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_RECAL_RESETVAL                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_RECAL_MAX                           (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_SSC_ACK_MASK                        (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_SSC_ACK_SHIFT                       (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_SSC_ACK_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_SSC_ACK_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_STAT_RESETVAL                            (0x00000000U)

/* PLL5_PLL_PWR_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISO_PRE_Z_MASK                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISO_PRE_Z_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISO_PRE_Z_RESETVAL              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISO_PRE_Z_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISOSCAN_PRE_Z_MASK              (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISOSCAN_PRE_Z_SHIFT             (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISOSCAN_PRE_Z_RESETVAL          (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISOSCAN_PRE_Z_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_OFFMODE_MASK                    (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_OFFMODE_SHIFT                   (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_OFFMODE_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_OFFMODE_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISO_CLR_MASK                    (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISO_CLR_SHIFT                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISO_CLR_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISO_CLR_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISOSCAN_CLR_MASK                (0x00020000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISOSCAN_CLR_SHIFT               (0x00000011U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISOSCAN_CLR_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISOSCAN_CLR_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISORET_MASK                     (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISORET_SHIFT                    (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISORET_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISORET_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISO_SCAN_MASK                   (0x00200000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISO_SCAN_SHIFT                  (0x00000015U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISO_SCAN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_ISO_SCAN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_RET_MASK                        (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_RET_SHIFT                       (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_RET_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_RET_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_PGOODIN_MASK                    (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_PGOODIN_SHIFT                   (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_PGOODIN_RESETVAL                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_PGOODIN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_PONIN_MASK                      (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_PONIN_SHIFT                     (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_PONIN_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_PONIN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_CTRL_RESETVAL                        (0xC0000003U)

/* PLL5_PLL_PWR_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_LDOPWDN_MASK                    (0x00004000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_LDOPWDN_SHIFT                   (0x0000000EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_LDOPWDN_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_LDOPWDN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_TICOPWDN_MASK                   (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_TICOPWDN_SHIFT                  (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_TICOPWDN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_TICOPWDN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_PGOODOUT_MASK                   (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_PGOODOUT_SHIFT                  (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_PGOODOUT_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_PGOODOUT_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_PONOUT_MASK                     (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_PONOUT_SHIFT                    (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_PONOUT_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_PONOUT_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_PWR_STAT_RESETVAL                        (0x00000000U)

/* PLL5_PLL_SS_SPREAD */

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_MASK          (0x0003FFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_SHIFT         (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_RESETVAL      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_MAX           (0x0003FFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_SPREAD_DELTA_MSTEP_INT_MASK           (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_SPREAD_DELTA_MSTEP_INT_SHIFT          (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_SPREAD_DELTA_MSTEP_INT_RESETVAL       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_SPREAD_DELTA_MSTEP_INT_MAX            (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_SPREAD_RESETVAL                       (0x00000000U)

/* PLL5_PLL_SS_MODFREQ */

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_MASK         (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_SHIFT        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_RESETVAL     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_MAX          (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_MASK          (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_SHIFT         (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_RESETVAL      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_MAX           (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL5_PLL_SS_MODFREQ_RESETVAL                      (0x00000000U)

/* PLL6_KICK0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL6_KICK0_KICK0_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_KICK0_KICK0_VAL_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_KICK0_KICK0_VAL_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_KICK0_KICK0_VAL_MAX                          (0xFFFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_KICK0_RESETVAL                               (0x00000000U)

/* PLL6_KICK1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL6_KICK1_KICK1_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_KICK1_KICK1_VAL_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_KICK1_KICK1_VAL_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_KICK1_KICK1_VAL_MAX                          (0xFFFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_KICK1_RESETVAL                               (0x00000000U)

/* PLL6_PLL_FREQ_CTL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL0_NDIV_MASK                      (0x000000FFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL0_NDIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL0_NDIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL0_NDIV_MAX                       (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL0_M_INT_MULT_MASK                (0x000FFF00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL0_M_INT_MULT_SHIFT               (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL0_M_INT_MULT_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL0_M_INT_MULT_MAX                 (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL0_SD_DIV_MASK                    (0xFF000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL0_SD_DIV_SHIFT                   (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL0_SD_DIV_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL0_SD_DIV_MAX                     (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL0_RESETVAL                       (0x00000000U)

/* PLL6_PLL_FREQ_CTL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL1_M_FRAC_MULT_MASK               (0x0003FFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL1_M_FRAC_MULT_SHIFT              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL1_M_FRAC_MULT_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL1_M_FRAC_MULT_MAX                (0x0003FFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL1_SEL_FREQ_DCO_MASK              (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL1_SEL_FREQ_DCO_SHIFT             (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL1_SEL_FREQ_DCO_RESETVAL          (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL1_SEL_FREQ_DCO_MAX               (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_FREQ_CTL1_RESETVAL                       (0x00000000U)

/* PLL6_PLL_CLK_DIV */

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CLK_DIV_M1_DIV_MASK                      (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CLK_DIV_M1_DIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CLK_DIV_M1_DIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CLK_DIV_M1_DIV_MAX                       (0x0000000FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CLK_DIV_M2_DIV_MASK                      (0x00007F00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CLK_DIV_M2_DIV_SHIFT                     (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CLK_DIV_M2_DIV_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CLK_DIV_M2_DIV_MAX                       (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CLK_DIV_M3_DIV_MASK                      (0x001F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CLK_DIV_M3_DIV_SHIFT                     (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CLK_DIV_M3_DIV_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CLK_DIV_M3_DIV_MAX                       (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CLK_DIV_RESETVAL                         (0x00010100U)

/* PLL6_PLL_PROG */

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PROG_TINIT_Z_MASK                        (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PROG_TINIT_Z_SHIFT                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PROG_TINIT_Z_RESETVAL                    (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PROG_TINIT_Z_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PROG_TENABLE_MASK                        (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PROG_TENABLE_SHIFT                       (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PROG_TENABLE_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PROG_TENABLE_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PROG_TENABLEDIV_MASK                     (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PROG_TENABLEDIV_SHIFT                    (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PROG_TENABLEDIV_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PROG_TENABLEDIV_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PROG_RESETVAL                            (0x00000001U)

/* PLL6_PLL_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUTBYPASS_EN_MASK                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUTBYPASS_EN_SHIFT               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUTBYPASS_EN_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUTBYPASS_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUTHIF_EN_MASK                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUTHIF_EN_SHIFT                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUTHIF_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUTHIF_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUTX2_EN_MASK                    (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUTX2_EN_SHIFT                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUTX2_EN_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUTX2_EN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUT_EN_MASK                      (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUT_EN_SHIFT                     (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUT_EN_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUT_EN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKDCOLDO_EN_MASK                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKDCOLDO_EN_SHIFT                  (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKDCOLDO_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKDCOLDO_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUTLDO_EN_MASK                   (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUTLDO_EN_SHIFT                  (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUTLDO_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKOUTLDO_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_IDLE_MASK                           (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_IDLE_SHIFT                          (0x00000007U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_IDLE_RESETVAL                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_IDLE_MAX                            (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKRAMP_RATE_MASK                   (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKRAMP_RATE_SHIFT                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKRAMP_RATE_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKRAMP_RATE_MAX                    (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKRAMP_LEVEL_MASK                  (0x00003000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKRAMP_LEVEL_SHIFT                 (0x0000000CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKRAMP_LEVEL_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKRAMP_LEVEL_MAX                   (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_ULOWCLK_EN_MASK                     (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_ULOWCLK_EN_SHIFT                    (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_ULOWCLK_EN_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_ULOWCLK_EN_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_DRIFTGUARD_EN_MASK                  (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_DRIFTGUARD_EN_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_DRIFTGUARD_EN_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_DRIFTGUARD_EN_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_REGM4X_EN_MASK                      (0x00020000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_REGM4X_EN_SHIFT                     (0x00000011U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_REGM4X_EN_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_REGM4X_EN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKINPHIF_SEL_MASK                  (0x00040000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKINPHIF_SEL_SHIFT                 (0x00000012U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKINPHIF_SEL_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CLKINPHIF_SEL_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_RELOCK_RAMP_EN_MASK                 (0x00080000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_RELOCK_RAMP_EN_SHIFT                (0x00000013U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_RELOCK_RAMP_EN_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_RELOCK_RAMP_EN_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CYCLESLIP_EN_MASK                   (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CYCLESLIP_EN_SHIFT                  (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CYCLESLIP_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_CYCLESLIP_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_LOWCURRSTBY_MASK                    (0x00200000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_LOWCURRSTBY_SHIFT                   (0x00000015U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_LOWCURRSTBY_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_LOWCURRSTBY_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_RELAXED_LOCK_MASK                   (0x00400000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_RELAXED_LOCK_SHIFT                  (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_RELAXED_LOCK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_RELAXED_LOCK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_LPMODE_MASK                         (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_LPMODE_SHIFT                        (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_LPMODE_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_LPMODE_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_BW_CONTROL_MASK                     (0x03000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_BW_CONTROL_SHIFT                    (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_BW_CONTROL_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_BW_CONTROL_MAX                      (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_BW_INCR_DECRZ_MASK                  (0x04000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_BW_INCR_DECRZ_SHIFT                 (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_BW_INCR_DECRZ_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_BW_INCR_DECRZ_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_DC_CORRECTOR_EN_MASK                (0x08000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_DC_CORRECTOR_EN_SHIFT               (0x0000001BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_DC_CORRECTOR_EN_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_DC_CORRECTOR_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_DOWNSPREAD_MASK                     (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_DOWNSPREAD_SHIFT                    (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_DOWNSPREAD_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_DOWNSPREAD_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_SSC_EN_MASK                         (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_SSC_EN_SHIFT                        (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_SSC_EN_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_SSC_EN_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_CTRL_RESETVAL                            (0x00000000U)

/* PLL6_PLL_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUTBYPASS_EN_ACK_MASK            (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUTBYPASS_EN_ACK_SHIFT           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUTBYPASS_EN_ACK_RESETVAL        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUTBYPASS_EN_ACK_MAX             (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUTHIF_EN_ACK_MASK               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUTHIF_EN_ACK_SHIFT              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUTHIF_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUTHIF_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUTX2_EN_ACK_MASK                (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUTX2_EN_ACK_SHIFT               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUTX2_EN_ACK_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUTX2_EN_ACK_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUT_EN_ACK_MASK                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUT_EN_ACK_SHIFT                 (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUT_EN_ACK_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUT_EN_ACK_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKDCOLDO_EN_ACK_MASK               (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKDCOLDO_EN_ACK_SHIFT              (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKDCOLDO_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKDCOLDO_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUTLDO_EN_ACK_MASK               (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUTLDO_EN_ACK_SHIFT              (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUTLDO_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_CLKOUTLDO_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_BYPASS_ACK_MASK                     (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_BYPASS_ACK_SHIFT                    (0x00000007U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_BYPASS_ACK_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_BYPASS_ACK_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_M1CHANGE_ACK_MASK                   (0x00000200U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_M1CHANGE_ACK_SHIFT                  (0x00000009U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_M1CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_M1CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_M2CHANGE_ACK_MASK                   (0x00000400U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_M2CHANGE_ACK_SHIFT                  (0x0000000AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_M2CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_M2CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_M3CHANGE_ACK_MASK                   (0x00000800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_M3CHANGE_ACK_SHIFT                  (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_M3CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_M3CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_BYPASS_CLKTYPE_MASK                 (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_BYPASS_CLKTYPE_SHIFT                (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_BYPASS_CLKTYPE_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_BYPASS_CLKTYPE_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_TVALID_MASK                         (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_TVALID_SHIFT                        (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_TVALID_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_TVALID_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_LOCK2_MASK                          (0x00400000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_LOCK2_SHIFT                         (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_LOCK2_RESETVAL                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_LOCK2_MAX                           (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_HIGHJITTER_MASK                     (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_HIGHJITTER_SHIFT                    (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_HIGHJITTER_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_HIGHJITTER_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_PHASELOCK_MASK                      (0x01000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_PHASELOCK_SHIFT                     (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_PHASELOCK_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_PHASELOCK_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_FREQLOCK_MASK                       (0x02000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_FREQLOCK_SHIFT                      (0x00000019U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_FREQLOCK_RESETVAL                   (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_FREQLOCK_MAX                        (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_LOSSREF_MASK                        (0x04000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_LOSSREF_SHIFT                       (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_LOSSREF_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_LOSSREF_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_RECAL_MASK                          (0x08000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_RECAL_SHIFT                         (0x0000001BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_RECAL_RESETVAL                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_RECAL_MAX                           (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_SSC_ACK_MASK                        (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_SSC_ACK_SHIFT                       (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_SSC_ACK_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_SSC_ACK_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_STAT_RESETVAL                            (0x00000000U)

/* PLL6_PLL_PWR_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISO_PRE_Z_MASK                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISO_PRE_Z_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISO_PRE_Z_RESETVAL              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISO_PRE_Z_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISOSCAN_PRE_Z_MASK              (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISOSCAN_PRE_Z_SHIFT             (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISOSCAN_PRE_Z_RESETVAL          (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISOSCAN_PRE_Z_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_OFFMODE_MASK                    (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_OFFMODE_SHIFT                   (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_OFFMODE_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_OFFMODE_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISO_CLR_MASK                    (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISO_CLR_SHIFT                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISO_CLR_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISO_CLR_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISOSCAN_CLR_MASK                (0x00020000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISOSCAN_CLR_SHIFT               (0x00000011U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISOSCAN_CLR_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISOSCAN_CLR_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISORET_MASK                     (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISORET_SHIFT                    (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISORET_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISORET_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISO_SCAN_MASK                   (0x00200000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISO_SCAN_SHIFT                  (0x00000015U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISO_SCAN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_ISO_SCAN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_RET_MASK                        (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_RET_SHIFT                       (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_RET_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_RET_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_PGOODIN_MASK                    (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_PGOODIN_SHIFT                   (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_PGOODIN_RESETVAL                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_PGOODIN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_PONIN_MASK                      (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_PONIN_SHIFT                     (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_PONIN_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_PONIN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_CTRL_RESETVAL                        (0xC0000003U)

/* PLL6_PLL_PWR_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_LDOPWDN_MASK                    (0x00004000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_LDOPWDN_SHIFT                   (0x0000000EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_LDOPWDN_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_LDOPWDN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_TICOPWDN_MASK                   (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_TICOPWDN_SHIFT                  (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_TICOPWDN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_TICOPWDN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_PGOODOUT_MASK                   (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_PGOODOUT_SHIFT                  (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_PGOODOUT_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_PGOODOUT_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_PONOUT_MASK                     (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_PONOUT_SHIFT                    (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_PONOUT_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_PONOUT_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_PWR_STAT_RESETVAL                        (0x00000000U)

/* PLL6_PLL_SS_SPREAD */

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_MASK          (0x0003FFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_SHIFT         (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_RESETVAL      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_MAX           (0x0003FFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_SPREAD_DELTA_MSTEP_INT_MASK           (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_SPREAD_DELTA_MSTEP_INT_SHIFT          (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_SPREAD_DELTA_MSTEP_INT_RESETVAL       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_SPREAD_DELTA_MSTEP_INT_MAX            (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_SPREAD_RESETVAL                       (0x00000000U)

/* PLL6_PLL_SS_MODFREQ */

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_MASK         (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_SHIFT        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_RESETVAL     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_MAX          (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_MASK          (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_SHIFT         (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_RESETVAL      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_MAX           (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL6_PLL_SS_MODFREQ_RESETVAL                      (0x00000000U)

/* PLL7_KICK0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL7_KICK0_KICK0_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_KICK0_KICK0_VAL_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_KICK0_KICK0_VAL_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_KICK0_KICK0_VAL_MAX                          (0xFFFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_KICK0_RESETVAL                               (0x00000000U)

/* PLL7_KICK1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL7_KICK1_KICK1_VAL_MASK                         (0xFFFFFFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_KICK1_KICK1_VAL_SHIFT                        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_KICK1_KICK1_VAL_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_KICK1_KICK1_VAL_MAX                          (0xFFFFFFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_KICK1_RESETVAL                               (0x00000000U)

/* PLL7_PLL_FREQ_CTL0 */

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL0_NDIV_MASK                      (0x000000FFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL0_NDIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL0_NDIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL0_NDIV_MAX                       (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL0_M_INT_MULT_MASK                (0x000FFF00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL0_M_INT_MULT_SHIFT               (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL0_M_INT_MULT_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL0_M_INT_MULT_MAX                 (0x00000FFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL0_SD_DIV_MASK                    (0xFF000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL0_SD_DIV_SHIFT                   (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL0_SD_DIV_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL0_SD_DIV_MAX                     (0x000000FFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL0_RESETVAL                       (0x00000000U)

/* PLL7_PLL_FREQ_CTL1 */

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL1_M_FRAC_MULT_MASK               (0x0003FFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL1_M_FRAC_MULT_SHIFT              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL1_M_FRAC_MULT_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL1_M_FRAC_MULT_MAX                (0x0003FFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL1_SEL_FREQ_DCO_MASK              (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL1_SEL_FREQ_DCO_SHIFT             (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL1_SEL_FREQ_DCO_RESETVAL          (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL1_SEL_FREQ_DCO_MAX               (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_FREQ_CTL1_RESETVAL                       (0x00000000U)

/* PLL7_PLL_CLK_DIV */

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CLK_DIV_M1_DIV_MASK                      (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CLK_DIV_M1_DIV_SHIFT                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CLK_DIV_M1_DIV_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CLK_DIV_M1_DIV_MAX                       (0x0000000FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CLK_DIV_M2_DIV_MASK                      (0x00007F00U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CLK_DIV_M2_DIV_SHIFT                     (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CLK_DIV_M2_DIV_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CLK_DIV_M2_DIV_MAX                       (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CLK_DIV_M3_DIV_MASK                      (0x001F0000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CLK_DIV_M3_DIV_SHIFT                     (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CLK_DIV_M3_DIV_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CLK_DIV_M3_DIV_MAX                       (0x0000001FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CLK_DIV_RESETVAL                         (0x00010100U)

/* PLL7_PLL_PROG */

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PROG_TINIT_Z_MASK                        (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PROG_TINIT_Z_SHIFT                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PROG_TINIT_Z_RESETVAL                    (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PROG_TINIT_Z_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PROG_TENABLE_MASK                        (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PROG_TENABLE_SHIFT                       (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PROG_TENABLE_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PROG_TENABLE_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PROG_TENABLEDIV_MASK                     (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PROG_TENABLEDIV_SHIFT                    (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PROG_TENABLEDIV_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PROG_TENABLEDIV_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PROG_RESETVAL                            (0x00000001U)

/* PLL7_PLL_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUTBYPASS_EN_MASK                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUTBYPASS_EN_SHIFT               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUTBYPASS_EN_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUTBYPASS_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUTHIF_EN_MASK                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUTHIF_EN_SHIFT                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUTHIF_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUTHIF_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUTX2_EN_MASK                    (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUTX2_EN_SHIFT                   (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUTX2_EN_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUTX2_EN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUT_EN_MASK                      (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUT_EN_SHIFT                     (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUT_EN_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUT_EN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKDCOLDO_EN_MASK                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKDCOLDO_EN_SHIFT                  (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKDCOLDO_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKDCOLDO_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUTLDO_EN_MASK                   (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUTLDO_EN_SHIFT                  (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUTLDO_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKOUTLDO_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_IDLE_MASK                           (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_IDLE_SHIFT                          (0x00000007U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_IDLE_RESETVAL                       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_IDLE_MAX                            (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKRAMP_RATE_MASK                   (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKRAMP_RATE_SHIFT                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKRAMP_RATE_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKRAMP_RATE_MAX                    (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKRAMP_LEVEL_MASK                  (0x00003000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKRAMP_LEVEL_SHIFT                 (0x0000000CU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKRAMP_LEVEL_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKRAMP_LEVEL_MAX                   (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_ULOWCLK_EN_MASK                     (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_ULOWCLK_EN_SHIFT                    (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_ULOWCLK_EN_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_ULOWCLK_EN_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_DRIFTGUARD_EN_MASK                  (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_DRIFTGUARD_EN_SHIFT                 (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_DRIFTGUARD_EN_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_DRIFTGUARD_EN_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_REGM4X_EN_MASK                      (0x00020000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_REGM4X_EN_SHIFT                     (0x00000011U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_REGM4X_EN_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_REGM4X_EN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKINPHIF_SEL_MASK                  (0x00040000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKINPHIF_SEL_SHIFT                 (0x00000012U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKINPHIF_SEL_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CLKINPHIF_SEL_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_RELOCK_RAMP_EN_MASK                 (0x00080000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_RELOCK_RAMP_EN_SHIFT                (0x00000013U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_RELOCK_RAMP_EN_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_RELOCK_RAMP_EN_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CYCLESLIP_EN_MASK                   (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CYCLESLIP_EN_SHIFT                  (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CYCLESLIP_EN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_CYCLESLIP_EN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_LOWCURRSTBY_MASK                    (0x00200000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_LOWCURRSTBY_SHIFT                   (0x00000015U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_LOWCURRSTBY_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_LOWCURRSTBY_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_RELAXED_LOCK_MASK                   (0x00400000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_RELAXED_LOCK_SHIFT                  (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_RELAXED_LOCK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_RELAXED_LOCK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_LPMODE_MASK                         (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_LPMODE_SHIFT                        (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_LPMODE_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_LPMODE_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_BW_CONTROL_MASK                     (0x03000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_BW_CONTROL_SHIFT                    (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_BW_CONTROL_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_BW_CONTROL_MAX                      (0x00000003U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_BW_INCR_DECRZ_MASK                  (0x04000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_BW_INCR_DECRZ_SHIFT                 (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_BW_INCR_DECRZ_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_BW_INCR_DECRZ_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_DC_CORRECTOR_EN_MASK                (0x08000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_DC_CORRECTOR_EN_SHIFT               (0x0000001BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_DC_CORRECTOR_EN_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_DC_CORRECTOR_EN_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_DOWNSPREAD_MASK                     (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_DOWNSPREAD_SHIFT                    (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_DOWNSPREAD_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_DOWNSPREAD_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_SSC_EN_MASK                         (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_SSC_EN_SHIFT                        (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_SSC_EN_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_SSC_EN_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_CTRL_RESETVAL                            (0x00000000U)

/* PLL7_PLL_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUTBYPASS_EN_ACK_MASK            (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUTBYPASS_EN_ACK_SHIFT           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUTBYPASS_EN_ACK_RESETVAL        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUTBYPASS_EN_ACK_MAX             (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUTHIF_EN_ACK_MASK               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUTHIF_EN_ACK_SHIFT              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUTHIF_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUTHIF_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUTX2_EN_ACK_MASK                (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUTX2_EN_ACK_SHIFT               (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUTX2_EN_ACK_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUTX2_EN_ACK_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUT_EN_ACK_MASK                  (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUT_EN_ACK_SHIFT                 (0x00000003U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUT_EN_ACK_RESETVAL              (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUT_EN_ACK_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKDCOLDO_EN_ACK_MASK               (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKDCOLDO_EN_ACK_SHIFT              (0x00000004U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKDCOLDO_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKDCOLDO_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUTLDO_EN_ACK_MASK               (0x00000020U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUTLDO_EN_ACK_SHIFT              (0x00000005U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUTLDO_EN_ACK_RESETVAL           (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_CLKOUTLDO_EN_ACK_MAX                (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_BYPASS_ACK_MASK                     (0x00000080U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_BYPASS_ACK_SHIFT                    (0x00000007U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_BYPASS_ACK_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_BYPASS_ACK_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_M1CHANGE_ACK_MASK                   (0x00000200U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_M1CHANGE_ACK_SHIFT                  (0x00000009U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_M1CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_M1CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_M2CHANGE_ACK_MASK                   (0x00000400U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_M2CHANGE_ACK_SHIFT                  (0x0000000AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_M2CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_M2CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_M3CHANGE_ACK_MASK                   (0x00000800U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_M3CHANGE_ACK_SHIFT                  (0x0000000BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_M3CHANGE_ACK_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_M3CHANGE_ACK_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_BYPASS_CLKTYPE_MASK                 (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_BYPASS_CLKTYPE_SHIFT                (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_BYPASS_CLKTYPE_RESETVAL             (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_BYPASS_CLKTYPE_MAX                  (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_TVALID_MASK                         (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_TVALID_SHIFT                        (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_TVALID_RESETVAL                     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_TVALID_MAX                          (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_LOCK2_MASK                          (0x00400000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_LOCK2_SHIFT                         (0x00000016U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_LOCK2_RESETVAL                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_LOCK2_MAX                           (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_HIGHJITTER_MASK                     (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_HIGHJITTER_SHIFT                    (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_HIGHJITTER_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_HIGHJITTER_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_PHASELOCK_MASK                      (0x01000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_PHASELOCK_SHIFT                     (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_PHASELOCK_RESETVAL                  (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_PHASELOCK_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_FREQLOCK_MASK                       (0x02000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_FREQLOCK_SHIFT                      (0x00000019U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_FREQLOCK_RESETVAL                   (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_FREQLOCK_MAX                        (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_LOSSREF_MASK                        (0x04000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_LOSSREF_SHIFT                       (0x0000001AU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_LOSSREF_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_LOSSREF_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_RECAL_MASK                          (0x08000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_RECAL_SHIFT                         (0x0000001BU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_RECAL_RESETVAL                      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_RECAL_MAX                           (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_SSC_ACK_MASK                        (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_SSC_ACK_SHIFT                       (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_SSC_ACK_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_SSC_ACK_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_STAT_RESETVAL                            (0x00000000U)

/* PLL7_PLL_PWR_CTRL */

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISO_PRE_Z_MASK                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISO_PRE_Z_SHIFT                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISO_PRE_Z_RESETVAL              (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISO_PRE_Z_MAX                   (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISOSCAN_PRE_Z_MASK              (0x00000002U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISOSCAN_PRE_Z_SHIFT             (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISOSCAN_PRE_Z_RESETVAL          (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISOSCAN_PRE_Z_MAX               (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_OFFMODE_MASK                    (0x00000100U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_OFFMODE_SHIFT                   (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_OFFMODE_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_OFFMODE_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISO_CLR_MASK                    (0x00010000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISO_CLR_SHIFT                   (0x00000010U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISO_CLR_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISO_CLR_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISOSCAN_CLR_MASK                (0x00020000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISOSCAN_CLR_SHIFT               (0x00000011U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISOSCAN_CLR_RESETVAL            (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISOSCAN_CLR_MAX                 (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISORET_MASK                     (0x00100000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISORET_SHIFT                    (0x00000014U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISORET_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISORET_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISO_SCAN_MASK                   (0x00200000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISO_SCAN_SHIFT                  (0x00000015U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISO_SCAN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_ISO_SCAN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_RET_MASK                        (0x00800000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_RET_SHIFT                       (0x00000017U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_RET_RESETVAL                    (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_RET_MAX                         (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_PGOODIN_MASK                    (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_PGOODIN_SHIFT                   (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_PGOODIN_RESETVAL                (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_PGOODIN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_PONIN_MASK                      (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_PONIN_SHIFT                     (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_PONIN_RESETVAL                  (0x00000001U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_PONIN_MAX                       (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_CTRL_RESETVAL                        (0xC0000003U)

/* PLL7_PLL_PWR_STAT */

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_LDOPWDN_MASK                    (0x00004000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_LDOPWDN_SHIFT                   (0x0000000EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_LDOPWDN_RESETVAL                (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_LDOPWDN_MAX                     (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_TICOPWDN_MASK                   (0x00008000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_TICOPWDN_SHIFT                  (0x0000000FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_TICOPWDN_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_TICOPWDN_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_PGOODOUT_MASK                   (0x40000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_PGOODOUT_SHIFT                  (0x0000001EU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_PGOODOUT_RESETVAL               (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_PGOODOUT_MAX                    (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_PONOUT_MASK                     (0x80000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_PONOUT_SHIFT                    (0x0000001FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_PONOUT_RESETVAL                 (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_PONOUT_MAX                      (0x00000001U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_PWR_STAT_RESETVAL                        (0x00000000U)

/* PLL7_PLL_SS_SPREAD */

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_MASK          (0x0003FFFFU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_SHIFT         (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_RESETVAL      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_SPREAD_DELTA_MSTEP_FRAC_MAX           (0x0003FFFFU)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_SPREAD_DELTA_MSTEP_INT_MASK           (0x07000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_SPREAD_DELTA_MSTEP_INT_SHIFT          (0x00000018U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_SPREAD_DELTA_MSTEP_INT_RESETVAL       (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_SPREAD_DELTA_MSTEP_INT_MAX            (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_SPREAD_RESETVAL                       (0x00000000U)

/* PLL7_PLL_SS_MODFREQ */

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_MASK         (0x0000007FU)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_SHIFT        (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_RESETVAL     (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_MODFREQ_MODFREQ_DIV_MANT_MAX          (0x0000007FU)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_MASK          (0x00000700U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_SHIFT         (0x00000008U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_RESETVAL      (0x00000000U)
#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_MODFREQ_MODFREQ_DIV_EXP_MAX           (0x00000007U)

#define CSL_MAIN_PLL_MMR_CFG_PLL7_PLL_SS_MODFREQ_RESETVAL                      (0x00000000U)

#ifdef __cplusplus
}
#endif
#endif
