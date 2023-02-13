/* Copyright (c) 2022-23 Texas Instruments Incorporated
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

 /**
 *  \file     ecc_bus_safety.c
 *
 *  \brief    This file contains Ecc Bus Safety for all the nodes in MSS and DSS example code.
 *
 *  \details  ECC_BUS_SAFETY APP
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "ecc_bus_safety.h"
/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
static void ecc_bus_safety_testExecute(void);
/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/* Define the  application interface */
typedef struct sdlEccBusSafetyApp_s
{
    int32_t  (*application)(void);   /* The code that runs the application */
    char      *name;                  /* The application name */
    int32_t    status;                /* App Status */
    int64_t    test_time;            /* App Test Time */
} sdlEccBusSafetyApp_t;

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/

sdlEccBusSafetyApp_t  sdlEccBusSafetyAppTestList[] = {

#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)
    /* DSS MD0 FIFO */
    {SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_DED_Test_Polling,        "MDO_FIFO_DED_Test in Polling Method",           SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_RED_Test_Polling,        "MDO_FIFO_RED_Test in Polling Method",           SDL_APP_NOT_RUN },
#endif
#endif

#if defined (SOC_AM273X) ||  defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)
    /* DSS MCRC */
    {SDL_ECC_BUS_SAFETY_DSS_MCRC_SEC_Test,                    "MCRC_SEC_Test in Interrupt Method",              SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MCRC_DED_Test,                    "MCRC_DED_Test in Interrupt Method",              SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MCRC_RED_Test,                    "MCRC_RED_Test in Interrupt Method",              SDL_APP_NOT_RUN },
    /* DSS L3 BANKA */
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_SEC_Test,                "DSS_L3_BANKA_SEC_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_DED_Test,                "DSS_L3_BANKA_DED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_RED_Test,                "DSS_L3_BANKA_RED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    /* DSS L3 BANKB */
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_SEC_Test,                "DSS_L3_BANKB_SEC_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_DED_Test,                "DSS_L3_BANKB_DED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_RED_Test,                "DSS_L3_BANKB_RED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    /* DSS L3 BANKC */
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_SEC_Test,                "DSS_L3_BANKC_SEC_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_DED_Test,                "DSS_L3_BANKC_DED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_RED_Test,                "DSS_L3_BANKC_RED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
#if defined (SOC_AM273X)
    /* DSS L3 BANKD */
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_SEC_Test,                "DSS_L3_BANKD_SEC_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_DED_Test,                "DSS_L3_BANKD_DED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_RED_Test,                "DSS_L3_BANKD_RED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
#endif
    /* DSS HWA DMA 0 */
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_SEC_Test,                "DSS_HWA_DMA0_SEC_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_DED_Test,                "DSS_HWA_DMA0_DED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_RED_Test,                "DSS_HWA_DMA0_RED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    /* DSS HWA DMA 1 */
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_SEC_Test,                "DSS_HWA_DMA1_SEC_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_DED_Test,                "DSS_HWA_DMA1_DED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_RED_Test,                "DSS_HWA_DMA1_RED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    /* DSS MBOX */
    {SDL_ECC_BUS_SAFETY_DSS_MBOX_SEC_Test,                    "DSS_MBOX_SEC_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MBOX_DED_Test,                    "DSS_MBOX_DED_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MBOX_RED_Test,                    "DSS_MBOX_RED_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    /* DSS CBUFF FIFO */
    {SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_SEC_Test,              "CBUFF_FIFO_SEC_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_DED_Test,              "CBUFF_FIFO_DED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_RED_Test,              "CBUFF_FIFO_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC A0 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC A1 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC B0 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC B1 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC C0 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC C1 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC C2 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC C3 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC C4 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC C5 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC A0 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_SEC_Test,              "TPTC_A0_RD_SEC_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_DED_Test,              "TPTC_A0_RD_DED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_RED_Test,              "TPTC_A0_RD_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC A1 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_SEC_Test,              "TPTC_A1_RD_SEC_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_DED_Test,              "TPTC_A1_RD_DED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_RED_Test,              "TPTC_A1_RD_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC B0 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_SEC_Test,              "TPTC_B0_RD_SEC_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_DED_Test,              "TPTC_B0_RD_DED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_RED_Test,              "TPTC_B0_RD_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC B1 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_SEC_Test,              "TPTC_B1_RD_SEC_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_DED_Test,              "TPTC_B1_RD_DED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_RED_Test,              "TPTC_B1_RD_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC C0 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_SEC_Test,              "TPTC_C0_RD_SEC_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_DED_Test,              "TPTC_C0_RD_DED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_RED_Test,              "TPTC_C0_RD_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC C0 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_SEC_Test,              "TPTC_C1_RD_SEC_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_DED_Test,              "TPTC_C1_RD_DED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_RED_Test,              "TPTC_C1_RD_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC C1 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_SEC_Test,              "TPTC_C2_RD_SEC_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_DED_Test,              "TPTC_C2_RD_DED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_RED_Test,              "TPTC_C2_RD_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC C2 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_SEC_Test,              "TPTC_C3_RD_SEC_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_DED_Test,              "TPTC_C3_RD_DED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_RED_Test,              "TPTC_C3_RD_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC C4 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_SEC_Test,              "TPTC_C4_RD_SEC_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_DED_Test,              "TPTC_C4_RD_DED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_RED_Test,              "TPTC_C4_RD_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS TPTC C5 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_SEC_Test,              "TPTC_C5_RD_SEC_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_DED_Test,              "TPTC_C5_RD_DED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_RED_Test,              "TPTC_C5_RD_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* DSS PCR */
    {SDL_ECC_BUS_SAFETY_DSS_PCR_SEC_Test,                     "PCR_SEC_Test in Interrupt Method",               SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_PCR_DED_Test,                     "PCR_DED_Test in Interrupt Method",               SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_PCR_RED_Test,                     "PCR_RED_Test in Interrupt Method",               SDL_APP_NOT_RUN },
    /* DSS DSP SDMA */
    {SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_SEC_Test,                "DSP_SDMA_SEC_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_DED_Test,                "DSP_SDMA_DED_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_RED_Test,                "DSP_SDMA_RED_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    /* DSS DSP MDMA */
    {SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_RED_Test,                "DSP_MDMA_RED_Test in Interrupt Method",          SDL_APP_NOT_RUN },
#endif
#endif

#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)
    /* DSS MD0 FIFO */
    {SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_SEC_Test,                "MDO_FIFO_SEC_Test in Interrupt Method",          SDL_APP_NOT_RUN },
#endif
#endif

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#if defined SUBSYS_MSS
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_RED_Test,                "CR5A_AHB_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_RED_Test,                "CR5B_AHB_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    /* MSS TPTC A0 WR */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* MSS TPTC A1 WR */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test,              "TPTC_A1_WR_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
     /* MSS TPTC B0 WR */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR_RED_Test,              "TPTC_B0_WR_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
       /* MSS TPTC A0 RD */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_Test,              "TPTC_A0_RD_SEC_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_Test,              "TPTC_A0_RD_DED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_Test,              "TPTC_A0_RD_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
     /* MSS TPTC A1 RD */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_Test,              "TPTC_A1_RD_SEC_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_Test,              "TPTC_A1_RD_DED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_Test,              "TPTC_A1_RD_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
     /* MSS TPTC B0 RD */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_SEC_Test,              "TPTC_B0_RD_SEC_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_DED_Test,              "TPTC_B0_RD_DED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_RED_Test,              "TPTC_B0_RD_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
     /* MSS MBOX */
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_SEC_Test,                    "MSS_MBOX_SEC_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_Test,                    "MSS_MBOX_DED_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_RED_Test,                    "MSS_MBOX_RED_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    /*MSS L2 A*/
    {SDL_ECC_BUS_SAFETY_MSS_L2_A_RED_Test,                    "MSS_L2_A_RED_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    /*MSS L2 B*/
    {SDL_ECC_BUS_SAFETY_MSS_L2_B_RED_Test,                    "MSS_L2_B_RED_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    /* Node MSS_DMM_SLV */
    {SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_SEC_Test,                 "MSS_DMM_SLV_SEC_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_DED_Test,                 "MSS_DMM_SLV_DED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_RED_Test,                 "MSS_DMM_SLV_RED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    /* Node MSS_DMM */
    {SDL_ECC_BUS_SAFETY_MSS_DMM_RED_Test,                     "MSS_DMM_RED_Test in Interrupt Method",           SDL_APP_NOT_RUN },
    /* Node MSS_GPADC */
    {SDL_ECC_BUS_SAFETY_MSS_GPADC_SEC_Test,                   "MSS_GPADC_SEC_Test in Interrupt Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_GPADC_SEC_Test,                   "MSS_GPADC_DED_Test in Interrupt Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_GPADC_SEC_Test,                   "MSS_GPADC_RED_Test in Interrupt Method",         SDL_APP_NOT_RUN },
    /* Node MSS_PCR */
    {SDL_ECC_BUS_SAFETY_MSS_PCR_SEC_Test,                     "MSS_PCR_SEC_Test in Interrupt Method",           SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_PCR_DED_Test,                     "MSS_PCR_DED_Test in Interrupt Method",           SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_PCR_RED_Test,                     "MSS_PCR_RED_Test in Interrupt Method",           SDL_APP_NOT_RUN },
    /* Node MSS_PCR2 */
    {SDL_ECC_BUS_SAFETY_MSS_PCR2_SEC_Test,                    "MSS_PCR2_SEC_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_PCR2_DED_Test,                    "MSS_PCR2_DED_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_PCR2_RED_Test,                    "MSS_PCR2_RED_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    /* Node MSS_CPSW */
    {SDL_ECC_BUS_SAFETY_MSS_CPSW_SEC_Test,                    "MSS_CPSW_SEC_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CPSW_RED_Test,                    "MSS_CPSW_RED_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    /* MSS CR5A_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_Test,             "MSS_CR5A_AXI_WR_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* MSS CR5B_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_Test,             "MSS_CR5B_AXI_WR_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* MSS CR5A_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_RED_Test,             "MSS_CR5A_AXI_RD_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* MSS CR5B_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_RED_Test,             "MSS_CR5B_AXI_RD_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* MSS CR5A_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_Test,              "MSS_CR5A_AXI_S_RED_Test in Interrupt Method",    SDL_APP_NOT_RUN },
    /* MSS CR5B_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_Test,              "MSS_CR5B_AXI_S_RED_Test in Interrupt Method",    SDL_APP_NOT_RUN },
#endif
#endif

#if defined (SOC_AM263X)
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_RED_Test,                "CR5A_AHB_RED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_RED_Test,                "CR5B_AHB_RED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB_RED_Test,                "CR5C_AHB_RED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB_RED_Test,                "CR5D_AHB_RED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
     /* MSS TPTC A0 WR */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* MSS TPTC A1 WR */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test,              "TPTC_A1_WR_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* MSS TPTC A0 RD */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_Test,              "TPTC_A0_RD_SEC_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_Test,              "TPTC_A0_RD_DED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_Test,              "TPTC_A0_RD_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* MSS TPTC A1 RD */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_Test,              "TPTC_A1_RD_SEC_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_Test,              "TPTC_A1_RD_DED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_Test,              "TPTC_A1_RD_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    /* MSS CR5A_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_Test,             "MSS_CR5A_AXI_WR_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* MSS CR5B_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_Test,             "MSS_CR5B_AXI_WR_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* MSS CR5C_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_RED_Test,             "MSS_CR5C_AXI_WR_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* MSS CR5D_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_RED_Test,             "MSS_CR5D_AXI_WR_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* MSS CR5A_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_SEC_Test,             "MSS_CR5A_AXI_RD_SEC_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_RED_Test,             "MSS_CR5A_AXI_RD_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* MSS CR5B_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_SEC_Test,             "MSS_CR5B_AXI_RD_SEC_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_RED_Test,             "MSS_CR5B_AXI_RD_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* MSS CR5C_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_SEC_Test,             "MSS_CR5C_AXI_RD_SEC_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_RED_Test,             "MSS_CR5C_AXI_RD_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* MSS CR5D_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_SEC_Test,             "MSS_CR5D_AXI_RD_SEC_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_RED_Test,             "MSS_CR5D_AXI_RD_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* MSS CR5A_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_SEC_Test,              "MSS_CR5A_AXI_S_SEC_Test in Interrupt Method",    SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_Test,              "MSS_CR5A_AXI_S_RED_Test in Interrupt Method",    SDL_APP_NOT_RUN },
    /* MSS CR5B_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_SEC_Test,              "MSS_CR5B_AXI_S_SEC_Test in Interrupt Method",    SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_Test,              "MSS_CR5B_AXI_S_RED_Test in Interrupt Method",    SDL_APP_NOT_RUN },
    /* MSS CR5C_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_SEC_Test,              "MSS_CR5C_AXI_S_SEC_Test in Interrupt Method",    SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_RED_Test,              "MSS_CR5C_AXI_S_RED_Test in Interrupt Method",    SDL_APP_NOT_RUN },
    /* MSS CR5D_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_SEC_Test,              "MSS_CR5D_AXI_S_SEC_Test in Interrupt Method",    SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_RED_Test,              "MSS_CR5D_AXI_S_RED_Test in Interrupt Method",    SDL_APP_NOT_RUN },
    /* Node MSS_MMC_S */
    {SDL_ECC_BUS_SAFETY_MSS_MMC_SEC_Test,                     "MSS_MMC_S_SEC_Test in Interrupt  Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MMC_DED_Test,                     "MSS_MMC_S_DED_Test in Interrupt  Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MMC_RED_Test,                     "MSS_MMC_S_RED_Test in Interrupt  Method",        SDL_APP_NOT_RUN },
    /* Node MSS_GPMC */
    {SDL_ECC_BUS_SAFETY_MSS_GPMC_SEC_Test,                    "MSS_GPMC_SEC_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_GPMC_DED_Test,                    "MSS_GPMC_DED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_GPMC_RED_Test,                    "MSS_GPMC_RED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    /* Node MSS_L2_A */
    {SDL_ECC_BUS_SAFETY_MSS_L2_A_RED_Test,                    "MSS_L2_A_RED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    /* Node MSS_L2_B */
    {SDL_ECC_BUS_SAFETY_MSS_L2_B_RED_Test,                    "MSS_L2_B_RED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    /* Node MSS_L2_C */
    {SDL_ECC_BUS_SAFETY_MSS_L2_C_RED_Test,                    "MSS_L2_C_RED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    /* Node MSS_L2_D */
    {SDL_ECC_BUS_SAFETY_MSS_L2_D_RED_Test,                    "MSS_L2_D_RED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    /* Node MAIN_VBUSP */
    {SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP_RED_Test,              "MAIN_VBUSP_RED_Test in Interrupt  Method",       SDL_APP_NOT_RUN },
    /* Node _PERI_VBUSP */
    {SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP_RED_Test,              "PERI_VBUSP_RED_Test in Interrupt  Method",       SDL_APP_NOT_RUN },
#endif
    {NULL,                                                    "TERMINATING CONDITION",                          SDL_APP_NOT_RUN }
};

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/
void ecc_bus_safety_edma_boardInit(void)
{
   Drivers_open();
   Board_driversOpen();
}

void ecc_bus_safety_edma_boardDeinit(void)
{
    Board_driversClose();
    Drivers_close();
}

static void ecc_bus_safety_testExecute(void)
{
    /* Declarations of variables */
    int32_t    result = SDL_APP_PASS;
    int32_t    count;
    uint64_t testStartTime;
    uint64_t testEndTime;
    uint64_t diffTime;
    ecc_bus_safety_edma_boardInit();
#if defined (SOC_AWR294X) || defined(SOC_AM273X)
#if defined (SUBSYS_MSS)
    for( count =0; count<24U; count++)
    {
	    result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_BUS_SAFETY_TestparamsMSS[count],NULL,NULL);
    }
#endif
#endif
#if defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)
    for( count =0; count<33U; count++)
    {
	    result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_BUS_SAFETY_TestparamsDSS[count],NULL,NULL);
    }
#endif
#endif
#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)
    for( count =0; count<33U; count++)
    {
	    result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_BUS_SAFETY_TestparamsDSS[count],NULL,NULL);
    }
#endif
#endif
    if( SDL_APP_PASS== result)
    {
        DebugP_log("\n ECC BUS SAFETY TEST START : starting\r\n");
        for ( count = 0; sdlEccBusSafetyAppTestList[count].application != NULL; count++)
        {
            /* Get start time of test */
            testStartTime = ClockP_getTimeUsec();
            result = sdlEccBusSafetyAppTestList[count].application();
            /* Record test end time */
            testEndTime = ClockP_getTimeUsec();
            diffTime = testEndTime-testStartTime;
            sdlEccBusSafetyAppTestList[count].status = result;
            sdlEccBusSafetyAppTestList[count].test_time =diffTime;
        }
        result = SDL_APP_PASS;
        for ( count = 0; sdlEccBusSafetyAppTestList[count].application != NULL; count++)
        {
            if (sdlEccBusSafetyAppTestList[count].status != SDL_APP_PASS)
            {
                DebugP_log("\n Applications Name: %s  FAILED and Time taken for the Test is %d  micro secs \r\n", sdlEccBusSafetyAppTestList[count].name, (uint32_t)sdlEccBusSafetyAppTestList[count].test_time);
                result = SDL_APP_FAILED;
                break;
            }
            else
            {
                DebugP_log("\nApplications Name: %s  PASSED  and Time taken for the Test is %d  micro secs \r\n", sdlEccBusSafetyAppTestList[count].name ,(uint32_t)sdlEccBusSafetyAppTestList[count].test_time );
            }
        }
        if (result == SDL_APP_PASS)
        {
            DebugP_log("\n All tests have passed \r\n");
        }
        else
        {
            DebugP_log("\n Few/all tests Failed \r\n");
        }
    }
    else
    {
        DebugP_log("\n ESM init Fail \r\n");
    }
    ecc_bus_safety_edma_boardDeinit();
}

/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

void sdl_ecc_bus_safety_test_main(void *args)
{
    /* Declarations of variables */
    int32_t    result = SDL_APP_PASS;
    /* Init Dpl */
    result = SDL_TEST_dplInit();
    if (result != SDL_PASS)
    {
       DebugP_log("Error: DPL Init Failed\r\n");
    }

    DebugP_log("\n ECC BUS SAFETY  Application\r\n");

    ecc_bus_safety_testExecute();
}

/* Nothing past this point */
