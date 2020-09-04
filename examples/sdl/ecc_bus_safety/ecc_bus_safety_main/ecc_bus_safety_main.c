/* Copyright (c) 2022 Texas Instruments Incorporated
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
uint32_t            *srcBuffPtr, *dstBuffPtr;
EDMACCPaRAMEntry   edmaParam1, edmaParam2;

/* The source buffer used for transfer */
static uint32_t gEdmaTestSrcBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)))= {0x12345678U,0x1234567U};
/* The destination buffer used for transfer */
static uint32_t gEdmaTestDstBuff[EDMA_TEST_BUFFER_SIZE] __attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));


sdlEccBusSafetyApp_t  sdlEccBusSafetyAppTestList[] = {
#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)
    {SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_DED_Test_Polling,        "MDO_FIFO_DED_Test in Polling Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_RED_Test_Polling,        "MDO_FIFO_RED_Test in Polling Method",         SDL_APP_NOT_RUN },
#endif
#endif

#if defined (SOC_AM273X) ||  (SOC_AWR294X)
#if defined (SUBSYS_DSS)
    /* DSS MCRC */
    {SDL_ECC_BUS_SAFETY_DSS_MCRC_SEC_Test,                    "MCRC_SEC_Test in Interrupt Method",           SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MCRC_DED_Test,                    "MCRC_DED_Test in Interrupt Method",           SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MCRC_RED_Test,                    "MCRC_RED_Test in Interrupt Method",           SDL_APP_NOT_RUN },
    /* DSS L3 BANKA */
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_SEC_Test,                "DSS_L3_BANKA_SEC_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_DED_Test,                "DSS_L3_BANKA_DED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_RED_Test,                "DSS_L3_BANKA_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* DSS L3 BANKB */
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_SEC_Test,                "DSS_L3_BANKB_SEC_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_DED_Test,                "DSS_L3_BANKB_DED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_RED_Test,                "DSS_L3_BANKB_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* DSS L3 BANKC */
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_SEC_Test,                "DSS_L3_BANKC_SEC_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_DED_Test,                "DSS_L3_BANKC_DED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_RED_Test,                "DSS_L3_BANKC_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* DSS L3 BANKD */
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_SEC_Test,                "DSS_L3_BANKD_SEC_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_DED_Test,                "DSS_L3_BANKD_DED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_RED_Test,                "DSS_L3_BANKD_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* DSS HWA DMA 0 */
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_SEC_Test,                "DSS_HWA_DMA0_SEC_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_DED_Test,                "DSS_HWA_DMA0_DED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_RED_Test,                "DSS_HWA_DMA0_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* DSS HWA DMA 1 */
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_SEC_Test,                "DSS_HWA_DMA1_SEC_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_DED_Test,                "DSS_HWA_DMA1_DED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_RED_Test,                "DSS_HWA_DMA1_RED_Test in Interrupt Method",   SDL_APP_NOT_RUN },
    /* DSS MBOX */
    {SDL_ECC_BUS_SAFETY_DSS_MBOX_SEC_Test,                    "DSS_MBOX_SEC_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MBOX_DED_Test,                    "DSS_MBOX_DED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MBOX_RED_Test,                    "DSS_MBOX_RED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    /* DSS CBUFF FIFO */
    {SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_SEC_Test,              "CBUFF_FIFO_SEC_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_DED_Test,              "CBUFF_FIFO_DED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_RED_Test,              "CBUFF_FIFO_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC A0 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC A1 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC B0 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC B1 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC C0 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC C1 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC C2 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC C3 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC C4 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC C5 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR_RED_Test,              "TPTC_A0_WR_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC A0 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_SEC_Test,              "TPTC_A0_RD_SEC_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_DED_Test,              "TPTC_A0_RD_DED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_RED_Test,              "TPTC_A0_RD_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC A1 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_SEC_Test,              "TPTC_A1_RD_SEC_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_DED_Test,              "TPTC_A1_RD_DED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_RED_Test,              "TPTC_A1_RD_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC B0 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_SEC_Test,              "TPTC_B0_RD_SEC_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_DED_Test,              "TPTC_B0_RD_DED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_RED_Test,              "TPTC_B0_RD_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC B1 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_SEC_Test,              "TPTC_B1_RD_SEC_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_DED_Test,              "TPTC_B1_RD_DED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_RED_Test,              "TPTC_B1_RD_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC C0 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_SEC_Test,              "TPTC_C0_RD_SEC_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_DED_Test,              "TPTC_C0_RD_DED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_RED_Test,              "TPTC_C0_RD_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC C0 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_SEC_Test,              "TPTC_C1_RD_SEC_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_DED_Test,              "TPTC_C1_RD_DED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_RED_Test,              "TPTC_C1_RD_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC C1 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_SEC_Test,              "TPTC_C2_RD_SEC_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_DED_Test,              "TPTC_C2_RD_DED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_RED_Test,              "TPTC_C2_RD_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC C2 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_SEC_Test,              "TPTC_C3_RD_SEC_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_DED_Test,              "TPTC_C3_RD_DED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_RED_Test,              "TPTC_C3_RD_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC C4 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_SEC_Test,              "TPTC_C4_RD_SEC_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_DED_Test,              "TPTC_C4_RD_DED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_RED_Test,              "TPTC_C4_RD_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS TPTC C5 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_SEC_Test,              "TPTC_C5_RD_SEC_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_DED_Test,              "TPTC_C5_RD_DED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_RED_Test,              "TPTC_C5_RD_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* DSS PCR */
    {SDL_ECC_BUS_SAFETY_DSS_PCR_SEC_Test,                     "PCR_SEC_Test in Interrupt Method",            SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_PCR_DED_Test,                     "PCR_DED_Test in Interrupt Method",            SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_PCR_RED_Test,                     "PCR_RED_Test in Interrupt Method",            SDL_APP_NOT_RUN },
    /* DSS DSP SDMA */
    {SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_SEC_Test,                "DSP_SDMA_SEC_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_DED_Test,                "DSP_SDMA_DED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_RED_Test,                "DSP_SDMA_RED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    /* DSS DSP MDMA */
    {SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_RED_Test,                "DSP_MDMA_RED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
#endif
#endif

#if defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)
    /* RSS MBOX */
    {SDL_ECC_BUS_SAFETY_RSS_MBOX_SEC_Test,                    "RSS_MBOX_SEC_Test in Interrupt Method",           SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_RSS_MBOX_DED_Test,                    "RSS_MBOX_DED_Test in Interrupt Method",           SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_RSS_MBOX_RED_Test,                    "RSS_MBOX_RED_Test in Interrupt Method",           SDL_APP_NOT_RUN },
    /* RSS ADCBUF RD */
    {SDL_ECC_BUS_SAFETY_RSS_ADCBUF_RD_RED_Test,               "RSS_ADCBUF_RD_RED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    /* RSS ADCBUF WR */
    {SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR_SEC_Test,               "RSS_ADCBUF_WR_SEC_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR_DED_Test,               "RSS_ADCBUF_WR_DED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR_RED_Test,               "RSS_ADCBUF_WR_RED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
#endif
#endif

#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)
    /* DSS MD0 FIFO */
    {SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_SEC_Test,                "MDO_FIFO_SEC_Test in Interrupt Method",           SDL_APP_NOT_RUN },
#endif
#endif
#if defined SUBSYS_MSS
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_RED_Test,              "CR5A_AHB_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_RED_Test,              "CR5B_AHB_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    /* MSS TPTC A0 WR */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test,            "TPTC_A0_WR_RED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    /* MSS TPTC A1 WR */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test,            "TPTC_A1_WR_RED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
     /* MSS TPTC B0 WR */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR_RED_Test,            "TPTC_B0_WR_RED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
       /* MSS TPTC A0 RD */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_Test,            "TPTC_A0_RD_SEC_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_Test,            "TPTC_A0_RD_DED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_Test,            "TPTC_A0_RD_RED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
     /* MSS TPTC A1 RD */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_Test,            "TPTC_A1_RD_SEC_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_Test,            "TPTC_A1_RD_DED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_Test,            "TPTC_A1_RD_RED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
     /* MSS TPTC B0 RD */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_SEC_Test,            "TPTC_B0_RD_SEC_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_DED_Test,            "TPTC_B0_RD_DED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_RED_Test,            "TPTC_B0_RD_RED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
     /* MSS MBOX */
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_SEC_Test,                   "MSS_MBOX_SEC_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_Test,                  "MSS_MBOX_DED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_RED_Test,                  "MSS_MBOX_RED_Test in Interrupt Method",        SDL_APP_NOT_RUN },
     /* MSS CR5A_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_Test,           "MSS_CR5A_AXI_WR_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    /* MSS CR5B_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_Test,           "MSS_CR5B_AXI_WR_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },

     /* MSS CR5A_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_RED_Test,           "MSS_CR5A_AXI_RD_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },

    /* MSS CR5B_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_RED_Test,           "MSS_CR5B_AXI_RD_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },

   /* MSS CR5A_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_Test,            "MSS_CR5A_AXI_S_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },

   /* MSS CR5B_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_Test,            "MSS_CR5B_AXI_S_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },

#endif

#if defined (SOC_AM263X)
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_RED_Test,              "CR5A_AHB_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_RED_Test,              "CR5B_AHB_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB_RED_Test,              "CR5C_AHB_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB_RED_Test,              "CR5D_AHB_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
     /* MSS TPTC A0 WR */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test,            "TPTC_A0_WR_RED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    /* MSS TPTC A1 WR */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test,            "TPTC_A1_WR_RED_Test in Interrupt Method",       SDL_APP_NOT_RUN },

      /* MSS TPTC A0 RD */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_Test,            "TPTC_A0_RD_SEC_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_Test,            "TPTC_A0_RD_DED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_Test,            "TPTC_A0_RD_RED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    /* MSS TPTC A1 RD */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_Test,            "TPTC_A1_RD_SEC_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_Test,            "TPTC_A1_RD_DED_Test in Interrupt Method",       SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_Test,            "TPTC_A1_RD_RED_Test in Interrupt Method",       SDL_APP_NOT_RUN },

     /* MSS CR5A_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_Test,           "MSS_CR5A_AXI_WR_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    /* MSS CR5B_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_Test,           "MSS_CR5B_AXI_WR_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },

    /* MSS CR5C_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_RED_Test,           "MSS_CR5C_AXI_WR_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    /* MSS CR5D_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_RED_Test,           "MSS_CR5D_AXI_WR_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },

     /* MSS CR5A_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_SEC_Test,           "MSS_CR5A_AXI_RD_SEC_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_RED_Test,           "MSS_CR5A_AXI_RD_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },

     /* MSS CR5B_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_SEC_Test,           "MSS_CR5B_AXI_RD_SEC_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_RED_Test,           "MSS_CR5B_AXI_RD_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },

     /* MSS CR5C_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_SEC_Test,           "MSS_CR5C_AXI_RD_SEC_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_RED_Test,           "MSS_CR5C_AXI_RD_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },

     /* MSS CR5D_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_SEC_Test,           "MSS_CR5D_AXI_RD_SEC_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_RED_Test,           "MSS_CR5D_AXI_RD_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },

      /* MSS CR5A_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_SEC_Test,            "MSS_CR5A_AXI_S_SEC_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_Test,            "MSS_CR5A_AXI_S_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },

      /* MSS CR5B_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_SEC_Test,            "MSS_CR5B_AXI_S_SEC_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_Test,            "MSS_CR5B_AXI_S_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },

     /* MSS CR5C_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_SEC_Test,            "MSS_CR5C_AXI_S_SEC_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_RED_Test,            "MSS_CR5C_AXI_S_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },

     /* MSS CR5D_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_SEC_Test,            "MSS_CR5D_AXI_S_SEC_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_RED_Test,            "MSS_CR5D_AXI_S_RED_Test in INTERRUPT  Method",         SDL_APP_NOT_RUN },


#endif
#endif

    {NULL,                                                  "TERMINATING CONDITION",                         SDL_APP_NOT_RUN }

};

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/
/*
 *   In this test, EDMA transfer in A synchronized transfer is verified
 *   in polling mode.
 */
void test_edmaATransfer(uint32_t busSftyNode, uint32_t dmaCh, uint32_t tcc,uint32_t param, uint32_t queNum,\
                                                                       uint32_t edmaNum )
{
    uint32_t            loopCnt = 0;
    uint32_t            baseAddr, regionId;
    int32_t             testStatus = SystemP_SUCCESS;
    baseAddr = EDMA_getBaseAddr(gEdmaHandle[edmaNum]);
    DebugP_assert(baseAddr != 0);

    regionId = EDMA_getRegionId(gEdmaHandle[edmaNum]);
    DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

    testStatus = EDMA_allocDmaChannel(gEdmaHandle[edmaNum], &dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    testStatus = EDMA_allocTcc(gEdmaHandle[edmaNum], &tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    testStatus = EDMA_allocParam(gEdmaHandle[edmaNum], &param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    /*
     * Initialize the source address with a pattern
     * initialize dst address with zero/another pattern (optional)
     */
    srcBuffPtr = (uint32_t *) gEdmaTestSrcBuff;

#if defined (SUBSYS_MSS)
    dstBuffPtr = (uint32_t *) gEdmaTestDstBuff;
#endif
#if defined (SUBSYS_DSS)
   if( busSftyNode == SDL_ECC_BUS_SAFETY_DSS_PCR)
   {
       dstBuffPtr = (uint32_t *) (SDL_DSS_CTRL_U_BASE);
   }
   else
   {
       dstBuffPtr = (uint32_t *) gEdmaTestDstBuff;
   }
#endif
    /* Request channel */
    EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, tcc, param, queNum);

    /* Disable the interrupt for the channel to transfer in polled mode */
    EDMA_disableEvtIntrRegion(baseAddr, regionId, dmaCh);

    /* Program Param Set */
    EDMA_ccPaRAMEntry_init(&edmaParam1);
    edmaParam1.srcAddr       = (uint32_t) SOC_virtToPhy(srcBuffPtr);
    edmaParam1.destAddr      = (uint32_t) SOC_virtToPhy(dstBuffPtr);
    edmaParam1.aCnt          = (uint16_t) EDMA_TEST_A_COUNT;
    edmaParam1.bCnt          = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam1.cCnt          = (uint16_t) EDMA_TEST_C_COUNT;
    edmaParam1.bCntReload    = (uint16_t) EDMA_TEST_B_COUNT;
    edmaParam1.srcBIdx       = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam1.destBIdx      = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam1.srcCIdx       = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam1.destCIdx      = (int16_t) EDMA_TEST_A_COUNT;
    edmaParam1.linkAddr      = 0xFFFFU;
    edmaParam1.opt          |=
        (EDMA_OPT_TCINTEN_MASK | EDMA_OPT_ITCINTEN_MASK |
         ((((uint32_t)tcc) << EDMA_OPT_TCC_SHIFT) & EDMA_OPT_TCC_MASK));
    EDMA_setPaRAM(baseAddr, param, &edmaParam1);

    /*
     * Transfer is done in A sync mode
     * Number of triggers required are B_COUNT * C_COUNT
     */
    for(loopCnt = 0; loopCnt < (EDMA_TEST_B_COUNT * EDMA_TEST_C_COUNT); loopCnt++)
    {
        EDMA_enableTransferRegion(baseAddr, regionId, dmaCh,
             EDMA_TRIG_MODE_MANUAL);
        EDMA_clrIntrRegion(baseAddr, regionId, tcc);
    }

    /* Free channel */
    EDMA_freeChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
         dmaCh, EDMA_TRIG_MODE_MANUAL, tcc, queNum);

    /* Free the EDMA resources managed by driver. */
    testStatus = EDMA_freeDmaChannel(gEdmaHandle[edmaNum], &dmaCh);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeTcc(gEdmaHandle[edmaNum], &tcc);
    DebugP_assert(testStatus == SystemP_SUCCESS);
    testStatus = EDMA_freeParam(gEdmaHandle[edmaNum], &param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

}

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
    int32_t    i;
    uint64_t testStartTime;
    uint64_t testEndTime;
    uint64_t diffTime;
    ecc_bus_safety_edma_boardInit();
    #if defined (SOC_AWR294X) || (SOC_AM273X)
#if defined (SUBSYS_MSS)
    for( i =0; i<16U; i++)
    {
	    result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_BUS_SAFETY_TestparamsMSS[i],NULL,NULL);
    }
#endif
#endif
#if defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)
    for( i =0; i<37U; i++)
    {
	    result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_BUS_SAFETY_TestparamsDSS[i],NULL,NULL);
    }
#endif
#endif
#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)
    for( i =0; i<33U; i++)
    {
	    result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_BUS_SAFETY_TestparamsDSS[i],NULL,NULL);
    }
#endif
#endif
    if( SDL_APP_PASS== result)
    {
        DebugP_log("\n ECC BUS SAFETY TEST START : starting\r\n");
        for ( i = 0; sdlEccBusSafetyAppTestList[i].application != NULL; i++)
        {
            /* Get start time of test */
            testStartTime = ClockP_getTimeUsec();
            result = sdlEccBusSafetyAppTestList[i].application();
            /* Record test end time */
            testEndTime = ClockP_getTimeUsec();
            diffTime = testEndTime-testStartTime;
            sdlEccBusSafetyAppTestList[i].status = result;
            sdlEccBusSafetyAppTestList[i].test_time =diffTime;
        }
        result = SDL_APP_PASS;
        for ( i = 0; sdlEccBusSafetyAppTestList[i].application != NULL; i++)
        {
            if (sdlEccBusSafetyAppTestList[i].status != SDL_APP_PASS)
            {
                DebugP_log("\n Applications Name: %s  FAILED and Time taken for the Test is %d  micro secs \r\n", sdlEccBusSafetyAppTestList[i].name, (uint32_t)sdlEccBusSafetyAppTestList[i].test_time);
                result = SDL_APP_FAILED;
                break;
            }
            else
            {
                DebugP_log("\nApplications Name: %s  PASSED  and Time taken for the Test is %d  micro secs \r\n", sdlEccBusSafetyAppTestList[i].name ,(uint32_t)sdlEccBusSafetyAppTestList[i].test_time );
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
