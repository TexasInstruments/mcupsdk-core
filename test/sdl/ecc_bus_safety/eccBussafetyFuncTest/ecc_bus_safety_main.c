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
 *  DATA, OR PROFITS; OR BUSINESS InterruptION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

 /**
 *  \file     ecc_bus_safety_main.c
 *
 *  \brief    This file contains Ecc Bus Safety for all the nodes in MSS and DSS.
 *
 *  \details  ECC_BUS_SAFETY APP
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "ecc_bus_safety_main.h"
#include <dpl_interface.h>
#include <kernel/dpl/CacheP.h>

#ifdef UNITY_INCLUDE_CONFIG_H
#include <ti/build/unit-test/Unity/src/unity.h>
#include <ti/build/unit-test/config/unity_config.h>
#endif


/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/
/* None */

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
/* None */

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
/* Unity functions */
void test_sdl_ecc_bus_safety_baremetal_test_app_runner(void);
void test_sdl_ecc_bus_safety_baremetal_test_app (void);

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/

sdlECCBusSftyTest_t  sdlEccBusSftyTestList[] = {
#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)
    /* DSS MD0 FIFO */
    {SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_DED_Test_Polling,               "MDO_FIFO_DED_Test in Polling Method",                            SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_RED_Test_Polling,               "MDO_FIFO_RED_Test in Polling Method",                            SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_DED_End_Addr_Test_Polling,      "MDO_DED_Test for end address in Polling Method",                 SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_RED_FI_Main_Test_Polling,       "MDO_FIFO_RED_Test with fi mainin Polling Method",                SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_RED_FI_Safe_Test_Polling,       "MDO_FIFO_RED_Test with fi safe Polling Method",                  SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_RED_FI_Global_Main_Test_Polling,"MDO_FIFO_RED_Test with fi global safe Polling Method",           SDL_APP_TEST_NOT_RUN },
#endif
#endif
#if defined (SOC_AM273X) ||  (SOC_AWR294X)
#if defined (SUBSYS_DSS)
    /* DSS MCRC */
    {SDL_ECC_BUS_SAFETY_DSS_MCRC_SEC_Test,                           "DSS_MCRC_SEC_Test in Interrupt Method",                          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MCRC_DED_Test,                           "DSS_MCRC_DED_Test in Interrupt Method",                          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MCRC_RED_Test,                           "DSS_MCRC_RED_Test in Interrupt Method",                          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MCRC_SEC_End_Addr_Test,                  "DSS_MCRC_SEC_Test for end address in Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MCRC_DED_End_Addr_Test,                  "DSS_MCRC_DED_Test for end address in Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MCRC_RED_FI_Main_Test,                   "DSS_MCRC_RED_Test with fi mainin Interrupt Method",              SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MCRC_RED_FI_Safe_Test,                   "DSS_RED_Test with fi safe Interrupt Method",                     SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MCRC_RED_FI_Global_Main_Test,            "DSS_RED_Test with fi global safe Interrupt Method",              SDL_APP_TEST_NOT_RUN },
    /* DSS L3 BANKA */
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_SEC_Test,                       "DSS_L3_BANKA_SEC_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_DED_Test,                       "DSS_L3_BANKA_DED_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_RED_Test,                       "DSS_L3_BANKA_RED_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_SEC_End_Addr_Test,              "DSS_L3_BANKA_SEC_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_DED_End_Addr_Test,              "DSS_L3_BANKA_DED_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_RED_FI_Main_Test,               "DSS_L3_BANKA_RED_Test with fi mainin Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_RED_FI_Safe_Test,               "DSS_L3_BANKA_RED_Test with fi safe Interrupt Method",            SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_RED_FI_Global_Main_Test,        "DSS_L3_BANKA_RED_Test with fi global safe Interrupt Method",     SDL_APP_TEST_NOT_RUN },
    /* DSS L3 BANKB */
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_SEC_Test,                       "DSS_L3_BANKB_SEC_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_DED_Test,                       "DSS_L3_BANKB_DED_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_RED_Test,                       "DSS_L3_BANKB_RED_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_SEC_End_Addr_Test,              "DSS_L3_BANKB_SEC_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_DED_End_Addr_Test,              "DSS_L3_BANKB_DED_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_RED_FI_Main_Test,               "DSS_L3_BANKB_RED_Test with fi mainin Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_RED_FI_Safe_Test,               "DSS_L3_BANKB_RED_Test with fi safe Interrupt Method",            SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_RED_FI_Global_Main_Test,        "DSS_L3_BANKB_RED_Test with fi global safe Interrupt Method",     SDL_APP_TEST_NOT_RUN },
    /* DSS L3 BANKC */
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_SEC_Test,                       "DSS_L3_BANKC_SEC_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_DED_Test,                       "DSS_L3_BANKC_DED_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_RED_Test,                       "DSS_L3_BANKC_RED_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_SEC_End_Addr_Test,              "DSS_L3_BANKC_SEC_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_DED_End_Addr_Test,              "DSS_L3_BANKC_DED_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_RED_FI_Main_Test,               "DSS_L3_BANKC_RED_Test with fi mainin Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_RED_FI_Safe_Test,               "DSS_L3_BANKC_RED_Test with fi safe Interrupt Method",            SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_RED_FI_Global_Main_Test,        "DSS_L3_BANKC_RED_Test with fi global safe Interrupt Method",     SDL_APP_TEST_NOT_RUN },
#if defined  (SOC_AM273X)
    /* DSS L3 BANKD */
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_SEC_Test,                       "DSS_L3_BANKD_SEC_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_DED_Test,                       "DSS_L3_BANKD_DED_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_RED_Test,                       "DSS_L3_BANKD_RED_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_SEC_End_Addr_Test,              "DSS_L3_BANKD_SEC_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_DED_End_Addr_Test,              "DSS_L3_BANKD_DED_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_RED_FI_Main_Test,               "DSS_L3_BANKD_RED_Test with fi mainin Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_RED_FI_Safe_Test,               "DSS_L3_BANKD_RED_Test with fi safe Interrupt Method",            SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_RED_FI_Global_Main_Test,        "DSS_L3_BANKD_RED_Test with fi global safe Interrupt Method",     SDL_APP_TEST_NOT_RUN },
#endif
    /* DSS HWA DMA 0 */
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_SEC_Test,                       "DSS_HWA_DMA0_SEC_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_DED_Test,                       "DSS_HWA_DMA0_DED_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_RED_Test,                       "DSS_HWA_DMA0_RED_Test in Interrupt Method",                       SDL_APP_TEST_NOT_RUN},
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_SEC_End_Addr_Test,              "DSS_HWA_DMA0_SEC_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_DED_End_Addr_Test,              "DSS_HWA_DMA0_DED_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_RED_FI_Main_Test,               "DSS_HWA_DMA0_RED_Test with fi mainin Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_RED_FI_Safe_Test,               "DSS_HWA_DMA0_RED_Test with fi safe Interrupt Method",            SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_RED_FI_Global_Main_Test,        "DSS_HWA_DMA0_RED_Test with fi global safe Interrupt Method",     SDL_APP_TEST_NOT_RUN },
    /* DSS HWA DMA 1 */
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_SEC_Test,                       "DSS_HWA_DMA1_SEC_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_DED_Test,                       "DSS_HWA_DMA1_DED_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_RED_Test,                       "DSS_HWA_DMA1_RED_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_SEC_End_Addr_Test,              "DSS_HWA_DMA1_SEC_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_DED_End_Addr_Test,              "DSS_HWA_DMA1_DED_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_RED_FI_Main_Test,               "DSS_HWA_DMA1_RED_Test with fi mainin Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_RED_FI_Safe_Test,               "DSS_HWA_DMA1_RED_Test with fi safe Interrupt Method",            SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_RED_FI_Global_Main_Test,        "DSS_HWA_DMA1_RED_Test with fi global safe Interrupt Method",     SDL_APP_TEST_NOT_RUN },
    /* DSS MBOX */
    {SDL_ECC_BUS_SAFETY_DSS_MBOX_SEC_Test,                           "DSS_MBOX_SEC_Test in Interrupt Method",                          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MBOX_DED_Test,                           "DSS_MBOX_DED_Test in Interrupt Method",                          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MBOX_RED_Test,                           "DSS_MBOX_RED_Test in Interrupt Method",                          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MBOX_SEC_End_Addr_Test,                  "DSS_MBOX_SEC_Test for end address in Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MBOX_DED_End_Addr_Test,                  "DSS_MBOX_DED_Test for end address in Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MBOX_RED_FI_Main_Test,                   "DSS_MBOX_RED_Test with fi mainin Interrupt Method",              SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MBOX_RED_FI_Safe_Test,                   "DSS_MBOX_RED_Test with fi safe Interrupt Method",                SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MBOX_RED_FI_Global_Main_Test,            "DSS_MBOX_RED_Test with fi global safe Interrupt Method",         SDL_APP_TEST_NOT_RUN },
    /* DSS CBUFF FIFO */
    {SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_SEC_Test,                     "DSS_CBUFF_FIFO_SEC_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_DED_Test,                     "DSS_CBUFF_FIFO_DED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_RED_Test,                     "DSS_CBUFF_FIFO_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_SEC_End_Addr_Test,            "DSS_CBUFF_FIFO_SEC_Test for end address in Interrupt Method",    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_DED_End_Addr_Test,            "DSS_CBUFF_FIFO_DED_Test for end address in Interrupt Method",    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_RED_FI_Main_Test,             "DSS_CBUFF_FIFO_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_RED_FI_Safe_Test,             "DSS_CBUFF_FIFO_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_RED_FI_Global_Main_Test,      "DSS_CBUFF_FIFO_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC A0 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR_RED_Test,                     "DSS_TPTC_A0_WR_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR_RED_FI_Main_Test,             "DSS_TPTC_A0_WR_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR_RED_FI_Safe_Test,             "DSS_TPTC_A0_WR_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR_RED_FI_Global_Main_Test,      "DSS_TPTC_A0_WR_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC A1 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR_RED_Test,                     "DSS_TPTC_A0_WR_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR_RED_FI_Main_Test,             "DSS_TPTC_A1_WR_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR_RED_FI_Safe_Test,             "DSS_TPTC_A1_WR_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR_RED_FI_Global_Main_Test,      "DSS_TPTC_A1_WR_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC B0 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR_RED_Test,                     "DSS_TPTC_A0_WR_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR_RED_FI_Main_Test,             "DSS_TPTC_B0_WR_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR_RED_FI_Safe_Test,             "DSS_TPTC_B0_WR_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR_RED_FI_Global_Main_Test,      "DSS_TPTC_B0_WR_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC B1 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR_RED_Test,                     "DSS_TPTC_A0_WR_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR_RED_FI_Main_Test,             "DSS_TPTC_B1_WR_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR_RED_FI_Safe_Test,             "DSS_TPTC_B1_WR_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR_RED_FI_Global_Main_Test,      "DSS_TPTC_B1_WR_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC C0 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR_RED_Test,                     "DSS_TPTC_A0_WR_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR_RED_FI_Main_Test,             "DSS_TPTC_C0_WR_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR_RED_FI_Safe_Test,             "DSS_TPTC_C0_WR_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR_RED_FI_Global_Main_Test,      "DSS_TPTC_C0_WR_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC C1 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR_RED_Test,                     "DSS_TPTC_A0_WR_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR_RED_FI_Main_Test,             "DSS_TPTC_C1_WR_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR_RED_FI_Safe_Test,             "DSS_TPTC_C1_WR_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR_RED_FI_Global_Main_Test,      "DSS_TPTC_C1_WR_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC C2 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR_RED_Test,                     "DSS_TPTC_A0_WR_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR_RED_FI_Main_Test,             "DSS_TPTC_C2_WR_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR_RED_FI_Safe_Test,             "DSS_TPTC_C2_WR_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR_RED_FI_Global_Main_Test,      "DSS_TPTC_C2_WR_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC C3 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR_RED_Test,                     "DSS_TPTC_A0_WR_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR_RED_FI_Main_Test,             "DSS_TPTC_C3_WR_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR_RED_FI_Safe_Test,             "DSS_TPTC_C3_WR_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR_RED_FI_Global_Main_Test,      "DSS_TPTC_C3_WR_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC C4 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR_RED_Test,                     "DSS_TPTC_A0_WR_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR_RED_FI_Main_Test,             "DSS_TPTC_C4_WR_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR_RED_FI_Safe_Test,             "DSS_TPTC_C4_WR_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR_RED_FI_Global_Main_Test,      "DSS_TPTC_C4_WR_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC C5 WR */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR_RED_Test,                     "DSS_TPTC_A0_WR_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR_RED_FI_Main_Test,             "DSS_TPTC_C5_WR_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR_RED_FI_Safe_Test,             "DSS_TPTC_C5_WR_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR_RED_FI_Global_Main_Test,      "DSS_TPTC_C5_WR_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC A0 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_SEC_Test,                     "DSS_TPTC_A0_RD_SEC_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_DED_Test,                     "DSS_TPTC_A0_RD_DED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_RED_Test,                     "DSS_TPTC_A0_RD_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_RED_FI_Main_Test,             "DSS_TPTC_A0_RD_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_RED_FI_Safe_Test,             "DSS_TPTC_A0_RD_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_RED_FI_Global_Main_Test,      "DSS_TPTC_A0_RD_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC A1 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_SEC_Test,                     "DSS_TPTC_A1_RD_SEC_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_DED_Test,                     "DSS_TPTC_A1_RD_DED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_RED_Test,                     "DSS_TPTC_A1_RD_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_RED_FI_Main_Test,             "DSS_TPTC_A1_RD_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_RED_FI_Safe_Test,             "DSS_TPTC_A1_RD_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_RED_FI_Global_Main_Test,      "DSS_TPTC_A1_RD_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC B0 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_SEC_Test,                     "DSS_TPTC_B0_RD_SEC_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_DED_Test,                     "DSS_TPTC_B0_RD_DED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_RED_Test,                     "DSS_TPTC_B0_RD_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_RED_FI_Main_Test,             "DSS_TPTC_B0_RD_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_RED_FI_Safe_Test,             "DSS_TPTC_B0_RD_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_RED_FI_Global_Main_Test,      "DSS_TPTC_B0_RD_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC B1 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_SEC_Test,                     "DSS_TPTC_B1_RD_SEC_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_DED_Test,                     "DSS_TPTC_B1_RD_DED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_RED_Test,                     "DSS_TPTC_B1_RD_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_RED_FI_Main_Test,             "DSS_TPTC_B1_RD_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_RED_FI_Safe_Test,             "DSS_TPTC_B1_RD_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_RED_FI_Global_Main_Test,      "DSS_TPTC_B1_RD_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC C0 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_SEC_Test,                     "DSS_TPTC_C0_RD_SEC_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_DED_Test,                     "DSS_TPTC_C0_RD_DED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_RED_Test,                     "DSS_TPTC_C0_RD_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_RED_FI_Main_Test,             "DSS_TPTC_C0_RD_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_RED_FI_Safe_Test,             "DSS_TPTC_C0_RD_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_RED_FI_Global_Main_Test,      "DSS_TPTC_C0_RD_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC C0 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_SEC_Test,                     "DSS_TPTC_C1_RD_SEC_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_DED_Test,                     "DSS_TPTC_C1_RD_DED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_RED_Test,                     "DSS_TPTC_C1_RD_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_RED_FI_Main_Test,             "DSS_TPTC_C1_RD_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_RED_FI_Safe_Test,             "DSS_TPTC_C1_RD_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_RED_FI_Global_Main_Test,      "DSS_TPTC_C1_RD_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC C1 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_SEC_Test,                     "DSS_TPTC_C2_RD_SEC_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_DED_Test,                     "DSS_TPTC_C2_RD_DED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_RED_Test,                     "DSS_TPTC_C2_RD_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_RED_FI_Main_Test,             "DSS_TPTC_C2_RD_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_RED_FI_Safe_Test,             "DSS_TPTC_C2_RD_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_RED_FI_Global_Main_Test,      "DSS_TPTC_C2_RD_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC C2 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_SEC_Test,                     "DSS_TPTC_C3_RD_SEC_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_DED_Test,                     "DSS_TPTC_C3_RD_DED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_RED_Test,                     "DSS_TPTC_C3_RD_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_RED_FI_Main_Test,             "DSS_TPTC_C3_RD_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_RED_FI_Safe_Test,             "DSS_TPTC_C3_RD_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_RED_FI_Global_Main_Test,      "DSS_TPTC_C3_RD_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC C4 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_SEC_Test,                     "DSS_TPTC_C4_RD_SEC_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_DED_Test,                     "DSS_TPTC_C4_RD_DED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_RED_Test,                     "DSS_TPTC_C4_RD_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_RED_FI_Main_Test,             "DSS_TPTC_C4_RD_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_RED_FI_Safe_Test,             "DSS_TPTC_C4_RD_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_RED_FI_Global_Main_Test,      "DSS_TPTC_C4_RD_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS TPTC C5 RD */
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_SEC_Test,                     "DSS_TPTC_C5_RD_SEC_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_DED_Test,                     "DSS_TPTC_C5_RD_DED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_RED_Test,                     "DSS_TPTC_C5_RD_RED_Test in Interrupt Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_RED_FI_Main_Test,             "DSS_TPTC_C5_RD_RED_Test with fi mainin Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_RED_FI_Safe_Test,             "DSS_TPTC_C5_RD_RED_Test with fi safe Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_RED_FI_Global_Main_Test,      "DSS_TPTC_C5_RD_RED_Test with fi global safe Interrupt Method",   SDL_APP_TEST_NOT_RUN },
    /* DSS PCR */
    {SDL_ECC_BUS_SAFETY_DSS_PCR_SEC_Test,                            "DSS_PCR_SEC_Test in Interrupt Method",                           SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_PCR_DED_Test,                            "DSS_PCR_DED_Test in Interrupt Method",                           SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_PCR_RED_Test,                            "DSS_PCR_RED_Test in Interrupt Method",                           SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_PCR_RED_FI_Main_Test,                    "DSS_PCR_RD_RED_Test with fi mainin Interrupt Method",            SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_PCR_RED_FI_Safe_Test,                    "DSS_PCR_RD_RED_Test with fi safe Interrupt Method",              SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_PCR_RED_FI_Global_Main_Test,             "DSS_PCR_RD_RED_Test with fi global safe Interrupt Method",       SDL_APP_TEST_NOT_RUN },
    /* DSS DSP SDMA */
    {SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_SEC_Test,                       "DSP_SDMA_SEC_Test in Interrupt Method",                          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_DED_Test,                       "DSP_SDMA_DED_Test in Interrupt Method",                          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_RED_Test,                       "DSP_SDMA_RED_Test in Interrupt Method",                          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_SEC_End_Addr_Test,              "DSS_DSP_SDMA_SEC_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_DED_End_Addr_Test,              "DSS_DSP_SDMA_DED_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_RED_FI_Main_Test,               "DSS_DSP_SDMA_RED_Test with fi mainin Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_RED_FI_Safe_Test,               "DSS_DSP_SDMA_RED_Test with fi safe Interrupt Method",            SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_RED_FI_Global_Main_Test,        "DSS_DSP_SDMA_RED_Test with fi global safe Interrupt Method",     SDL_APP_TEST_NOT_RUN },
    /* DSS DSP MDMA */
    {SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_RED_Test,                       "DSP_MDMA_RED_Test in Interrupt Method",                          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_RED_FI_Main_Test,               "DSS_DSP_MDMA_RED_Test with fi mainin Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_RED_FI_Safe_Test,               "DSS_DSP_MDMA_RED_Test with fi safe Interrupt Method",            SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_RED_FI_Global_Main_Test,        "DSS_DSP_MDMA_RED_Test with fi global safe Interrupt Method",     SDL_APP_TEST_NOT_RUN },
#endif
#endif

#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)
    /* DSS MD0 FIFO */
    {SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_SEC_Test,                       "DSS_MDO_FIFO_SEC_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_SEC_End_Addr_Test,              "DSS_MDO_FIFO_SEC_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
#endif
#endif

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
#if defined (SUBSYS_MSS)
     /* MSS_CR5A_AHB */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_RED_Test,                       "MSS_CR5A_AHB_RED_Test in Interrupt Method",                     SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_RED_FI_Main_Test,               "MSS_CR5A_AHB_RED_FI_Main_Test in Interrupt Method",             SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_RED_FI_Safe_Test,               "MSS_CR5A_AHB_RED_FI_Safe_Test in Interrupt Method",             SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_RED_FI_Global_Main_Test,        "MSS_CR5A_AHB_RED_FI_Global_Main_Test in Interrupt Method",      SDL_APP_TEST_NOT_RUN },

     /* MSS_CR5B_AHB */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_RED_Test,                       "MSS_CR5B_AHB_RED_Test in Interrupt Method",                     SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_RED_FI_Main_Test,               "MSS_CR5B_AHB_RED_FI_Main_Test in Interrupt Method",             SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_RED_FI_Safe_Test,               "MSS_CR5B_AHB_RED_FI_Safe_Test in Interrupt Method",             SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_RED_FI_Global_Main_Test,        "MSS_CR5B_AHB_RED_FI_Global_Main_Test in Interrupt Method",      SDL_APP_TEST_NOT_RUN },

     /* MSS_TPTC_A0_WR */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test,                     "MSS_TPTC_A0_WR_RED_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_FI_Main_Test,             "MSS_TPTC_A0_WR_RED_FI_Main_Test in Interrupt Method",           SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_FI_Safe_Test,             "MSS_TPTC_A0_WR_RED_FI_Safe_Test in Interrupt Method",           SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_FI_Global_Main_Test,      "MSS_TPTC_A0_WR_RED_FI_Global_Main_Test in Interrupt Method",    SDL_APP_TEST_NOT_RUN },

     /* MSS_TPTC_A1_WR */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test,                     "MSS_TPTC_A1_WR_RED_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_FI_Main_Test,             "MSS_TPTC_A1_WR_RED_FI_Main_Test in Interrupt Method",           SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_FI_Safe_Test,             "MSS_TPTC_A1_WR_RED_FI_Safe_Test in Interrupt Method",           SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_FI_Global_Main_Test,      "MSS_TPTC_A1_WR_RED_FI_Global_Main_Test in Interrupt Method",    SDL_APP_TEST_NOT_RUN },

     /* MSS_TPTC_B0_WR */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR_RED_Test,                     "MSS_TPTC_B0_WR_RED_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR_RED_FI_Main_Test,             "MSS_TPTC_B0_WR_RED_FI_Main_Test in Interrupt Method",           SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR_RED_FI_Safe_Test,             "MSS_TPTC_B0_WR_RED_FI_Safe_Test in Interrupt Method",           SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR_RED_FI_Global_Main_Test,      "MSS_TPTC_B0_WR_RED_FI_Global_Main_Test in Interrupt Method",    SDL_APP_TEST_NOT_RUN },

     /* MSS TPTC A0 RD */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_Test,                     "MSS_TPTC_A0_RD_SEC_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_Test,                     "MSS_TPTC_A0_RD_DED_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_Test,                     "MSS_TPTC_A0_RD_RED_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_FI_Main_Test,             "MSS_TPTC_A0_RD_RED_FI_Main_Test in Interrupt Method",           SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_FI_Safe_Test,             "MSS_TPTC_A0_RD_RED_FI_Safe_Test in Interrupt Method",           SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_FI_Global_Main_Test,      "MSS_TPTC_A0_RD_RED_FI_Global_Main_Test in Interrupt Method",    SDL_APP_TEST_NOT_RUN },

     /* MSS TPTC A1 RD */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_Test,                     "MSS_TPTC_A1_RD_SEC_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_Test,                     "MSS_TPTC_A1_RD_DED_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_Test,                     "MSS_TPTC_A1_RD_RED_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_FI_Main_Test,             "MSS_TPTC_A1_RD_RED_FI_Main_Test in Interrupt Method",           SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_FI_Safe_Test,             "MSS_TPTC_A1_RD_RED_FI_Safe_Test in Interrupt Method",           SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_FI_Global_Main_Test,      "MSS_TPTC_A1_RD_RED_FI_Global_Main_Test in Interrupt Method",    SDL_APP_TEST_NOT_RUN },

     /* MSS TPTC B0 RD */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_SEC_Test,                     "MSS_TPTC_B0_RD_SEC_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_DED_Test,                     "MSS_TPTC_B0_RD_DED_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_RED_Test,                     "MSS_TPTC_B0_RD_RED_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_RED_FI_Main_Test,             "MSS_TPTC_B0_RD_RED_FI_Main_Test in Interrupt Method",           SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_RED_FI_Safe_Test,             "MSS_TPTC_B0_RD_RED_FI_Safe_Test in Interrupt Method",           SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_RED_FI_Global_Main_Test,      "MSS_TPTC_B0_RD_RED_FI_Global_Main_Test in Interrupt Method",    SDL_APP_TEST_NOT_RUN },

     /* MSS MBOX */
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_SEC_Test,                           "MSS_MBOX_SEC_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_Test,                           "MSS_MBOX_DED_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_SEC_End_Addr_Test,                  "MSS_MBOX_SEC_End_Addr_Test in Interrupt Method",                SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_End_Addr_Test,                  "MSS_MBOX_DED_End_Addr_Test in Interrupt Method",                SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_RED_Test,                           "MSS_MBOX_RED_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_RED_FI_Main_Test,                   "MSS_MBOX_RED_FI_Main_Test in Interrupt Method",                 SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_RED_FI_Safe_Test,                   "MSS_MBOX_RED_FI_Safe_Test in Interrupt Method",                 SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_RED_FI_Global_Main_Test,            "MSS_MBOX_RED_FI_Global_Main_Test in Interrupt Method",          SDL_APP_TEST_NOT_RUN },

    /* MSS MCRC */
   {SDL_ECC_BUS_SAFETY_MSS_MCRC_SEC_Test,                            "MSS_MCRC_SEC_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
   {SDL_ECC_BUS_SAFETY_MSS_MCRC_DED_Test,                            "MSS_MCRC_DED_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
   {SDL_ECC_BUS_SAFETY_MSS_MCRC_SEC_End_Addr_Test,                   "MSS_MCRC_SEC_End_Addr_Test in Interrupt Method",                SDL_APP_TEST_NOT_RUN },
   {SDL_ECC_BUS_SAFETY_MSS_MCRC_DED_End_Addr_Test,                   "MSS_MCRC_DED_End_Addr_Test in Interrupt Method",                SDL_APP_TEST_NOT_RUN },
   {SDL_ECC_BUS_SAFETY_MSS_MCRC_RED_Test,                            "MSS_MCRC_RED_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
   {SDL_ECC_BUS_SAFETY_MSS_MCRC_RED_FI_Main_Test,                    "MSS_MCRC_RED_FI_Main_Test in Interrupt Method",                 SDL_APP_TEST_NOT_RUN },
   {SDL_ECC_BUS_SAFETY_MSS_MCRC_RED_FI_Safe_Test,                    "MSS_MCRC_RED_FI_Safe_Test in Interrupt Method",                 SDL_APP_TEST_NOT_RUN },
   {SDL_ECC_BUS_SAFETY_MSS_MCRC_RED_FI_Global_Main_Test,             "MSS_MCRC_RED_FI_Global_Main_Test in Interrupt Method",          SDL_APP_TEST_NOT_RUN },

   /* MSS QSPI */
   {SDL_ECC_BUS_SAFETY_MSS_QSPI_SEC_Test,                            "MSS_QSPI_SEC_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
   {SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_Test,                            "MSS_QSPI_DED_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
   {SDL_ECC_BUS_SAFETY_MSS_QSPI_SEC_End_Addr_Test,                   "MSS_QSPI_SEC_End_Addr_Test in Interrupt Method",                SDL_APP_TEST_NOT_RUN },
   {SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_End_Addr_Test,                   "MSS_QSPI_DED_End_Addr_Test in Interrupt Method",                SDL_APP_TEST_NOT_RUN },
   {SDL_ECC_BUS_SAFETY_MSS_QSPI_RED_Test,                            "MSS_QSPI_RED_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
   {SDL_ECC_BUS_SAFETY_MSS_QSPI_RED_FI_Main_Test,                    "MSS_QSPI_RED_FI_Main_Test in Interrupt Method",                 SDL_APP_TEST_NOT_RUN },
   {SDL_ECC_BUS_SAFETY_MSS_QSPI_RED_FI_Safe_Test,                    "MSS_QSPI_RED_FI_Safe_Test in Interrupt Method",                 SDL_APP_TEST_NOT_RUN },
   {SDL_ECC_BUS_SAFETY_MSS_QSPI_RED_FI_Global_Main_Test,             "MSS_QSPI_RED_FI_Global_Main_Test in Interrupt Method",          SDL_APP_TEST_NOT_RUN },

   /* MSS SWBUF */
  {SDL_ECC_BUS_SAFETY_MSS_SWBUF_SEC_Test,                            "MSS_SWBUF_SEC_Test in Interrupt Method",                        SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_SWBUF_DED_Test,                            "MSS_SWBUF_DED_Test in Interrupt Method",                        SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_SWBUF_SEC_End_Addr_Test,                   "MSS_SWBUF_SEC_End_Addr_Test in Interrupt Method",               SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_SWBUF_DED_End_Addr_Test,                   "MSS_SWBUF_DED_End_Addr_Test in Interrupt Method",               SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_SWBUF_RED_Test,                            "MSS_SWBUF_RED_Test in Interrupt Method",                        SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_SWBUF_RED_FI_Main_Test,                    "MSS_SWBUF_RED_FI_Main_Test in Interrupt Method",                SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_SWBUF_RED_FI_Safe_Test,                    "MSS_SWBUF_RED_FI_Safe_Test in Interrupt Method",                SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_SWBUF_RED_FI_Global_Main_Test,             "MSS_SWBUF_RED_FI_Global_Main_Test in Interrupt Method",         SDL_APP_TEST_NOT_RUN },

  /* MSS_TO_MDO */
  {SDL_ECC_BUS_SAFETY_MSS_TO_MDO_SEC_Test,                           "MSS_TO_MDO_SEC_Test in Interrupt Method",                       SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_TO_MDO_DED_Test,                           "MSS_TO_MDO_DED_Test in Interrupt Method",                       SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_TO_MDO_SEC_End_Addr_Test,                  "MSS_TO_MDO_SEC_End_Addr_Test in Interrupt Method",              SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_TO_MDO_DED_End_Addr_Test,                  "MSS_TO_MDO_DED_End_Addr_Test in Interrupt Method",              SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_TO_MDO_RED_Test,                           "MSS_TO_MDO_RED_Test in Interrupt Method",                       SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_TO_MDO_RED_FI_Main_Test,                   "MSS_TO_MDO_RED_FI_Main_Test in Interrupt Method",               SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_TO_MDO_RED_FI_Safe_Test,                   "MSS_TO_MDO_RED_FI_Safe_Test in Interrupt Method",               SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_TO_MDO_RED_FI_Global_Main_Test,            "MSS_TO_MDO_RED_FI_Global_Main_Test in Interrupt Method",        SDL_APP_TEST_NOT_RUN },

   /* DAP_R232 */
  {SDL_ECC_BUS_SAFETY_DAP_R232_RED_Test,                             "DAP_R232_RED_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_DAP_R232_RED_FI_Main_Test,                     "DAP_R232_RED_FI_Main_Test in Interrupt Method",                 SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_DAP_R232_RED_FI_Safe_Test,                     "DAP_R232_RED_FI_Safe_Test in Interrupt Method",                 SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_DAP_R232_RED_FI_Global_Main_Test,              "DAP_R232_RED_FI_Global_Main_Test in Interrupt Method",          SDL_APP_TEST_NOT_RUN },

  /* MSS_SCRP */
  {SDL_ECC_BUS_SAFETY_MSS_SCRP_RED_Test,                             "MSS_SCRP_RED_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_SCRP_RED_FI_Main_Test,                     "MSS_SCRP_RED_FI_Main_Test in Interrupt Method",                 SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_SCRP_RED_FI_Safe_Test,                     "MSS_SCRP_RED_FI_Safe_Test in Interrupt Method",                 SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_SCRP_RED_FI_Global_Main_Test,              "MSS_SCRP_RED_FI_Global_Main_Test in Interrupt Method",          SDL_APP_TEST_NOT_RUN },

  /*MSS L2 A*/
  {SDL_ECC_BUS_SAFETY_MSS_L2_A_RED_Test,                             "MSS_L2_A_RED_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_L2_A_RED_FI_Main_Test,                     "MSS_L2_A_RED_Test with fi mainin Interrupt Method",             SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_L2_A_RED_FI_Safe_Test,                     "MSS_L2_A_RED_Test with fi safe Interrupt Method",               SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_L2_A_RED_FI_Global_Main_Test,              "MSS_L2_A_RED_Test with fi global safe Interrupt Method",        SDL_APP_TEST_NOT_RUN },
  /*MSS L2 B*/
  {SDL_ECC_BUS_SAFETY_MSS_L2_B_RED_Test,                             "MSS_L2_B_RED_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_L2_B_RED_FI_Main_Test,                     "MSS_L2_B_RED_Test with fi mainin Interrupt Method",             SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_L2_B_RED_FI_Safe_Test,                     "MSS_L2_B_RED_Test with fi safe Interrupt Method",               SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_L2_B_RED_FI_Global_Main_Test,              "MSS_L2_B_RED_Test with fi global safe Interrupt Method",        SDL_APP_TEST_NOT_RUN },
  /* Node MSS_DMM_SLV */                                             
  {SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_SEC_Test,                          "MSS_DMM_SLV_SEC_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_DED_Test,                          "MSS_DMM_SLV_DED_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_RED_Test,                          "MSS_DMM_SLV_RED_Test in Interrupt Method",                      SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_SEC_End_Addr_Test,                 "MSS_DMM_SLV_SEC_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_DED_End_Addr_Test,                 "MSS_DMM_SLV_DED_Test for end address in Interrupt Method",      SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_RED_FI_Main_Test,                  "MSS_DMM_SLV_RED_Test with fi mainin Interrupt Method",          SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_RED_FI_Safe_Test,                  "MSS_DMM_SLV_RED_Test with fi safe Interrupt Method",            SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_DMM_SLV_RED_FI_Global_Main_Test,           "MSS_DMM_SLV_RED_Test with fi global safe Interrupt Method",     SDL_APP_TEST_NOT_RUN },
  /* Node MSS_DMM */                                                 
  {SDL_ECC_BUS_SAFETY_MSS_DMM_RED_Test,                              "MSS_DMM_RED_Test in Interrupt Method",                          SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_DMM_RED_FI_Main_Test,                      "MSS_DMM_RED_Test with fi mainin Interrupt Method",              SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_DMM_RED_FI_Safe_Test,                      "MSS_DMM_RED_Test with fi safe Interrupt Method",                SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_DMM_RED_FI_Global_Main_Test,               "MSS_DMM_RED_Test with fi global safe Interrupt Method",         SDL_APP_TEST_NOT_RUN },
  /* Node MSS_GPADC */                                               
  {SDL_ECC_BUS_SAFETY_MSS_GPADC_SEC_Test,                            "MSS_GPADC_SEC_Test in Interrupt Method",                        SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_GPADC_DED_Test,                            "MSS_GPADC_DED_Test in Interrupt Method",                        SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_GPADC_RED_Test,                            "MSS_GPADC_RED_Test in Interrupt Method",                        SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_GPADC_SEC_End_Addr_Test,                   "MSS_GPADC_SEC_Test for end address in Interrupt Method",        SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_GPADC_DED_End_Addr_Test,                   "MSS_GPADC_DED_Test for end address in Interrupt Method",        SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_GPADC_RED_FI_Main_Test,                    "MSS_GPADC_RED_Test with fi mainin Interrupt Method",            SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_GPADC_RED_FI_Safe_Test,                    "MSS_GPADC_RED_Test with fi safe Interrupt Method",              SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_GPADC_RED_FI_Global_Main_Test,             "MSS_GPADC_RED_Test with fi global safe Interrupt Method",       SDL_APP_TEST_NOT_RUN },
  /* Node MSS_PCR */                                                 
  {SDL_ECC_BUS_SAFETY_MSS_PCR_SEC_Test,                              "MSS_PCR_SEC_Test in Interrupt Method",                          SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_PCR_DED_Test,                              "MSS_PCR_DED_Test in Interrupt Method",                          SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_PCR_RED_Test,                              "MSS_PCR_RED_Test in Interrupt Method",                          SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_PCR_RED_FI_Main_Test,                      "MSS_PCR_RED_Test with fi mainin Interrupt Method",              SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_PCR_RED_FI_Safe_Test,                      "MSS_PCR_RED_Test with fi safe Interrupt Method",                SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_PCR_RED_FI_Global_Main_Test,               "MSS_PCR_RED_Test with fi global safe Interrupt Method",         SDL_APP_TEST_NOT_RUN },
  /* Node MSS_PCR2 */                                                
  {SDL_ECC_BUS_SAFETY_MSS_PCR2_SEC_Test,                             "MSS_PCR2_SEC_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_PCR2_DED_Test,                             "MSS_PCR2_DED_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_PCR2_RED_Test,                             "MSS_PCR2_RED_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_PCR2_RED_FI_Main_Test,                     "MSS_PCR2_RED_Test with fi mainin Interrupt Method",             SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_PCR2_RED_FI_Safe_Test,                     "MSS_PCR2_RED_Test with fi safe Interrupt Method",               SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_PCR2_RED_FI_Global_Main_Test,              "MSS_PCR2_RED_Test with fi global safe Interrupt Method",        SDL_APP_TEST_NOT_RUN },
  /* Node MSS_CPSW */                                                
  {SDL_ECC_BUS_SAFETY_MSS_CPSW_RED_Test,                             "MSS_CPSW_RED_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CPSW_RED_FI_Main_Test,                     "MSS_CPSW_RED_Test with fi mainin Interrupt Method",             SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CPSW_RED_FI_Safe_Test,                     "MSS_CPSW_RED_Test with fi safe Interrupt Method",               SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CPSW_RED_FI_Global_Main_Test,              "MSS_CPSW_RED_Test with fi global safe Interrupt Method",        SDL_APP_TEST_NOT_RUN },

  /* MSS_CR5A_AXI_WR */
  {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_Test,                      "MSS_CR5A_AXI_WR_RED_Test in Interrupt Method",                  SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_FI_Main_Test,              "MSS_CR5A_AXI_WR_RED_FI_Main_Test in Interrupt Method",          SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_FI_Safe_Test,              "MSS_CR5A_AXI_WR_RED_FI_Safe_Test in Interrupt Method",          SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_FI_Global_Main_Test,       "MSS_CR5A_AXI_WR_RED_FI_Global_Main_Test in Interrupt Method",   SDL_APP_TEST_NOT_RUN },

  /* MSS_CR5B_AXI_WR */
  {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_Test,                      "MSS_CR5B_AXI_WR_RED_Test in Interrupt Method",                  SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_FI_Main_Test,              "MSS_CR5B_AXI_WR_RED_FI_Main_Test in Interrupt Method",          SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_FI_Safe_Test,              "MSS_CR5B_AXI_WR_RED_FI_Safe_Test in Interrupt Method",          SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_FI_Global_Main_Test,       "MSS_CR5B_AXI_WR_RED_FI_Global_Main_Test in Interrupt Method",   SDL_APP_TEST_NOT_RUN },

  /* MSS_CR5A_AXI_RD */
  {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_RED_Test,                      "MSS_CR5A_AXI_RD_RED_Test in Interrupt Method",                  SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_RED_FI_Main_Test,              "MSS_CR5A_AXI_RD_RED_FI_Main_Test in Interrupt Method",          SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_RED_FI_Safe_Test,              "MSS_CR5A_AXI_RD_RED_FI_Safe_Test in Interrupt Method",          SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_RED_FI_Global_Main_Test,       "MSS_CR5A_AXI_RD_RED_FI_Global_Main_Test in Interrupt Method",   SDL_APP_TEST_NOT_RUN },

  /* MSS_CR5B_AXI_RD */
  {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_RED_Test,                      "MSS_CR5B_AXI_RD_RED_Test in Interrupt Method",                  SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_RED_FI_Main_Test,              "MSS_CR5B_AXI_RD_RED_FI_Main_Test in Interrupt Method",          SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_RED_FI_Safe_Test,              "MSS_CR5B_AXI_RD_RED_FI_Safe_Test in Interrupt Method",          SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_RED_FI_Global_Main_Test,       "MSS_CR5B_AXI_RD_RED_FI_Global_Main_Test in Interrupt Method",   SDL_APP_TEST_NOT_RUN },

  /* MSS_CR5A_AXI_S */
  {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_Test,                       "MSS_CR5A_AXI_S_RED_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_FI_Main_Test,               "MSS_CR5A_AXI_S_RED_FI_Main_Test in Interrupt Method",           SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_FI_Safe_Test,               "MSS_CR5A_AXI_S_RED_FI_Safe_Test in Interrupt Method",           SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_FI_Global_Main_Test,        "MSS_CR5A_AXI_S_RED_FI_Global_Main_Test in Interrupt Method",    SDL_APP_TEST_NOT_RUN },

  /* MSS_CR5B_AXI_S */
  {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_Test,                       "MSS_CR5B_AXI_S_RED_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_FI_Main_Test,               "MSS_CR5B_AXI_S_RED_FI_Main_Test in Interrupt Method",           SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_FI_Safe_Test,               "MSS_CR5B_AXI_S_RED_FI_Safe_Test in Interrupt Method",           SDL_APP_TEST_NOT_RUN },
  {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_FI_Global_Main_Test,        "MSS_CR5B_AXI_S_RED_FI_Global_Main_Test in Interrupt Method",    SDL_APP_TEST_NOT_RUN },

#endif
#endif
#if defined (SOC_AM263X)
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_RED_Test,                       "CR5A_AHB_RED_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_RED_Test,                       "CR5B_AHB_RED_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB_RED_Test,                       "CR5C_AHB_RED_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB_RED_Test,                       "CR5D_AHB_RED_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },
     /* MSS TPTC A0 WR */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test,                     "TPTC_A0_WR_RED_Test in Interrupt Method",                       SDL_APP_TEST_NOT_RUN },
    /* MSS TPTC A1 WR */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test,                     "TPTC_A1_WR_RED_Test in Interrupt Method",                       SDL_APP_TEST_NOT_RUN },
    /* MSS TPTC A0 RD */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_Test,                     "TPTC_A0_RD_SEC_Test in Interrupt Method",                       SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_Test,                     "TPTC_A0_RD_DED_Test in Interrupt Method",                       SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_Test,                     "TPTC_A0_RD_RED_Test in Interrupt Method",                       SDL_APP_TEST_NOT_RUN },
    /* MSS TPTC A1 RD */
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_Test,                     "TPTC_A1_RD_SEC_Test in Interrupt Method",                       SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_Test,                     "TPTC_A1_RD_DED_Test in Interrupt Method",                       SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_Test,                     "TPTC_A1_RD_RED_Test in Interrupt Method",                       SDL_APP_TEST_NOT_RUN },
    /* MSS MBOX */
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_SEC_Test,                           "MSS_MBOX_SEC_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_Test,                           "MSS_MBOX_DED_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_RED_Test,                           "MSS_MBOX_RED_Test in Interrupt Method",                         SDL_APP_TEST_NOT_RUN },

    /* MSS_MCRC */
    {SDL_ECC_BUS_SAFETY_MSS_MCRC_SEC_Test,                           "MSS_MCRC_SEC_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MCRC_DED_Test,                           "MSS_MCRC_DED_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MCRC_RED_Test,                           "MSS_MCRC_RED_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },

    /* MSS_QSPI */
    {SDL_ECC_BUS_SAFETY_MSS_QSPI_SEC_Test,                           "MSS_QSPI_SEC_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_Test,                           "MSS_QSPI_DED_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_QSPI_RED_Test,                           "MSS_QSPI_RED_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },

    /* MSS_STM_STIM */
    {SDL_ECC_BUS_SAFETY_MSS_STM_STIM_SEC_Test,                       "MSS_STM_STIM_SEC_Test in Interrupt  Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_STM_STIM_DED_Test,                       "MSS_STM_STIM_DED_Test in Interrupt  Method",                    SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_STM_STIM_RED_Test,                       "MSS_STM_STIM_RED_Test in Interrupt  Method",                    SDL_APP_TEST_NOT_RUN },

    /* MSS_SCRP0 */
    {SDL_ECC_BUS_SAFETY_MSS_SCRP0_SEC_Test,                          "MSS_SCRP0_SEC_Test in Interrupt  Method",                       SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_SCRP0_DED_Test,                          "MSS_SCRP0_DED_Test in Interrupt  Method",                       SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_SCRP0_RED_Test,                          "MSS_SCRP0_RED_Test in Interrupt  Method",                       SDL_APP_TEST_NOT_RUN },

    /* MSS_SCRP1 */
    {SDL_ECC_BUS_SAFETY_MSS_SCRP1_SEC_Test,                          "MSS_SCRP1_SEC_Test in Interrupt  Method",                       SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_SCRP1_DED_Test,                          "MSS_SCRP1_DED_Test in Interrupt  Method",                       SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_SCRP1_RED_Test,                          "MSS_SCRP1_RED_Test in Interrupt  Method",                       SDL_APP_TEST_NOT_RUN },

    /* ICSSM_PDSP0 */
    {SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_SEC_Test,                        "ICSSM_PDSP0_SEC_Test in Interrupt  Method",                     SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_DED_Test,                        "ICSSM_PDSP0_DED_Test in Interrupt  Method",                     SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_RED_Test,                        "ICSSM_PDSP0_RED_Test in Interrupt  Method",                     SDL_APP_TEST_NOT_RUN },

    /* ICSSM_PDSP1 */
    {SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_SEC_Test,                        "ICSSM_PDSP1_SEC_Test in Interrupt  Method",                     SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_DED_Test,                        "ICSSM_PDSP1_DED_Test in Interrupt  Method",                     SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_RED_Test,                        "ICSSM_PDSP1_RED_Test in Interrupt  Method",                     SDL_APP_TEST_NOT_RUN },

    /* ICSSM_S */
    {SDL_ECC_BUS_SAFETY_ICSSM_S_SEC_Test,                            "ICSSM_S_SEC_Test in Interrupt  Method",                         SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_ICSSM_S_DED_Test,                            "ICSSM_S_DED_Test in Interrupt  Method",                         SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_ICSSM_S_RED_Test,                            "ICSSM_S_RED_Test in Interrupt  Method",                         SDL_APP_TEST_NOT_RUN },

    /* DAP */
    {SDL_ECC_BUS_SAFETY_DAP_SEC_Test,                                "DAP_SEC_Test in Interrupt  Method",                             SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DAP_DED_Test,                                "DAP_DED_Test in Interrupt  Method",                             SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DAP_RED_Test,                                "DAP_RED_Test in Interrupt  Method",                             SDL_APP_TEST_NOT_RUN },
    /* MSS CR5A_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_Test,                    "MSS_CR5A_AXI_WR_RED_Test in Interrupt Method",                  SDL_APP_TEST_NOT_RUN },
    /* MSS CR5B_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_Test,                    "MSS_CR5B_AXI_WR_RED_Test in Interrupt Method",                  SDL_APP_TEST_NOT_RUN },
    /* MSS CR5C_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_RED_Test,                    "MSS_CR5C_AXI_WR_RED_Test in Interrupt Method",                  SDL_APP_TEST_NOT_RUN },
    /* MSS CR5D_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_RED_Test,                    "MSS_CR5D_AXI_WR_RED_Test in Interrupt Method",                  SDL_APP_TEST_NOT_RUN },
    /* MSS CR5A_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_SEC_Test,                    "MSS_CR5A_AXI_RD_SEC_Test in Interrupt Method",                  SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_RED_Test,                    "MSS_CR5A_AXI_RD_RED_Test in Interrupt Method",                  SDL_APP_TEST_NOT_RUN },
    /* MSS CR5B_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_SEC_Test,                    "MSS_CR5B_AXI_RD_SEC_Test in Interrupt Method",                  SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_RED_Test,                    "MSS_CR5B_AXI_RD_RED_Test in Interrupt Method",                  SDL_APP_TEST_NOT_RUN },
    /* MSS CR5C_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_SEC_Test,                    "MSS_CR5C_AXI_RD_SEC_Test in Interrupt Method",                  SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_RED_Test,                    "MSS_CR5C_AXI_RD_RED_Test in Interrupt Method",                  SDL_APP_TEST_NOT_RUN },
    /* MSS CR5D_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_SEC_Test,                    "MSS_CR5D_AXI_RD_SEC_Test in Interrupt Method",                  SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_RED_Test,                    "MSS_CR5D_AXI_RD_RED_Test in Interrupt Method",                  SDL_APP_TEST_NOT_RUN },
    /* MSS CR5A_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_SEC_Test,                     "MSS_CR5A_AXI_S_SEC_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_Test,                     "MSS_CR5A_AXI_S_RED_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    /* MSS CR5B_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_SEC_Test,                     "MSS_CR5B_AXI_S_SEC_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_Test,                     "MSS_CR5B_AXI_S_RED_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    /* MSS CR5C_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_SEC_Test,                     "MSS_CR5C_AXI_S_SEC_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_RED_Test,                     "MSS_CR5C_AXI_S_RED_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    /* MSS CR5D_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_SEC_Test,                     "MSS_CR5D_AXI_S_SEC_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_RED_Test,                     "MSS_CR5D_AXI_S_RED_Test in Interrupt Method",                   SDL_APP_TEST_NOT_RUN },
    /* Node MSS_MMC_S */
    {SDL_ECC_BUS_SAFETY_MSS_MMC_SEC_Test,                            "MSS_MMC_S_SEC_Test in Interrupt  Method",                       SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MMC_DED_Test,                            "MSS_MMC_S_DED_Test in Interrupt  Method",                       SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MMC_RED_Test,                            "MSS_MMC_S_RED_Test in Interrupt  Method",                       SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MMC_SEC_End_Addr_Test,                   "MSS_MMC_SEC_Test for end address in Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MMC_DED_End_Addr_Test,                   "MSS_MMC_DED_Test for end address in Interrupt Method",          SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MMC_RED_FI_Main_Test,                    "MSS_MMC_RED_Test with fi mainin Interrupt Method",              SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MMC_RED_FI_Safe_Test,                    "MSS_MMC_RED_Test with fi safe Interrupt Method",                SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MMC_RED_FI_Global_Main_Test,             "MSS_MMC_RED_Test with fi global safe Interrupt Method",         SDL_APP_TEST_NOT_RUN },

    /* Node MSS_GPMC */
    {SDL_ECC_BUS_SAFETY_MSS_GPMC_SEC_Test,                           "MSS_GPMC_SEC_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_GPMC_DED_Test,                           "MSS_GPMC_DED_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_GPMC_RED_Test,                           "MSS_GPMC_RED_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_GPMC_SEC_End_Addr_Test,                  "MSS_GPMC_SEC_Test for end address in Interrupt Method",         SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_GPMC_DED_End_Addr_Test,                  "MSS_GPMC_DED_Test for end address in Interrupt Method",         SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_GPMC_RED_FI_Main_Test,                   "MSS_GPMC_RED_Test with fi mainin Interrupt Method",             SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_GPMC_RED_FI_Safe_Test,                   "MSS_GPMC_RED_Test with fi safe Interrupt Method",               SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_GPMC_RED_FI_Global_Main_Test,            "MSS_GPMC_RED_Test with fi global safe Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    /* Node MSS_L2_A */
    {SDL_ECC_BUS_SAFETY_MSS_L2_A_RED_Test,                           "MSS_L2_A_RED_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_L2_A_RED_FI_Main_Test,                   "MSS_L2_A_RED_Test with fi mainin Interrupt Method",             SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_L2_A_RED_FI_Safe_Test,                   "MSS_L2_A_RED_Test with fi safe Interrupt Method",               SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_L2_A_RED_FI_Global_Main_Test,            "MSS_L2_A_RED_Test with fi global safe Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    /* Node MSS_L2_B */
    {SDL_ECC_BUS_SAFETY_MSS_L2_B_RED_Test,                           "MSS_L2_B_RED_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_L2_B_RED_FI_Main_Test,                   "MSS_L2_B_RED_Test with fi mainin Interrupt Method",             SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_L2_B_RED_FI_Safe_Test,                   "MSS_L2_B_RED_Test with fi safe Interrupt Method",               SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_L2_B_RED_FI_Global_Main_Test,            "MSS_L2_B_RED_Test with fi global safe Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    /* Node MSS_L2_C */
    {SDL_ECC_BUS_SAFETY_MSS_L2_C_RED_Test,                           "MSS_L2_C_RED_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_L2_C_RED_FI_Main_Test,                   "MSS_L2_C_RED_Test with fi mainin Interrupt Method",             SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_L2_C_RED_FI_Safe_Test,                   "MSS_L2_C_RED_Test with fi safe Interrupt Method",               SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_L2_C_RED_FI_Global_Main_Test,            "MSS_L2_C_RED_Test with fi global safe Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    /* Node MSS_L2_D */
    {SDL_ECC_BUS_SAFETY_MSS_L2_D_RED_Test,                           "MSS_L2_D_RED_Test in Interrupt  Method",                        SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_L2_D_RED_FI_Main_Test,                   "MSS_L2_D_RED_Test with fi mainin Interrupt Method",             SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_L2_D_RED_FI_Safe_Test,                   "MSS_L2_D_RED_Test with fi safe Interrupt Method",               SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_L2_D_RED_FI_Global_Main_Test,            "MSS_L2_D_RED_Test with fi global safe Interrupt Method",        SDL_APP_TEST_NOT_RUN },
    /* Node MAIN_VBUSP */
    {SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP_RED_Test,                     "MAIN_VBUSP_RED_Test in Interrupt  Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP_RED_FI_Main_Test,             "MSS_MAIN_VBUSP_RED_Test with fi mainin Interrupt Method",       SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP_RED_FI_Safe_Test,             "MSS_MAIN_VBUSP_RED_Test with fi safe Interrupt Method",         SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP_RED_FI_Global_Main_Test,      "MSS_MAIN_VBUSP_RED_Test with fi global safe Interrupt Method",  SDL_APP_TEST_NOT_RUN },
    /* Node _PERI_VBUSP */
    {SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP_RED_Test,                     "PERI_VBUSP_RED_Test in Interrupt  Method",                      SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP_RED_FI_Main_Test,             "MSS_PERI_VBUSP_RED_Test with fi mainin Interrupt Method",       SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP_RED_FI_Safe_Test,             "MSS_PERI_VBUSP_RED_Test with fi safe Interrupt Method",         SDL_APP_TEST_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP_RED_FI_Global_Main_Test,      "MSS_PERI_VBUSP_RED_Test with fi global safe Interrupt Method",  SDL_APP_TEST_NOT_RUN },
#endif
    {NULL,                                                           "TERMINATING CONDITION",                                         SDL_APP_TEST_NOT_RUN }
};

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/

#ifdef UNITY_INCLUDE_CONFIG_H
/*
 *  ======== Unity set up and tear down ========
 */
void setUp(void)
{
    /* Do nothing */
}

void tearDown(void)
{
    /* Do nothing */
}
#endif

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

static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("Error: Init Failed\r\n");
    }

    return ret;
}

/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/
void test_sdl_ecc_bus_safety_baremetal_test_app (void)
{
    /* Declarations of variables */
    int32_t    testResult = SDL_APP_TEST_PASS;
    int32_t    i;
    uint64_t testStartTime;
    uint64_t testEndTime;
    uint64_t diffTime;
    ecc_bus_safety_edma_boardInit();
    /* Init Dpl */
    sdlApp_dplInit();
#if defined (SOC_AWR294X) || (SOC_AM273X)
#if defined (SUBSYS_MSS)
    for( i =0; i<30U; i++)
    {
	    testResult = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &ECC_BUS_SAFETY_TestparamsMSS[i],NULL,NULL);
    }
#endif
#endif
#if defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)
    for( i =0; i<33U; i++)
    {
	    testResult = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_BUS_SAFETY_TestparamsDSS[i],NULL,NULL);
    }
#endif
#endif
#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)
    for( i =0; i<33U; i++)
    {
	    testResult = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &ECC_BUS_SAFETY_TestparamsDSS[i],NULL,NULL);
    }
#endif
#endif
    DebugP_log("\n ECC BUS SAFETY Test Application\r\n");

    for ( i = 0; sdlEccBusSftyTestList[i].testFunction != NULL; i++)
    {
        /* Get start time of test */
        testStartTime = ClockP_getTimeUsec();
        testResult = sdlEccBusSftyTestList[i].testFunction();
        /* Record test end time */
        testEndTime = ClockP_getTimeUsec();
        diffTime = testEndTime-testStartTime;
        sdlEccBusSftyTestList[i].testStatus = testResult;
        sdlEccBusSftyTestList[i].test_time =diffTime;
    }
    testResult = SDL_APP_TEST_PASS;
    for ( i = 0; sdlEccBusSftyTestList[i].testFunction != NULL; i++)
    {
        if (sdlEccBusSftyTestList[i].testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("\n Applications Name: %s  FAILED and Time taken for the Test is %d  micro secs \r\n", sdlEccBusSftyTestList[i].name, (uint32_t)sdlEccBusSftyTestList[i].test_time);
            testResult = SDL_APP_TEST_FAILED;
            break;
        }
        else
        {
           DebugP_log("\nApplications Name: %s  PASSED  and Time taken for the Test is %d  micro secs \r\n", sdlEccBusSftyTestList[i].name ,(uint32_t)sdlEccBusSftyTestList[i].test_time );
        }
    }

    if (testResult == SDL_APP_TEST_PASS)
    {
        DebugP_log("\nAll tests have passed\r\n");
    }
    else
    {
        DebugP_log("\nFew/all tests Failed \r\n");
    }

#if defined (UNITY_INCLUDE_CONFIG_H)
    TEST_ASSERT_EQUAL_INT32(SDL_APP_TEST_PASS, testResult);
#endif
}

void test_sdl_ecc_bus_safety_baremetal_test_app_runner(void)
{
#if defined(UNITY_INCLUDE_CONFIG_H)
    UNITY_BEGIN();
    RUN_TEST (test_sdl_ecc_bus_safety_baremetal_test_app);
    UNITY_END();
    /* Function to print results defined in our unity_config.h file */
    print_unityOutputBuffer_usingUARTstdio();
#else
    test_sdl_ecc_bus_safety_baremetal_test_app();
#endif
    ecc_bus_safety_edma_boardDeinit();
    return;
}

void test_main(void *args)
{
    test_sdl_ecc_bus_safety_baremetal_test_app_runner();
}

/* Nothing past this point */
