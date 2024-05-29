/* Copyright (c) 2022-2024 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *
 *  Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the
 *  distribution.
 *
 *  Neither the name of Texas Instruments Incorporated nor the names of
 *  its contributors may be used to endorse or promote products derived
 *  from this software without specific prior written permission.
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
#include "sdlexample.h"

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/* Define the  application interface */
typedef struct sdlEccBusSafetyApp_s
{
    int32_t  (*application)(void);    /* The code that runs the application */
    char      *name;                  /* The application name */
    int32_t    status;                /* App Status */
} sdlEccBusSafetyApp_t;

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/

sdlEccBusSafetyApp_t  sdlEccBusSafetyAppTestList[] = {
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

    /* MSS MBOX */
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_SEC_Test,                    "MSS_MBOX_SEC_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_Test,                    "MSS_MBOX_DED_Test in Interrupt Method",          SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MBOX_RED_Test,                    "MSS_MBOX_RED_Test in Interrupt Method",          SDL_APP_NOT_RUN },

   /* MSS_MCRC */
    {SDL_ECC_BUS_SAFETY_MSS_MCRC_SEC_Test,                    "MSS_MCRC_SEC_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MCRC_DED_Test,                    "MSS_MCRC_DED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MCRC_RED_Test,                    "MSS_MCRC_RED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },

    /* MSS_QSPI */
    {SDL_ECC_BUS_SAFETY_MSS_QSPI_SEC_Test,                    "MSS_QSPI_SEC_Test in Interrupt  Method",          SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_Test,                    "MSS_QSPI_DED_Test in Interrupt  Method",          SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_QSPI_RED_Test,                    "MSS_QSPI_RED_Test in Interrupt  Method",          SDL_APP_NOT_RUN },

    /* MSS_STM_STIM */
    {SDL_ECC_BUS_SAFETY_MSS_STM_STIM_SEC_Test,                "MSS_STM_STIM_SEC_Test in Interrupt  Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_STM_STIM_DED_Test,                "MSS_STM_STIM_DED_Test in Interrupt  Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_STM_STIM_RED_Test,                "MSS_STM_STIM_RED_Test in Interrupt  Method",      SDL_APP_NOT_RUN },

    /* MSS_SCRP0 */
    {SDL_ECC_BUS_SAFETY_MSS_SCRP0_SEC_Test,                   "MSS_SCRP0_SEC_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_SCRP0_DED_Test,                   "MSS_SCRP0_DED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_SCRP0_RED_Test,                   "MSS_SCRP0_RED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },

    /* MSS_SCRP1 */
    {SDL_ECC_BUS_SAFETY_MSS_SCRP1_SEC_Test,                   "MSS_SCRP1_SEC_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_SCRP1_DED_Test,                   "MSS_SCRP1_DED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_SCRP1_RED_Test,                   "MSS_SCRP1_RED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },

    /* ICSSM_PDSP0 */
    {SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_SEC_Test,                 "ICSSM_PDSP0_SEC_Test in Interrupt  Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_DED_Test,                 "ICSSM_PDSP0_DED_Test in Interrupt  Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_RED_Test,                 "ICSSM_PDSP0_RED_Test in Interrupt  Method",        SDL_APP_NOT_RUN },

    /* ICSSM_PDSP1 */
    {SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_SEC_Test,                 "ICSSM_PDSP1_SEC_Test in Interrupt  Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_DED_Test,                 "ICSSM_PDSP1_DED_Test in Interrupt  Method",        SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_RED_Test,                 "ICSSM_PDSP1_RED_Test in Interrupt  Method",        SDL_APP_NOT_RUN },

    /* ICSSM_S */
    {SDL_ECC_BUS_SAFETY_ICSSM_S_SEC_Test,                     "ICSSM_S_SEC_Test in Interrupt  Method",            SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_ICSSM_S_DED_Test,                     "ICSSM_S_DED_Test in Interrupt  Method",            SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_ICSSM_S_RED_Test,                     "ICSSM_S_RED_Test in Interrupt  Method",            SDL_APP_NOT_RUN },

    /* DAP */
    {SDL_ECC_BUS_SAFETY_DAP_SEC_Test,                         "DAP_SEC_Test in Interrupt  Method",                SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DAP_DED_Test,                         "DAP_DED_Test in Interrupt  Method",                SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_DAP_RED_Test,                         "DAP_RED_Test in Interrupt  Method",                SDL_APP_NOT_RUN },

     /* MSS CR5A_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_Test,             "MSS_CR5A_AXI_WR_RED_Test in Interrupt  Method",    SDL_APP_NOT_RUN },
    /* MSS CR5B_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_Test,             "MSS_CR5B_AXI_WR_RED_Test in Interrupt  Method",    SDL_APP_NOT_RUN },
    /* MSS CR5C_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_RED_Test,             "MSS_CR5C_AXI_WR_RED_Test in Interrupt  Method",    SDL_APP_NOT_RUN },
    /* MSS CR5D_AXI_WR */
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_RED_Test,             "MSS_CR5D_AXI_WR_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* MSS CR5A_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_SEC_Test,             "MSS_CR5A_AXI_RD_SEC_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_RED_Test,             "MSS_CR5A_AXI_RD_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* MSS CR5B_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_SEC_Test,             "MSS_CR5B_AXI_RD_SEC_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_RED_Test,             "MSS_CR5B_AXI_RD_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* MSS CR5C_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_SEC_Test,             "MSS_CR5C_AXI_RD_SEC_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_RED_Test,             "MSS_CR5C_AXI_RD_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* MSS CR5D_AXI_RD */
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_SEC_Test,             "MSS_CR5D_AXI_RD_SEC_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_RED_Test,             "MSS_CR5D_AXI_RD_RED_Test in Interrupt Method",     SDL_APP_NOT_RUN },
    /* MSS CR5A_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_SEC_Test,              "MSS_CR5A_AXI_S_SEC_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_Test,              "MSS_CR5A_AXI_S_RED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    /* MSS CR5B_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_SEC_Test,              "MSS_CR5B_AXI_S_SEC_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_Test,              "MSS_CR5B_AXI_S_RED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    /* MSS CR5C_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_SEC_Test,              "MSS_CR5C_AXI_S_SEC_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_RED_Test,              "MSS_CR5C_AXI_S_RED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    /* MSS CR5D_AXI_S */
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_SEC_Test,              "MSS_CR5D_AXI_S_SEC_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_RED_Test,              "MSS_CR5D_AXI_S_RED_Test in Interrupt Method",      SDL_APP_NOT_RUN },
    /* Node MSS_MMC_S */
    {SDL_ECC_BUS_SAFETY_MSS_MMC_SEC_Test,                     "MSS_MMC_S_SEC_Test in Interrupt  Method",          SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MMC_DED_Test,                     "MSS_MMC_S_DED_Test in Interrupt  Method",          SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_MMC_RED_Test,                     "MSS_MMC_S_RED_Test in Interrupt  Method",          SDL_APP_NOT_RUN },
    /* Node MSS_GPMC */
    {SDL_ECC_BUS_SAFETY_MSS_GPMC_SEC_Test,                    "MSS_GPMC_SEC_Test in Interrupt  Method",           SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_GPMC_DED_Test,                    "MSS_GPMC_DED_Test in Interrupt  Method",           SDL_APP_NOT_RUN },
    {SDL_ECC_BUS_SAFETY_MSS_GPMC_RED_Test,                    "MSS_GPMC_RED_Test in Interrupt  Method",           SDL_APP_NOT_RUN },
    /* Node MSS_L2_A */
    {SDL_ECC_BUS_SAFETY_MSS_L2_A_RED_Test,                    "MSS_L2_A_RED_Test in Interrupt  Method",           SDL_APP_NOT_RUN },
    /* Node MSS_L2_B */
    {SDL_ECC_BUS_SAFETY_MSS_L2_B_RED_Test,                    "MSS_L2_B_RED_Test in Interrupt  Method",           SDL_APP_NOT_RUN },
    /* Node MSS_L2_C */
    {SDL_ECC_BUS_SAFETY_MSS_L2_C_RED_Test,                    "MSS_L2_C_RED_Test in Interrupt  Method",           SDL_APP_NOT_RUN },
    /* Node MSS_L2_D */
    {SDL_ECC_BUS_SAFETY_MSS_L2_D_RED_Test,                    "MSS_L2_D_RED_Test in Interrupt  Method",           SDL_APP_NOT_RUN },
    /* Node MAIN_VBUSP */
    {SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP_RED_Test,              "MAIN_VBUSP_RED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },
    /* Node _PERI_VBUSP */
    {SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP_RED_Test,              "PERI_VBUSP_RED_Test in Interrupt  Method",         SDL_APP_NOT_RUN },

    {NULL,                                                    "TERMINATING CONDITION",                            SDL_APP_NOT_RUN }
};

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/


/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/
/* runs ECC Bus safety diagnostic tests */
int32_t ecc_bus_safety_testExecute(void)
{
    /* Declarations of variables */
    int32_t    result = SDL_APP_PASS;
    int32_t    count;

    for ( count = 0; sdlEccBusSafetyAppTestList[count].application != NULL; count++)
    {
        /* Get start time of test */
        result = sdlEccBusSafetyAppTestList[count].application();
        /* Record test end time */
        sdlEccBusSafetyAppTestList[count].status = result;
    }
    SDL_ESM_enableIntr(SDL_TOP_ESM_U_BASE, ESM_INT_BUSSAFETY1);
    SDL_ESM_enableIntr(SDL_TOP_ESM_U_BASE, ESM_INT_BUSSAFETY2);
    result = SDL_APP_PASS;
    for ( count = 0; sdlEccBusSafetyAppTestList[count].application != NULL; count++)
    {
        if (sdlEccBusSafetyAppTestList[count].status != SDL_APP_PASS)
        {
            result = SDL_EFAIL;
            break;
        }
        else
        {
            result = SDL_PASS;
        }
    }

  return result;
}


/* Nothing past this point */
