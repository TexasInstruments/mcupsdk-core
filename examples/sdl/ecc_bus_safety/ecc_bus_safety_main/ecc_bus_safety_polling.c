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
 *  \file     ecc_bus_safety_polling.c
 *
 *  \brief    This file contains Ecc Bus Safety for all the nodes in MSS and DSS example code in polling method.
 *
 *  \details  ECC_BUS_SAFETY APP
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "ecc_bus_safety.h"
/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
#if defined (SOC_AM273X) ||  (SOC_AWR294X)
#if defined (SUBSYS_DSS)
static int32_t SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(uint32_t busSftyNode ,uint32_t addr );
static int32_t SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(uint32_t busSftyNode ,uint32_t addr);
static int32_t SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(uint32_t busSftyNode, \
                        SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType);
#endif
#endif
#if defined (SUBSYS_MSS)
#if defined (SOC_AM273X) || defined (SOC_AWR294X) || defined (SOC_AM263X)
static int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_Test_Polling(uint32_t busSftyNode ,uint32_t addr );
static int32_t SDL_ECC_BUS_SAFETY_MSS_DED_Test_Polling(uint32_t busSftyNode ,uint32_t addr);
static int32_t SDL_ECC_BUS_SAFETY_MSS_RED_Test_Polling(uint32_t busSftyNode, \
                        SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType);
#endif

#endif
/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/
#if defined (SOC_AM273X) ||  (SOC_AWR294X)
#if defined (SUBSYS_DSS)
/********************************************************************************************************
*   For Node DSS_L3_BANKA
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA ,SDL_DSS_L3_BANKA_ADDRESS ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA ,SDL_DSS_L3_BANKA_ADDRESS));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKA_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_L3_BANKB
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_L3_BANKB ,SDL_DSS_L3_BANKB_ADDRESS ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_L3_BANKB ,SDL_DSS_L3_BANKB_ADDRESS));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKB_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_L3_BANKB,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_L3_BANKC
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_L3_BANKC ,SDL_DSS_L3_BANKC_ADDRESS ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling( SDL_ECC_BUS_SAFETY_DSS_L3_BANKC ,SDL_DSS_L3_BANKC_ADDRESS));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKC_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_L3_BANKC,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_L3_BANKD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_L3_BANKD ,SDL_DSS_L3_BANKD_ADDRESS ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_L3_BANKD ,SDL_DSS_L3_BANKD_ADDRESS));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_L3_BANKD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_L3_BANKD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_HWA_DMA0
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0 ,SDL_DSS_HWA_DMA0_U_BASE ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0 ,SDL_DSS_HWA_DMA0_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_HWA_DMA1
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1 ,SDL_DSS_HWA_DMA1_U_BASE ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1 ,SDL_DSS_HWA_DMA1_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_MBOX
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_MBOX_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_MBOX ,SDL_DSS_MAILBOX_U_BASE ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_MBOX_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling( SDL_ECC_BUS_SAFETY_DSS_MBOX ,SDL_DSS_MAILBOX_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_MBOX_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling( SDL_ECC_BUS_SAFETY_DSS_MBOX,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_MCRC
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_MCRC_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_MCRC ,SDL_DSS_MCRC_U_BASE ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_MCRC_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_MCRC ,SDL_DSS_MCRC_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_MCRC_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_MCRC,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_CBUFF_FIFO
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO ,SDL_DSS_CBUFF_FIFO_U_BASE ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO ,SDL_DSS_CBUFF_FIFO_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_DSP_MDMA
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA ,SDL_DSS_MAILBOX_U_BASE ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA ,SDL_DSS_MAILBOX_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_DSP_SDMA
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA ,SDL_DSS_L2_U_BASE ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA ,SDL_DSS_L2_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_PCR
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_PCR_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_PCR ,SDL_DSS_L2_U_BASE ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_PCR_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_PCR ,SDL_DSS_L2_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_PCR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_PCR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_A0_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_A1_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_B0_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_B1_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_C0_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_C1_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_C2_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_C3_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_C4_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_C5_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_A0_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD ,0U ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD ,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_A1_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD ,0U ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD ,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_B0_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD ,0U ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD ,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}



/********************************************************************************************************
*   For Node DSS_TPTC_B1_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD ,0U ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD ,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_C0_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD ,0U ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD ,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_C1_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD ,0U ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD ,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_C2_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD ,0U ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD ,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_C3_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD ,0U ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD ,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_C4_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD ,0U ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD ,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DSS_TPTC_C5_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD ,0U ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD ,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

#endif
#endif

#if defined (SOC_AM273X)
#if defined (SUBSYS_DSS)

/********************************************************************************************************
*   For Node DSS_MDO_FIFO
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO ,SDL_DSS_MDO_FIFO_U_BASE ));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO ,SDL_DSS_MDO_FIFO_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

#endif
#endif


#if defined (SOC_AWR294X)
#if defined (SUBSYS_DSS)

/********************************************************************************************************
*   For Node RSS_MBOX
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_RSS_MBOX_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_RSS_MBOX ,SDL_RSS_CR4_MBOX_U_BASE ));
}

int32_t SDL_ECC_BUS_SAFETY_RSS_MBOX_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_RSS_MBOX ,SDL_RSS_CR4_MBOX_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_RSS_MBOX_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_RSS_MBOX,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node RSS_ADCBUF_WR
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR ,SDL_RSS_ADCBUF_WRITE_U_BASE ));
}

int32_t SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR ,SDL_RSS_ADCBUF_WRITE_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node RSS_ADCBUF_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_RSS_ADCBUF_RD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_RSS_ADCBUF_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#endif
#endif

#if defined (SOC_AM273X) ||  (SOC_AWR294X)
#if defined (SUBSYS_DSS)

/********************************************************************************************************
*   For DSS SEC
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_DSS_SEC_Test_Polling(uint32_t busSftyNode ,uint32_t addr )
{
    int32_t retval = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t writeData = 0x1234567U;
    uint32_t status = 0U;
    uint32_t dmaCh = 0U;
    uint32_t tcc = 0U;
    uint32_t param = 0U;
    uint32_t queNum = 0U;
    uint32_t edmaNum = 0U;
    retval = SDL_ECC_BUS_SAFETY_DSS_secExecute(busSftyNode,addr,writeData);
    if(retval !=SDL_PASS )
    {
        retval = SDL_EFAIL;
    }
    else
    {
        if((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD>=busSftyNode))
        {
            SDL_ECC_BUS_SAFETY_DSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            test_edmaATransfer(busSftyNode, dmaCh,tcc,param,queNum,edmaNum );
        }
        else
        {
             /* Do Nothing */
        }
        /* Wait for test to complete/timeout. */
        while((status == 0U) && (timeout!=0U))
        {
            retval =SDL_ECC_BUS_SAFETY_DSS_getSecErrorStatus(busSftyNode,&status);
            if(retval !=SDL_PASS )
            {
                timeout--;
            }
            else
            {
                break;
            }

        }
        if(status!=0U)
        {
            retval = SDL_PASS;
            SDL_ECC_BUS_SAFETY_DSS_secErrorClear(busSftyNode );
        }
        else
        {
            retval = SDL_EFAIL;
        }
    }
    return retval;
}

/********************************************************************************************************
*   For DSS DED
*********************************************************************************************************/

static int32_t SDL_ECC_BUS_SAFETY_DSS_DED_Test_Polling(uint32_t busSftyNode ,uint32_t addr)
{
    int32_t retval = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t writeData = 0x1234567U;
    uint32_t status = 0U;
    uint32_t dmaCh = 0U;
    uint32_t tcc = 0U;
    uint32_t param = 0U;
    uint32_t queNum = 0U;
    uint32_t edmaNum = 0U;

    retval = SDL_ECC_BUS_SAFETY_DSS_dedExecute(busSftyNode,addr,writeData);
    if(retval !=SDL_PASS )
    {
        retval = SDL_EFAIL;
    }
    else
    {
        if((SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD>=busSftyNode))
        {
            SDL_ECC_BUS_SAFETY_DSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
        }
        else
        {
            /* Do Nothing */
        }
        /* Wait for test to complete/timeout. */
        while((status == 0U) && (timeout!=0U))
        {
            retval =SDL_ECC_BUS_SAFETY_DSS_getDedErrorStatus(busSftyNode,&status);
            if(retval !=SDL_PASS )
            {
                timeout--;
            }
            else
            {
                break;
            }

        }
        if(status!=0U)
        {
            retval = SDL_PASS;
            SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(busSftyNode);
        }
        else
        {
            retval = SDL_EFAIL;
        }
    }
    return retval;
}

/********************************************************************************************************
*   For DSS RED
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_DSS_RED_Test_Polling(uint32_t busSftyNode, SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType)
{
    int32_t retval = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t status = 0U;
    retval= SDL_ECC_BUS_SAFETY_DSS_redExecute(busSftyNode, fiType, redType);
    if(retval !=SDL_PASS )
    {
        retval = SDL_EFAIL;
    }
    else
    {
        /* Wait for test to complete/timeout. */
        while((status == 0U) && (timeout!=0U))
        {
            retval =SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(busSftyNode,&status);
            if(retval !=SDL_PASS )
            {
                timeout--;
            }
            else
            {
                break;
            }

        }
        if(status!=0U)
        {
            retval = SDL_PASS;
            SDL_ECC_BUS_SAFETY_DSS_redErrorClear(busSftyNode);
        }
        else
        {
            retval = SDL_EFAIL;
        }
    }
    return retval;
}

/********************************************************************************************************
*   Function to select the parameters for EDMA config according to Nodes
*********************************************************************************************************/

void SDL_ECC_BUS_SAFETY_DSS_getEDMAParameters(uint32_t busSftyNode , uint32_t *dmaCh, uint32_t *tcc,uint32_t *param, uint32_t *queNum, uint32_t *edmaNum )
{
    switch (busSftyNode)
    {
        /* DSS TPTC A0 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA0;
            break;
        }
        /* DSS TPTC A1 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *queNum = 1U;
            *edmaNum = CONFIG_EDMA0;
            break;
        }
        /* DSS TPTC B0 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ0;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA1;
            break;
        }
        /* DSS TPTC B1 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_B_EVT_HWA_DMA_REQ0;
            *queNum = 1U;
            *edmaNum = CONFIG_EDMA1;
            break;
        }
        /* DSS TPTC C0 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA2;
            break;
        }
        /* DSS TPTC C1 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *queNum = 1U;
            *edmaNum = CONFIG_EDMA2;
            break;
        }
        /* DSS TPTC C2 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *queNum = 2U;
            *edmaNum = CONFIG_EDMA2;
            break;
        }
        /* DSS TPTC C3 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *queNum = 3U;
            *edmaNum = CONFIG_EDMA2;
            break;
        }
        /* DSS TPTC C4 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *queNum = 4U;
            *edmaNum = CONFIG_EDMA2;
            break;
        }
        /* DSS TPTC C5 RD */
        case SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD :
        {
            *dmaCh = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_C_EVT_HWA_DMA_REQ0;
            *queNum = 5U;
            *edmaNum = CONFIG_EDMA2;
            break;
        }
        /* DSS PCR */
        case SDL_ECC_BUS_SAFETY_DSS_PCR :
        {
            *dmaCh = EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *tcc =   EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *param = EDMA_DSS_TPCC_A_EVT_HWA_DMA_REQ0;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA2;
            break;
        }
        default :
        {
            *dmaCh = EDMA_RESOURCE_ALLOC_ANY;
            *tcc =   EDMA_RESOURCE_ALLOC_ANY;
            *param = EDMA_RESOURCE_ALLOC_ANY;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA0;
            break;
        }
    }
}

#endif
#endif

#if defined (SUBSYS_MSS)
#if defined (SOC_AM273X) ||  (SOC_AWR294X)
/********************************************************************************************************
*   For Node MSS_TPTC_A0_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_MSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TPTC_A1_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_MSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TPTC_B0_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_MSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TPTC_A0_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD ,0U ));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD ,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_MSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TPTC_A1_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD ,0U ));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD ,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_MSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TPTC_B0_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD ,0U ));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD ,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_MSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5A_AXI_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD ,0U ));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD ,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_MSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
#endif
#if defined (SOC_AM263X)
/********************************************************************************************************
*   For Node MSS_TPTC_A0_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_MSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TPTC_A1_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_MSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TPTC_A0_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD ,0U ));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD ,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_MSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TPTC_A1_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD ,0U ));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_Test_Polling(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD ,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_Test_Polling(void)
{
    return ( SDL_ECC_BUS_SAFETY_MSS_RED_Test_Polling(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,\
             SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

#endif
#endif

#if defined (SUBSYS_MSS)
#if defined (SOC_AM273X) || defined (SOC_AWR294X) || defined (SOC_AM263X)
/********************************************************************************************************
*   For MSS SEC
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_Test_Polling(uint32_t busSftyNode ,uint32_t addr )
{
    int32_t retval = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t writeData = 0x1234567U;
    uint32_t status = 0U;
    uint32_t dmaCh = 0U;
    uint32_t tcc = 0U;
    uint32_t param = 0U;
    uint32_t queNum = 0U;
    uint32_t edmaNum = 0U;
    retval = SDL_ECC_BUS_SAFETY_MSS_secExecute(busSftyNode,addr,writeData);
    if(retval !=SDL_PASS )
    {
        retval = SDL_EFAIL;
    }
    else
    {
        if((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR>=busSftyNode))
        {
            SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
        }
        else
        {
             /* Do Nothing */
        }
        /* Wait for test to complete/timeout. */
        while((status == 0U) && (timeout!=0U))
        {
            retval =SDL_ECC_BUS_SAFETY_MSS_getSecErrorStatus(busSftyNode,&status);
            if(retval !=SDL_PASS )
            {
                timeout--;
            }
            else
            {
                break;
            }

        }
        if(status!=0U)
        {
            retval = SDL_PASS;
            SDL_ECC_BUS_SAFETY_MSS_secErrorClear(busSftyNode );
        }
        else
        {
            retval = SDL_EFAIL;
        }
    }
    return retval;
}

/********************************************************************************************************
*   For MSS DED
*********************************************************************************************************/

static int32_t SDL_ECC_BUS_SAFETY_MSS_DED_Test_Polling(uint32_t busSftyNode ,uint32_t addr)
{
    int32_t retval = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t writeData = 0x1234567U;
    uint32_t status = 0U;
    uint32_t dmaCh = 0U;
    uint32_t tcc = 0U;
    uint32_t param = 0U;
    uint32_t queNum = 0U;
    uint32_t edmaNum = 0U;

    retval = SDL_ECC_BUS_SAFETY_MSS_dedExecute(busSftyNode,addr,writeData);
    if(retval !=SDL_PASS )
    {
        retval = SDL_EFAIL;
    }
    else
    {
        if((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR>=busSftyNode))
        {
            SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
            test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
        }
        else
        {
            /* Do Nothing */
        }
        /* Wait for test to complete/timeout. */
        while((status == 0U) && (timeout!=0U))
        {
            retval =SDL_ECC_BUS_SAFETY_MSS_getDedErrorStatus(busSftyNode,&status);
            if(retval !=SDL_PASS )
            {
                timeout--;
            }
            else
            {
                break;
            }

        }
        if(status!=0U)
        {
            retval = SDL_PASS;
            SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(busSftyNode);
        }
        else
        {
            retval = SDL_EFAIL;
        }
    }
    return retval;
}

/********************************************************************************************************
*   For MSS RED
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_MSS_RED_Test_Polling(uint32_t busSftyNode, SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType)
{
    int32_t retval = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t status = 0U;
    retval= SDL_ECC_BUS_SAFETY_MSS_redExecute(busSftyNode, fiType, redType);
    if(retval !=SDL_PASS )
    {
        retval = SDL_EFAIL;
    }
    else
    {
        /* Wait for test to complete/timeout. */
        while((status == 0U) && (timeout!=0U))
        {
            retval =SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(busSftyNode,&status);
            if(retval !=SDL_PASS )
            {
                timeout--;
            }
            else
            {
                break;
            }

        }
        if(status!=0U)
        {
            retval = SDL_PASS;
            SDL_ECC_BUS_SAFETY_MSS_redErrorClear(busSftyNode);
        }
        else
        {
            retval = SDL_EFAIL;
        }
    }
    return retval;
}

/********************************************************************************************************
*   Function to select the parameters for EDMA config according to Nodes
*********************************************************************************************************/

void SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(uint32_t busSftyNode , uint32_t *dmaCh, uint32_t *tcc,uint32_t *param, uint32_t *queNum, uint32_t *edmaNum )
{
    switch (busSftyNode)
    {
      #if defined (SOC_AM273X) || defined (SOC_AWR294X)
        /* MSS TPTC A0 RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD :
        {
            *dmaCh = EDMA_MSS_TPCC_A_EVT_FREE_0;
            *tcc =   EDMA_MSS_TPCC_A_EVT_FREE_0;
            *param = EDMA_MSS_TPCC_A_EVT_FREE_0;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA0;
            break;
        }
        /* MSS TPTC A1 RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD :
        {
            *dmaCh = EDMA_MSS_TPCC_A_EVT_FREE_0;
            *tcc =   EDMA_MSS_TPCC_A_EVT_FREE_0;
            *param = EDMA_MSS_TPCC_A_EVT_FREE_0;
            *queNum = 1U;
            *edmaNum = CONFIG_EDMA0;
            break;
        }
        /* MSS TPTC B0 RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD :
        {
            *dmaCh = EDMA_MSS_TPCC_B_EVT_FREE_0;
            *tcc =   EDMA_MSS_TPCC_B_EVT_FREE_0;
            *param = EDMA_MSS_TPCC_B_EVT_FREE_0;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA1;
            break;
        }

        #endif
        #if defined (SOC_AM263X)
        /* MSS TPTC A0 RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD :
        {
            *dmaCh = EDMA_RESOURCE_ALLOC_ANY;
            *tcc =   EDMA_RESOURCE_ALLOC_ANY;
            *param = EDMA_RESOURCE_ALLOC_ANY;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA0;
            break;
        }
        /* MSS TPTC A1 RD */
        case SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD :
        {
            *dmaCh = EDMA_RESOURCE_ALLOC_ANY;
            *tcc =   EDMA_RESOURCE_ALLOC_ANY;
            *param = EDMA_RESOURCE_ALLOC_ANY;
            *queNum = 1U;
            *edmaNum = CONFIG_EDMA0;
            break;
        }
        #endif
        default :
        {
            *dmaCh = EDMA_RESOURCE_ALLOC_ANY;
            *tcc =   EDMA_RESOURCE_ALLOC_ANY;
            *param = EDMA_RESOURCE_ALLOC_ANY;
            *queNum = 0U;
            *edmaNum = CONFIG_EDMA0;
            break;
        }
    }
}

#endif
#endif


/* Nothing past this point */
