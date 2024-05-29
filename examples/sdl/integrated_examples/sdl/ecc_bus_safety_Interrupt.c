
/* Copyright (c) 2022-2024 Texas Instruments Incorporated
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
 *  \file     ecc_bus_safety_Interrupt.c
 *
 *  \brief    This file contains Ecc Bus Safety for all the nodes in MSS and DSS example code in Interrupt method.
 *
 *  \details  ECC_BUS_SAFETY APP
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "ecc_bus_safety.h"
#include "sdlexample.h"

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/


/* generalized helper function for red test  */
static int32_t SDL_ECC_BUS_SAFETY_MSS_RED_test(const SDL_ESM_Inst esmInstType, uint32_t busSftyNode, SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType);
/* generalized helper function for ded test  */
static int32_t SDL_ECC_BUS_SAFETY_MSS_DED_test(const SDL_ESM_Inst esmInstType, uint32_t busSftyNode, uint32_t addr);
/* generalized helper function for sec test */
static int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_test(const SDL_ESM_Inst esmInstType, uint32_t busSftyNode, uint32_t addr);


/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/

volatile bool mssSecFlag = FALSE;
volatile bool SDL_MSS_intrFlg[SDL_ECC_BUS_SAFETY_MSS_STM_STIM+1U];

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/

/********************************************************************************************************
*   ESM Callback Function
*********************************************************************************************************/


/* Set by the diagnsotics test code so we know how to clear */
volatile static uint32_t  currentBusSftyNode; /* current bus safety node being tested, else undefined */
volatile static uint32_t  busTestType;        /* 1 = SEC, 2 = DED, 3 = RED */

int32_t EccBusSafety_clearESM(void)
{
  int32_t   retVal = SDL_PASS;


  /* if we are not running diagnostics, clear and return. main task will handle. */
  if (runningDiags() != true)
  {
    /* Clear ESM registers. */
    SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, sdlstats.esm.intSrc);
    return retVal;
  }

  /* handle diagnostic esm callbacks */

  /* sanity check bus safety node */
  if (currentBusSftyNode > SDL_ECC_BUS_SAFETY_MSS_STM_STIM+1U)
  {
    /* Clear ESM registers - diags will fail */
    SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, sdlstats.esm.intSrc);
    return retVal;
  }

  if (busTestType == 1)
  {
    /* SEC */
    /*  MSS_TPTC_A0_RD */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD );
    /*  MSS_TPTC_A1_RD */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD );
    /*  MSS_MBOX */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_MBOX);
    /*  MSS_MMC */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_MMC);
    /*  MSS_GPMC */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_GPMC);
    /* MSS_L2_A */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_A);
    /* MSS_L2_B */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_B);
    /* MSS_L2_C */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_C);
    /* MSS_L2_D */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_L2_D);
    /* MSS_CPSW */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_CPSW);
    /* MSS_MCRC */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_MSS_MCRC);
    /* ICSSM_PDSP0 */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_ICSSM_PDSP0);
    /* ICSSM_PDSP1 */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_ICSSM_PDSP1);
    /* ICSSM_S */
    SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_ICSSM_S);
    mssSecFlag = TRUE;
  }
  else
  if (busTestType == 2)
  {
    /* DED */
    if((SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(currentBusSftyNode)) && \
       (SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(currentBusSftyNode)))
    {
        SDL_MSS_intrFlg[currentBusSftyNode] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[currentBusSftyNode] = FALSE;
    }
  }
  else
  if (busTestType == 3)
  {
    /* RED */
    if(SDL_PASS == SDL_ECC_BUS_SAFETY_MSS_redErrorClear(currentBusSftyNode))
    {
        SDL_MSS_intrFlg[currentBusSftyNode] = TRUE;
    }
    else
    {
        SDL_MSS_intrFlg[currentBusSftyNode] = FALSE;
    }
  }

  /* Clear ESM registers. */
  SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, sdlstats.esm.intSrc);

  return retVal;

}


/********************************************************************************************************
*   For Node MSS_L2_A
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_A,SDL_MPU_L2OCRAM_BANK0));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_A,SDL_MPU_L2OCRAM_BANK0));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_A_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_L2_A, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_L2_B
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_B,SDL_MPU_L2OCRAM_BANK1));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_B,SDL_MPU_L2OCRAM_BANK1));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_B_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_L2_B, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_L2_C
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_C_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_C,SDL_MPU_L2OCRAM_BANK2));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_C_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_C,SDL_MPU_L2OCRAM_BANK2));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_C_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_L2_C, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_L2_D
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_L2_D_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_D,SDL_MPU_L2OCRAM_BANK3));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_D_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_L2_D,SDL_MPU_L2OCRAM_BANK3));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_L2_D_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_L2_D, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_MMC_S
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_MMC_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_MMC,SDL_MMC0_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MMC_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_MMC,SDL_MMC0_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MMC_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_MMC, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CPSW
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_CPSW,0u));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_CPSW,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CPSW_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_CPSW, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}


/********************************************************************************************************
*   For Node MSS_GPMC
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_GPMC_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_GPMC,SDL_GPMC0_CFG_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_GPMC_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_GPMC,SDL_GPMC0_CFG_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_GPMC_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_GPMC, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_MAIN_VBUSP
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_MAIN_VBUSP
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_CR5A_AHB_RED_TEST
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB_RED_Test()
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_CR5B_AHB_RED_TEST
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB_RED_Test()
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_CR5C_AHB_RED_TEST
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB_RED_Test()
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0,SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_CR5D_AHB_RED_TEST
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB_RED_Test()
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TPTC_A0_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_TPTC_A1_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,0U));
}
int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_TPTC_A0_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}
/********************************************************************************************************
*   For Node MSS_TPTC_A1_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5A_AXI_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5B_AXI_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5C_AXI_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5D_AXI_WR
*********************************************************************************************************/

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5A_AXI_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5B_AXI_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5C_AXI_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5D_AXI_RD
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5A_AXI_S
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
            SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5B_AXI_S
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5C_AXI_S
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_CR5D_AXI_S
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,\
                                        SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_MCRC
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_SEC_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_MCRC,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_DED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_MCRC,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MCRC_RED_Test(void)
{
    return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_MCRC, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_QSPI
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_SEC_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_QSPI,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_DED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_QSPI,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_QSPI_RED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
          SDL_ECC_BUS_SAFETY_MSS_QSPI, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_MBOX
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_SEC_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_MBOX_SRAM_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_DED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_MBOX_SRAM_U_BASE));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_MBOX_RED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_MBOX, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_STM_STIM
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_STM_STIM_SEC_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_STM_STIM,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_STM_STIM_DED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_STM_STIM,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_STM_STIM_RED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_STM_STIM, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_SCRP0
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP0_SEC_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_SCRP0,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP0_DED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_SCRP0,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP0_RED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_SCRP0, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node MSS_SCRP1
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP1_SEC_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_SCRP1,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP1_DED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_MSS_SCRP1,0U));
}

int32_t SDL_ECC_BUS_SAFETY_MSS_SCRP1_RED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
          SDL_ECC_BUS_SAFETY_MSS_SCRP1, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node ICSSM_PDSP0
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_SEC_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_ICSSM_PDSP0,0U));
}

int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_DED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_ICSSM_PDSP0,0U));
}

int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP0_RED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
          SDL_ECC_BUS_SAFETY_ICSSM_PDSP0, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node ICSSM_PDSP1
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_SEC_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_ICSSM_PDSP1,0U));
}

int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_DED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0, SDL_ECC_BUS_SAFETY_ICSSM_PDSP1,0U));
}

int32_t SDL_ECC_BUS_SAFETY_ICSSM_PDSP1_RED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
          SDL_ECC_BUS_SAFETY_ICSSM_PDSP1, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node ICSSM_SLAVE
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_ICSSM_S_SEC_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,\
                                    SDL_ECC_BUS_SAFETY_ICSSM_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_ICSSM_S_DED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,\
                                      SDL_ECC_BUS_SAFETY_ICSSM_S,0U));
}

int32_t SDL_ECC_BUS_SAFETY_ICSSM_S_RED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
          SDL_ECC_BUS_SAFETY_ICSSM_S, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}

/********************************************************************************************************
*   For Node DAP
*********************************************************************************************************/
int32_t SDL_ECC_BUS_SAFETY_DAP_SEC_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_SEC_test(SDL_ESM_INST_MAIN_ESM0,\
                                    SDL_ECC_BUS_SAFETY_DAP,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DAP_DED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_DED_test(SDL_ESM_INST_MAIN_ESM0,\
                                      SDL_ECC_BUS_SAFETY_DAP,0U));
}

int32_t SDL_ECC_BUS_SAFETY_DAP_RED_Test(void)
{
  return (SDL_ECC_BUS_SAFETY_MSS_RED_test(SDL_ESM_INST_MAIN_ESM0, \
          SDL_ECC_BUS_SAFETY_DAP, SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE));
}



/********************************************************************************************************
*   For SEC
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_MSS_SEC_test(const SDL_ESM_Inst esmInstType, \
                                                                      uint32_t busSftyNode, uint32_t addr)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t writeData = 0x1234567U;
    uint32_t dmaCh = 0U;
    uint32_t tcc = 0U;
    uint32_t param = 0U;
    uint32_t queNum = 0U;
    uint32_t edmaNum = 0U;


    /* save off bus node for the ESM call back */
    currentBusSftyNode = busSftyNode;
    busTestType = 1;

    /* re-enable our interrupt, its disabled in the ESM callback */
    SDL_ESM_enableIntr(SDL_TOP_ESM_U_BASE, ESM_INT_BUSSAFETY1);
    SDL_ESM_enableIntr(SDL_TOP_ESM_U_BASE, ESM_INT_BUSSAFETY2);

        ret_val = SDL_ECC_BUS_SAFETY_MSS_secExecute(busSftyNode,addr,writeData);
        if(ret_val !=SDL_PASS )
        {
            ret_val = SDL_EFAIL;
        }
        else
        {
            if((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD>=busSftyNode))
            {
                SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
                test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
            }
            /* do nothing for others: SDL_ECC_BUS_SAFETY_MSS_CPSW, etc */

            /* wait for error notification from ESM, or for timeout */
            /* fault injection gets deasserted in ISR */
            while((mssSecFlag!=TRUE) && (timeout!=0U))
            {
                timeout--;
            }
            if(mssSecFlag==TRUE)
            {
                mssSecFlag=FALSE;
                ret_val = SDL_PASS;
            }
            else
            {
                ret_val = SDL_EFAIL;
            }
        }



    return ret_val;
}

/********************************************************************************************************
*   For DED
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_MSS_DED_test(const SDL_ESM_Inst esmInstType,\
                                                                     uint32_t busSftyNode, uint32_t addr)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;
    uint32_t writeData = 0x1234567U;
    uint32_t dmaCh = 0U;
    uint32_t tcc = 0U;
    uint32_t param = 0U;
    uint32_t queNum = 0U;
    uint32_t edmaNum = 0U;

    /* save off bus node for the ESM call back */
    currentBusSftyNode = busSftyNode;
    busTestType = 2;

    /* re-enable our interrupot, its disabled in the ESM callback */
    SDL_ESM_enableIntr(SDL_TOP_ESM_U_BASE, ESM_INT_BUSSAFETY1);
    SDL_ESM_enableIntr(SDL_TOP_ESM_U_BASE, ESM_INT_BUSSAFETY2);

    ret_val = SDL_ECC_BUS_SAFETY_MSS_dedExecute(busSftyNode,addr,writeData);

        if(ret_val !=SDL_PASS )
        {
           ret_val = SDL_EFAIL;
        }
        else
        {
            if((SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD <= busSftyNode )&&(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD>=busSftyNode))
            {
                SDL_ECC_BUS_SAFETY_MSS_getEDMAParameters(busSftyNode,&dmaCh, &tcc, &param, &queNum, &edmaNum );
                test_edmaATransfer(busSftyNode,dmaCh,tcc,param,queNum,edmaNum );
            }
            /* do nothing for others: SDL_ECC_BUS_SAFETY_MSS_CPSW, etc */

            /* wait for error notification from ESM, or for timeout */
            /* fault injection gets deasserted in ISR */
            while((SDL_MSS_intrFlg[busSftyNode]!=TRUE) && (timeout!=0U))
            {
                timeout--;
            }
            if(SDL_MSS_intrFlg[busSftyNode]==TRUE)
            {
                SDL_MSS_intrFlg[busSftyNode]=FALSE;
                ret_val = SDL_PASS;
            }
            else
            {
                ret_val = SDL_EFAIL;
            }
        }


    return ret_val;
}
/********************************************************************************************************
*   For RED
*********************************************************************************************************/
static int32_t SDL_ECC_BUS_SAFETY_MSS_RED_test(const SDL_ESM_Inst esmInstType,\
            uint32_t busSftyNode, SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ECC_BUS_SAFETY_TIMEOUT;

    /* save off bus node for the ESM call back */
    currentBusSftyNode = busSftyNode;
    busTestType = 3;

    /* re-enable our interrupot, its disabled in the ESM callback */
    SDL_ESM_enableIntr(SDL_TOP_ESM_U_BASE, ESM_INT_BUSSAFETY1);
    SDL_ESM_enableIntr(SDL_TOP_ESM_U_BASE, ESM_INT_BUSSAFETY2);

        ret_val= SDL_ECC_BUS_SAFETY_MSS_redExecute(busSftyNode, fiType, redType);
        if(ret_val !=SDL_PASS )
        {
            ret_val = SDL_EFAIL;
        }
        else
        {
            /* wait for error notification from ESM, or for timeout */
            /* fault injection gets deasserted in ISR */
            while((SDL_MSS_intrFlg[busSftyNode]!=TRUE) && (timeout!=0U))
            {
                timeout--;
            }
            if(SDL_MSS_intrFlg[busSftyNode]==TRUE)
            {
                SDL_MSS_intrFlg[busSftyNode]=FALSE;
                ret_val = SDL_PASS;
            }
            else
            {
                ret_val = SDL_EFAIL;
            }
        }

    return ret_val;
}


/* Nothing past this point */
