/*
 *   Copyright (c) Texas Instruments Incorporated 2022
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
 *  \file     sdl_stc_soc.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of STC.
 *            This also contains some related macros and Helper APIs.
 */

 /**
 * Design: PROC_SDL-1021
 */


#include <sdl/stc/v0/sdl_stc.h>



/********************************************************************************************************
*   API for getting the status of specified STC instance
*********************************************************************************************************/


int32_t SDL_STC_getStatus(SDL_STC_Inst instance)
{
    uint32_t  baseAddr;
    uint32_t stcResultDone, stcResultFail, stcResultActive;
    uint32_t stcReset=0U;
    int32_t stcResult= (int32_t)INVALID_RESULT;

    if (instance < SDL_STC_INVALID_INSTANCE)
    {
        if(instance == SDL_STC_INST_MAINR5F0)
        {
            stcReset= (uint32_t)HW_RD_FIELD32(SDL_MSS_RCM_U_BASE + SDL_MSS_RCM_R5SS0_RST_STATUS, SDL_MSS_STC_RESET);
        }
        else
        {
            stcReset= (uint32_t)HW_RD_FIELD32(SDL_MSS_RCM_U_BASE + SDL_MSS_RCM_R5SS1_RST_STATUS, SDL_MSS_STC_RESET);
        }
            /* Getting base address */
        baseAddr = SDL_STC_baseAddress[instance];

        stcResultDone  =   (uint32_t)HW_RD_FIELD32(baseAddr + SDL_STC_STCGSTAT, SDL_STC_TEST_DONE);
        stcResultFail  =   (uint32_t)HW_RD_FIELD32(baseAddr + SDL_STC_STCGSTAT, SDL_STC_TEST_FAIL);
        stcResultActive=   (uint32_t)HW_RD_FIELD32(baseAddr + SDL_STC_STCGSTAT, SDL_STC_ST_ACTIVE);

        if(stcReset==(1U))
        {
            if((stcResultDone==(1U))&&(stcResultFail==(1U)))
            {
                stcResult= (int32_t)SDL_STC_COMPLETED_FAILURE;
            }

            if ((stcResultDone==(1U))&&(stcResultFail==(0U)))
            {
                stcResult=(int32_t) SDL_STC_COMPLETED_SUCCESS;
            }
        }
        else
        {
            if (stcResultActive== (SDL_STC_ST_ACTIVE_ENABLE))
            {
                stcResult= (int32_t)SDL_STC_NOT_COMPLETED;
            }
            else
            {
                stcResult= (int32_t)SDL_STC_NOT_RUN;
            }
        }
    }
    else
    {
        stcResult= (int32_t)INVALID_RESULT;
    }

    return stcResult;
}


/********************************************************************************************************
* Helper  API for Configuring instance specified STC instance
*********************************************************************************************************/

static int32_t SDL_STC_configure(SDL_STC_Inst instance, SDL_STC_Config *pConfig, SDL_STC_TestType testType)
{
    int32_t sdlResult = SDL_EFAIL;
    uint32_t baseAddr;
    if((instance  <SDL_STC_INVALID_INSTANCE) && (pConfig != NULL))
    {
        /* Getting base address */
        baseAddr = SDL_STC_baseAddress[instance];

        /* Configure number of intervales to be run for STC R5F */
            HW_WR_FIELD32(baseAddr + SDL_STC_STCGCR0, SDL_STC_STCGCR0_INTCOUNT_B16,
            pConfig->intervalNum);
        /* Configure LP SCAN mode for STC R5F */
            HW_WR_FIELD32(baseAddr + SDL_STC_STCGCR1, SDL_STC_LP_SCAN_MODE,
            pConfig->modeConfig.lpScanMode);
        /* Configure CODEC SPREAD mode for STC R5F */
            HW_WR_FIELD32(baseAddr + SDL_STC_STCGCR1, SDL_STC_CODEC_SPREAD_MODE,
            pConfig->modeConfig.codecSpreadMode);
        /* Configure CAP IDLE CYCLE for STC R5F */
            HW_WR_FIELD32(baseAddr + SDL_STC_STCGCR0, SDL_STC_STCGCR0_CAP_IDLE_CYCLE,
            pConfig->modeConfig.capIdleCycle);
        /* Configure SCANEN HIGH CAP IDLE CYCLE for STC R5F */
            HW_WR_FIELD32(baseAddr + SDL_STC_STCGCR0, SDL_STC_STCGCR0_SCANEN_HIGH_CAP_IDLE_CYCLE,
            pConfig->modeConfig.scanEnHighCap_idleCycle);
        /* Configure Max run time for STC R5F */
            HW_WR_FIELD32(baseAddr + SDL_STC_STCTPR, SDL_STC_TO_PRELOAD,
            pConfig->maxRunTime);
        /* Configure Clock division value for STC R5F */
        /* if value is n then clock will be divided by n+1 */
            HW_WR_FIELD32(baseAddr + SDL_STC_CLKDIV, SDL_STC_CLKDIV0,
            pConfig->clkDiv);
        /* Configure the STC ROM start address */
            HW_WR_FIELD32(baseAddr + SDL_STC_SEG0_START_ADDR, SDL_STC_SEG0_START_ADDR,
            pConfig->romStartAddress);
        /* Configure the pointer for STC ROM start address */
            HW_WR_FIELD32(baseAddr + SDL_STC_STCGCR0, SDL_STC_STCGCR0_RS_CNT_B1,
            pConfig->pRomStartAdd);

        if (testType == SDL_STC_TEST)
        {
            /*For this configuration STC should Pass successfully.*/
            /* Configure this value in The Register for positive STC performed */
            HW_WR_FIELD32(baseAddr + SDL_STC_STCSCSCR, SDL_STC_FAULT_INS_B1,
            SDL_STC_FAULT_INS_B1_DISABLE);
            /* Configure this value in The Register for positive STC performed */
            HW_WR_FIELD32(baseAddr + SDL_STC_STCSCSCR, SDL_STC_SELF_CHECK_KEY_B4,
            SDL_STC_SELF_CHECK_KEY_B4_DISABLE);

            sdlResult = SDL_PASS;
        }
        else
        {
            if (testType == SDL_STC_NEG_TEST )
            {
                /*For this configuration STC should fail successfully.*/
                /* Configure this value in The Register for Negative STC performed */
                HW_WR_FIELD32(baseAddr + SDL_STC_STCSCSCR, SDL_STC_FAULT_INS_B1,
                    SDL_STC_FAULT_INS_B1_ENABLE);
                /* Configure this value in The Register for Negative STC performed */
                HW_WR_FIELD32(baseAddr + SDL_STC_STCSCSCR, SDL_STC_SELF_CHECK_KEY_B4,
                    SDL_STC_SELF_CHECK_KEY_B4_ENABLE);

                sdlResult = SDL_PASS;
            }
            else
            {
                sdlResult = SDL_EBADARGS;
            }
        }
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }
   return sdlResult;

}

/********************************************************************************************************
* Helper  API for Configuring to enable test for  specified STC instance
*********************************************************************************************************/

static int32_t  SDL_STC_runTest(SDL_STC_Inst instance )
{
   int32_t sdlResult = SDL_EFAIL;
   uint32_t baseAddr;
   int32_t count =100;

    if(instance  <SDL_STC_INVALID_INSTANCE)
    {
        /* Getting base address */
        baseAddr = SDL_STC_baseAddress[instance];
        /* run asm( "nop") opration for delay*/
        (void) SDL_STC_delay(count);

        /* Configure this value to Enable the STC for specified Instance */
        HW_WR_FIELD32(baseAddr + SDL_STC_STCGCR1, SDL_STC_ST_ENA_B4,
            SDL_STC_ST_ENA_B4_ENABLE);

        (void)SDL_STC_delay(count);
        /* Configure this Register for R5F to be in low power mode (WFI)mode*/
        /* Provide override WFI signal to STC indicating processor idle state*/

        if(instance ==SDL_STC_INST_MAINR5F0 )
        {
            HW_WR_FIELD32(SDL_MSS_CTRL_U_BASE + SDL_MSS_CTRL_R5SS0_FORCE_WFI ,SDL_MSS_CTRL_R5SS0_FORCE_WFI_CR5_WFI_OVERIDE,
                    SDL_MSS_CTRL_R5SS0_FORCE_WFI_CR5_WFI_OVERIDE_MAX);
        }
        else
        {
            HW_WR_FIELD32(SDL_MSS_CTRL_U_BASE + SDL_MSS_CTRL_R5SS1_FORCE_WFI ,SDL_MSS_CTRL_R5SS1_FORCE_WFI_CR5_WFI_OVERIDE,
                    SDL_MSS_CTRL_R5SS1_FORCE_WFI_CR5_WFI_OVERIDE_MAX);
        }

        /* run asm( "nop") opration for delay*/
        (void)SDL_STC_delay(count);
        sdlResult = SDL_PASS;
    }
 else
    {
        sdlResult= SDL_EBADARGS;
    }
    return sdlResult;
}


/********************************************************************************************************
* Helper  API for Performing STC test for specified STC instance
*********************************************************************************************************/

static void SDL_STC_resetCauseClearR5F0(void)
{
    HW_WR_FIELD32(SDL_MSS_RCM_U_BASE + SDL_MSS_RCM_R5SS0_RST_CAUSE_CLR, SDL_MSS_STC_RESET_CLEAR, SDL_MSS_STC_RESET_CLEAR_ENABLE);
}
static void SDL_STC_resetCauseClearR5F1(void)
{
    HW_WR_FIELD32(SDL_MSS_RCM_U_BASE + SDL_MSS_RCM_R5SS1_RST_CAUSE_CLR, SDL_MSS_STC_RESET_CLEAR, SDL_MSS_STC_RESET_CLEAR_ENABLE);
}

/********************************************************************************************************
* Helper  API for Performing Delay for STC test for specified STC instance
*********************************************************************************************************/
static void SDL_STC_delay(int32_t count)
{
    int32_t countVal=count;

    while((countVal)>=(0))
    {
        SDL_Delay();
        countVal--;
    }
}

static void  __attribute__((noinline)) SDL_Delay(void)
{
    asm("	nop");
}

/********************************************************************************************************
*   API for Performing STC test for specified STC instance
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3442
 */
int32_t   SDL_STC_selfTest(SDL_STC_Inst instance, SDL_STC_TestType testType, SDL_STC_Config *pConfig )
{
    int32_t sdlResult= SDL_EFAIL;

    if(instance < SDL_STC_INVALID_INSTANCE)
    {
        if(instance==SDL_STC_INST_MAINR5F0)
        {
            SDL_STC_resetCauseClearR5F0();
        }
        else
        {
            SDL_STC_resetCauseClearR5F1();
        }
    }
    else
    {
        ;/* Do nothing */
    }

    if(pConfig != NULL)
    {
        sdlResult =  SDL_STC_configure(instance,pConfig,testType);

        if(sdlResult == SDL_PASS)
        {
            sdlResult =  SDL_STC_runTest(instance );
        }
    }
    else
    {
        sdlResult=SDL_EBADARGS;
    }
    return sdlResult;
}

/* Nothing past this point */
