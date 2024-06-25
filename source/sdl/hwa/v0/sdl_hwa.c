/*
 *   Copyright (c) 2022-23 Texas Instruments Incorporated
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
 *  \file     sdl_hwa.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of hwa.
 *            This also contains some related macros.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include "sdl_hwa_hw.h"
#include "sdl_hwa.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/* ========================================================================== */
/*                   Internal Global Variables                                */
/* ========================================================================== */
bool dssHwaIntrFlag = TRUE;

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
/********************************************************************************************************
* CallBack Interrupt function
*********************************************************************************************************/
int32_t SDL_HWA_ESM_CallbackFunction(SDL_ESM_Inst instance, int32_t grpChannel,
                                     int32_t vecNum, void *arg)
{
    int32_t retVal = SDL_PASS;
    dssHwaIntrFlag = TRUE;
    return retVal;
}
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/********************************************************************************************************
* API to configuring and test parity error on HWA memory
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3883
 */
int32_t SDL_HWA_memParityExecute( SDL_HWA_MemID memID , SDL_HWA_MemBlock memBlock)
{
    uint32_t timeout = SDL_HWA_INIT_TIMEOUT;
    int32_t retSts = SDL_EFAIL;
    uint8_t retMemInitSts = 0U;
    uint8_t memBlockParity = 0U;
    volatile uint32_t writeData = 0x12345678U;
    uint32_t baseAddr;
    /* validate the memory block */
    if(SDL_HWA_getMemblockBaseaddr(memID,memBlock,
                                 &baseAddr) == SDL_PASS )
    {
        /* Select parity operation for Window RAM */
        if(memBlock == SDL_HWA_WINDOW_RAM )
        {
            memBlockParity = SDL_HWA_DMEM_WINDOW_RAM_PARITY;
        }
        /* Select parity operation for DMEM */
        else
        {
            memBlockParity = SDL_HWA_DMEM_PARITY;
        }
        /* Initialize the memory */
        SDL_HWA_memoryInitStart(memBlock);
        /* wait for memory initialization */
        do
        {
            timeout--;
        }
        while(((uint8_t)1U==SDL_HWA_memoryInitStatus(memBlock))&&(timeout>(uint32_t)0U))
        ;
        timeout = SDL_HWA_INIT_TIMEOUT;
        /* Memory init done Status */
        retMemInitSts = SDL_HWA_memoryInitDone(memBlock);
        if(1U == retMemInitSts)
        {
            /* Parity unmask */
            SDL_HWA_setParityErrMaskUnmask(memBlock, SDL_HWA_DISABLE_STS);
            if((uint8_t)0U == SDL_HWA_getParityErrMaskUnmask(memBlock))
            {
                /* Enable safety */
                SDL_HWA_setParityEnableDisable(memBlockParity,SDL_HWA_ENABLE_STS);
                if((uint8_t)1U == SDL_HWA_getParityStatus(memBlockParity))
                {
                    HW_WR_REG32(baseAddr, writeData);
                    SDL_HWA_setParityEnableDisable(memBlockParity,SDL_HWA_DISABLE_STS);
                    if((uint8_t)0U == SDL_HWA_getParityStatus(memBlockParity))
                    {
                        while(timeout!=0U)
                        {
                            timeout--;
                        }
                        timeout = SDL_HWA_INIT_TIMEOUT;
                        writeData = (writeData^((uint32_t)0x1));
                        HW_WR_REG32(baseAddr, writeData);
                        SDL_HWA_setParityEnableDisable(memBlockParity,SDL_HWA_ENABLE_STS);
                        if((uint8_t)1U == SDL_HWA_getParityStatus(memBlockParity))
                        {
                            writeData = HW_RD_REG32(baseAddr);
                            /* wait for error notification from ESM, or for timeout */
                            /* fault injection gets deasserted in ISR */
                            while((dssHwaIntrFlag!=TRUE) && (timeout!=(uint32_t)0U))
                            {
                                timeout--;
                            }
                            if(SDL_HWA_getErrStatus(memBlock)==1U)
                            {
                                SDL_HWA_clearErrStatus(memBlock);
                                dssHwaIntrFlag =FALSE;
                                retSts = SDL_PASS;
                            }
                            else
                            {
                                retSts = SDL_EFAIL;
                            }

                        }
                        else
                        {
                            retSts = SDL_EFAIL;
                        }

                    }
                    else
                    {
                        retSts = SDL_EFAIL;
                    }
                }
                else
                {
                    retSts = SDL_EFAIL;
                }
            }
            else
            {
                retSts = SDL_EFAIL;
            }
        }
        else
        {
            retSts = SDL_EFAIL;
        }
    }
    else
    {
        /* Invalid input parameter */
        retSts = SDL_EBADARGS;
    }
    return (retSts);
}
/********************************************************************************************************
* API to induce the error in the fsm lockstep for HWA
*********************************************************************************************************/
/**
 *  Design: PROC_SDL-3884
 */
int32_t SDL_HWA_fsmLockStepExecute( void)
{
    int32_t retSts = SDL_EBADARGS;
    SDL_HWA_Status_s enSts;
    uint32_t timeout = SDL_HWA_INIT_TIMEOUT;
    uint8_t memBlockParity=0U;
    SDL_HWA_MemBlock memBlock = SDL_HWA_FSM_LOCKSTEP;
    /* Enable HWA for FSM*/
    SDL_HWA_setHwaEnableDisable(SDL_HWA_ENABLE_STS);
    /* Get the FSM enable Status */
    retSts=SDL_HWA_getHwaEnableDisable(&enSts);
    if (SDL_PASS ==retSts)
    {
        if((enSts.SDL_HWA_enableHwaSTS == SDL_HWA_ENABLE_HWA_EN_ENABLE) &&
             (enSts.SDL_HWA_enableHwaClkSTS == SDL_HWA_ENABLE_HWA_CLK_EN_ENABLE))
        {
            /*Enable Parity*/
            SDL_HWA_setParityErrMaskUnmask(memBlock, SDL_HWA_DISABLE_STS);
            if((uint8_t)0U == SDL_HWA_getParityErrMaskUnmask(memBlock))
            {
                memBlockParity = SDL_HWA_DMEM_FSM_LOCKSTEP_EN;
                /* enable FSM safety */
                SDL_HWA_setParityEnableDisable(memBlockParity,SDL_HWA_ENABLE_STS);
                if((uint8_t)1U == SDL_HWA_getParityStatus(memBlockParity))
                {
                    memBlockParity = SDL_HWA_DMEM_FSM_LOCKSTEP_INV_EN;
                    /*Enable FSM safety*/
                    SDL_HWA_setParityEnableDisable(memBlockParity,SDL_HWA_ENABLE_STS);
                    if((uint8_t)0U == SDL_HWA_getParityStatus(memBlockParity))
                    {
                        /* wait for error notification from ESM, or for timeout */
                        /* fault injection gets deasserted in ISR */
                        while((dssHwaIntrFlag!=TRUE) && (timeout!=(uint32_t)0U))
                        {
                            timeout--;
                        }
                        if(SDL_HWA_getErrStatus(memBlock)==1U)
                        {
                            SDL_HWA_clearErrStatus(memBlock);
                            dssHwaIntrFlag =FALSE;
                            retSts = SDL_PASS;
                        }
                        else
                        {
                            retSts = SDL_EFAIL;
                        }
                    }
                    else
                    {
                        retSts = SDL_EFAIL;
                    }
                }
                else
                {
                    retSts = SDL_EFAIL;
                }
            }
            else
            {
                retSts = SDL_EFAIL;
            }
        }
        else
        {
            /* Invalid input parameter */
            retSts = SDL_EBADARGS;
        }
    }
    else
    {
        retSts = SDL_EFAIL;
    }
    return (retSts);
}
