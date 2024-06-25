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
 *  \file     sdl_ip_hwa.c
 *
 *  \brief    This file contains the implementation of the low level API's present in the
 *            device abstraction layer file of HWA.
 */

#include <stdint.h>
#include <stdbool.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include "sdl_ip_hwa.h"
#include "sdl_hwa_hw.h"
#include <sdl/hwa/v0/soc/sdl_hwa_soc.h>

/********************************************************************************************************
* API to enable or disable the parity for memory block and lockstep logic
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3885
 */
void SDL_HWA_setParityEnableDisable(uint8_t memBlockParity,uint8_t enableSts)
{
    if(SDL_HWA_ENABLE_STS == enableSts)
    {
         switch(memBlockParity)
         {
            case SDL_HWA_DMEM_PARITY :
                HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_EN),\
                               SDL_HWA_SAFETY_EN_CFG_DMEM_PARITY_EN,\
                               SDL_HWA_SAFETY_EN_CFG_DMEM_PARITY_EN_ENABLE);
            break;
            case SDL_HWA_DMEM_WINDOW_RAM_PARITY :
                HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_EN),\
                               SDL_HWA_SAFETY_EN_CFG_WINDOW_RAM_PARITY_EN, \
                               SDL_HWA_SAFETY_EN_CFG_WINDOW_RAM_PARITY_EN_ENABLE);
            break;
            case SDL_HWA_DMEM_FSM_LOCKSTEP_INV_EN:
                  HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_EN),\
                                 SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_INV_EN, \
                                 SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_INV_EN_ENABLE);
            break;
            case SDL_HWA_DMEM_FSM_LOCKSTEP_EN:
                HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_EN),\
                               SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_EN, \
                               SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_EN_ENABLE);
            break;
            default:
            /* do nothing */
            break;
        }
    }
    else
    {
        switch(memBlockParity)
        {
            case SDL_HWA_DMEM_PARITY :
                HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_EN),\
                               SDL_HWA_SAFETY_EN_CFG_DMEM_PARITY_EN, \
                               SDL_HWA_SAFETY_EN_CFG_DMEM_PARITY_EN_DISABLE);
            break;
            case SDL_HWA_DMEM_WINDOW_RAM_PARITY :
                HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_EN),\
                               SDL_HWA_SAFETY_EN_CFG_WINDOW_RAM_PARITY_EN, \
                               SDL_HWA_SAFETY_EN_CFG_WINDOW_RAM_PARITY_EN_DISABLE);
            break;
            case SDL_HWA_DMEM_FSM_LOCKSTEP_INV_EN:
                  HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_EN),\
                                 SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_INV_EN, \
                                 SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_INV_EN_DISABLE);
            break;
            case SDL_HWA_DMEM_FSM_LOCKSTEP_EN:
                HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_EN),\
                               SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_EN, \
                               SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_EN_DISABLE);
            break;
            default:
            /* do nothing */
            break;
        }
    }
}
/********************************************************************************************************
* API to to get status for parity of memory block
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3886
 */
uint8_t SDL_HWA_getParityStatus(uint8_t memBlockParity)
{
    uint8_t paritySts = 0U;
    switch(memBlockParity)
    {
        case SDL_HWA_DMEM_PARITY :
        paritySts= (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_EN),\
                                   SDL_HWA_SAFETY_EN_CFG_DMEM_PARITY_EN);
        break;
        case SDL_HWA_DMEM_WINDOW_RAM_PARITY :
        paritySts = (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_EN),\
                                   SDL_HWA_SAFETY_EN_CFG_WINDOW_RAM_PARITY_EN);
        break;
        case SDL_HWA_DMEM_FSM_LOCKSTEP_INV_EN:
        paritySts = (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_EN),\
                                   SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_INV_EN);
        break;
        case SDL_HWA_DMEM_FSM_LOCKSTEP_EN:
        paritySts = (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_EN),\
                                   SDL_HWA_SAFETY_EN_CFG_FSM_LOCKSTEP_EN);
        break;
        default:
            /* do nothing */
        break;
        }
    return paritySts;
}
/********************************************************************************************************
* API to enable/disable Radar Hardware Accelerator and respective clock
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3887
 */
void SDL_HWA_setHwaEnableDisable(uint8_t enableSts)
{
    if(SDL_HWA_ENABLE_STS == enableSts)
    {
        /*  enables the Radar Hardware Accelerator */
        HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_ENABLE),\
                       SDL_HWA_ENABLE_HWA_EN, \
                       SDL_HWA_ENABLE_HWA_EN_ENABLE);
        /* enable for the clock of the Radar Accelerator */
        HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_ENABLE),\
                       SDL_HWA_ENABLE_HWA_CLK_EN, \
                       SDL_HWA_ENABLE_HWA_CLK_EN_ENABLE);
    }
    else
    {
        /*  enables the Radar Hardware Accelerator */
        HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_ENABLE),\
                       SDL_HWA_ENABLE_HWA_EN, \
                       SDL_HWA_ENABLE_HWA_EN_DISABLE);
        /* enable for the clock of the Radar Accelerator */
        HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_ENABLE),\
                       SDL_HWA_ENABLE_HWA_CLK_EN, \
                       SDL_HWA_ENABLE_HWA_CLK_EN_DISABLE);
    }
}

/********************************************************************************************************
* API to get Radar Hardware Accelerator and respective clock
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3888
 */

int32_t SDL_HWA_getHwaEnableDisable(SDL_HWA_Status_s *pHWAStats)
{
    int32_t status = SDL_PASS;
    if (pHWAStats == NULL_PTR)
    {
        status = SDL_EBADARGS;

    }
    else
    {
        /*  enables/disable the Radar Hardware Accelerator */
        pHWAStats->SDL_HWA_enableHwaSTS =\
            (uint8_t) HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_ENABLE),\
                                    SDL_HWA_ENABLE_HWA_EN);

        /* enable/disable for the clock of the Radar Accelerator */
        pHWAStats->SDL_HWA_enableHwaClkSTS=\
             (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_ENABLE),\
                                    SDL_HWA_ENABLE_HWA_CLK_EN);
        status = SDL_PASS;
    }
    return status;
}

/********************************************************************************************************
* API to initialize the HWA memory
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3889
 */
void SDL_HWA_memoryInitStart(SDL_HWA_MemBlock memBlock)
{
    switch(memBlock)
    {
        /* Window RAM */
        case SDL_HWA_WINDOW_RAM:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_START),\
                            SDL_HWA_MEM_INIT_START_WINDOW_RAM, \
                            SDL_HWA_MEM_INIT_START_WINDOW_RAM_ENABLE);
        break;
        /* DMEM7 */
        case SDL_HWA_DMEM7:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_START),\
                            SDL_HWA_MEM_INIT_START_DMEM7, \
                            SDL_HWA_MEM_INIT_START_DMEM7_ENABLE);
        break;
        /* DMEM6 */
        case SDL_HWA_DMEM6:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_START),\
                            SDL_HWA_MEM_INIT_START_DMEM6, \
                            SDL_HWA_MEM_INIT_START_DMEM6_ENABLE);
        break;
        /* DMEM 5 */
        case SDL_HWA_DMEM5:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_START),\
                            SDL_HWA_MEM_INIT_START_DMEM5, \
                            SDL_HWA_MEM_INIT_START_DMEM5_ENABLE);
        break;
        /* DMEM 4 */
        case SDL_HWA_DMEM4:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_START),\
                            SDL_HWA_MEM_INIT_START_DMEM4, \
                            SDL_HWA_MEM_INIT_START_DMEM4_ENABLE);
        break;
        /* DMEM 3 */
        case SDL_HWA_DMEM3:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_START),\
                            SDL_HWA_MEM_INIT_START_DMEM3, \
                            SDL_HWA_MEM_INIT_START_DMEM3_ENABLE);
        break;
        /* DMEM 2 */
        case SDL_HWA_DMEM2:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_START),\
                            SDL_HWA_MEM_INIT_START_DMEM2, \
                            SDL_HWA_MEM_INIT_START_DMEM2_ENABLE);
        break;
        /* DMEM 1 */
        case SDL_HWA_DMEM1:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_START),\
                            SDL_HWA_MEM_INIT_START_DMEM1, \
                            SDL_HWA_MEM_INIT_START_DMEM1_ENABLE);
        break;
        /* DMEM 0 */
        case SDL_HWA_DMEM0:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_START),\
                            SDL_HWA_MEM_INIT_START_DMEM0, \
                            SDL_HWA_MEM_INIT_START_DMEM0_ENABLE);
        break;
        default:
        break;
    }
}

/********************************************************************************************************
* API to check the HWA memory initialize done status
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3890
 */
uint8_t SDL_HWA_memoryInitDone(SDL_HWA_MemBlock memBlock)
{
    uint8_t status = 0U;
    switch(memBlock)
    {
        /* Window RAM */
        case SDL_HWA_WINDOW_RAM:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_DONE),\
                                     SDL_HWA_MEM_INIT_DONE_WINDOW_RAM);
        break;
        /* DMEM7 */
        case SDL_HWA_DMEM7:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_DONE),\
                                     SDL_HWA_MEM_INIT_DONE_DMEM7);
        break;
        /* DMEM6 */
        case SDL_HWA_DMEM6:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_DONE),\
                                     SDL_HWA_MEM_INIT_DONE_DMEM6);
        break;
        /* DMEM 5 */
        case SDL_HWA_DMEM5:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_DONE),\
                                     SDL_HWA_MEM_INIT_DONE_DMEM5);
        break;
        /* DMEM 4 */
        case SDL_HWA_DMEM4:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_DONE),\
                                     SDL_HWA_MEM_INIT_DONE_DMEM4);
        break;
        /* DMEM 3 */
        case SDL_HWA_DMEM3:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_DONE),\
                                     SDL_HWA_MEM_INIT_DONE_DMEM3);
        break;
        /* DMEM 2 */
        case SDL_HWA_DMEM2:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_DONE),\
                                     SDL_HWA_MEM_INIT_DONE_DMEM2);
        break;
        /* DMEM 1 */
        case SDL_HWA_DMEM1:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_DONE),\
                                     SDL_HWA_MEM_INIT_DONE_DMEM1);
        break;
        /* DMEM 0 */
        case SDL_HWA_DMEM0:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_DONE),\
                                     SDL_HWA_MEM_INIT_DONE_DMEM0);
        break;
        default:
        break;
    }
    return status;
}

/********************************************************************************************************
* API to check the HWA memory initialize  status
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3891
 */

uint8_t SDL_HWA_memoryInitStatus(SDL_HWA_MemBlock memBlock)
{
    uint8_t status = 0U;
    switch(memBlock)
    {
        /* Window RAM */
        case SDL_HWA_WINDOW_RAM:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_STATUS),\
                                     SDL_HWA_MEM_INIT_STATUS_WINDOW_RAM);
        break;
        /* DMEM7 */
        case SDL_HWA_DMEM7:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_STATUS),\
                                     SDL_HWA_MEM_INIT_STATUS_DMEM7);
        break;
        /* DMEM6 */
        case SDL_HWA_DMEM6:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_STATUS),\
                                     SDL_HWA_MEM_INIT_STATUS_DMEM6);
        break;
        /* DMEM 5 */
        case SDL_HWA_DMEM5:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_STATUS),\
                                     SDL_HWA_MEM_INIT_STATUS_DMEM5);
        break;
        /* DMEM 4 */
        case SDL_HWA_DMEM4:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_STATUS),\
                                     SDL_HWA_MEM_INIT_STATUS_DMEM4);
        break;
        /* DMEM 3 */
        case SDL_HWA_DMEM3:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_STATUS),\
                                     SDL_HWA_MEM_INIT_STATUS_DMEM3);
        break;
        /* DMEM 2 */
        case SDL_HWA_DMEM2:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_STATUS),\
                                     SDL_HWA_MEM_INIT_STATUS_DMEM2);
        break;
        /* DMEM 1 */
        case SDL_HWA_DMEM1:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_STATUS),\
                                     SDL_HWA_MEM_INIT_STATUS_DMEM1);
        break;
        /* DMEM 0 */
        case SDL_HWA_DMEM0:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_MEM_INIT_STATUS),\
                                     SDL_HWA_MEM_INIT_STATUS_DMEM0);
        break;
        default:
        break;
    }
    return status;
}
/********************************************************************************************************
* This API to mask/unmask parity error status
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3892
 */
void SDL_HWA_setParityErrMaskUnmask(SDL_HWA_MemBlock memBlock, uint8_t enableSts)
{
    if (SDL_HWA_ENABLE_STS == enableSts)
    {
        switch(memBlock)
        {
            /* Window RAM */
            case SDL_HWA_FSM_LOCKSTEP:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_FSM_LOCKSTEP, \
                            SDL_HWA_SAFETY_ERR_MASK_FSM_LOCKSTEP_ENABLE);
            break;
            /* Window RAM */
            case SDL_HWA_WINDOW_RAM:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_WINDOW_RAM, \
                            SDL_HWA_SAFETY_ERR_MASK_WINDOW_RAM_ENABLE);
            break;
            /* DMEM7 */
            case SDL_HWA_DMEM7:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_DMEM7, \
                            SDL_HWA_SAFETY_ERR_MASK_DMEM7_ENABLE);
            break;
            /* DMEM6 */
            case SDL_HWA_DMEM6:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_DMEM6, \
                            SDL_HWA_SAFETY_ERR_MASK_DMEM6_ENABLE);
            break;
            /* DMEM 5 */
            case SDL_HWA_DMEM5:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_DMEM5, \
                            SDL_HWA_SAFETY_ERR_MASK_DMEM5_ENABLE);
            break;
            /* DMEM 4 */
            case SDL_HWA_DMEM4:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_DMEM4, \
                            SDL_HWA_SAFETY_ERR_MASK_DMEM4_ENABLE);
            break;
            /* DMEM 3 */
            case SDL_HWA_DMEM3:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_DMEM3, \
                            SDL_HWA_SAFETY_ERR_MASK_DMEM3_ENABLE);
            break;
            /* DMEM 2 */
            case SDL_HWA_DMEM2:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_DMEM2, \
                            SDL_HWA_SAFETY_ERR_MASK_DMEM2_ENABLE);
            break;
            /* DMEM 1 */
            case SDL_HWA_DMEM1:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_DMEM1, \
                            SDL_HWA_SAFETY_ERR_MASK_DMEM1_ENABLE);
            break;
            /* DMEM 0 */
            case SDL_HWA_DMEM0:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_DMEM0, \
                            SDL_HWA_SAFETY_ERR_MASK_DMEM0_ENABLE);
            break;
            default:
            break;
        }
    }
    else
    {
        switch(memBlock)
        {
            /* Window RAM */
            case SDL_HWA_FSM_LOCKSTEP:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_FSM_LOCKSTEP, \
                            SDL_HWA_SAFETY_ERR_MASK_FSM_LOCKSTEP_DISABLE);
            break;
            /* Window RAM */
            case SDL_HWA_WINDOW_RAM:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_WINDOW_RAM, \
                            SDL_HWA_SAFETY_ERR_MASK_WINDOW_RAM_DISABLE);
            break;
            /* DMEM7 */
            case SDL_HWA_DMEM7:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_DMEM7, \
                            SDL_HWA_SAFETY_ERR_MASK_DMEM7_DISABLE);
            break;
            /* DMEM6 */
            case SDL_HWA_DMEM6:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_DMEM6, \
                            SDL_HWA_SAFETY_ERR_MASK_DMEM6_DISABLE);
            break;
            /* DMEM 5 */
            case SDL_HWA_DMEM5:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_DMEM5, \
                            SDL_HWA_SAFETY_ERR_MASK_DMEM5_DISABLE);
            break;
            /* DMEM 4 */
            case SDL_HWA_DMEM4:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_DMEM4, \
                            SDL_HWA_SAFETY_ERR_MASK_DMEM4_DISABLE);
            break;
            /* DMEM 3 */
            case SDL_HWA_DMEM3:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_DMEM3, \
                            SDL_HWA_SAFETY_ERR_MASK_DMEM3_DISABLE);
            break;
            /* DMEM 2 */
            case SDL_HWA_DMEM2:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_DMEM2, \
                            SDL_HWA_SAFETY_ERR_MASK_DMEM2_DISABLE);
            break;
            /* DMEM 1 */
            case SDL_HWA_DMEM1:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_DMEM1, \
                            SDL_HWA_SAFETY_ERR_MASK_DMEM1_DISABLE);
            break;
            /* DMEM 0 */
            case SDL_HWA_DMEM0:
             HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                            SDL_HWA_SAFETY_ERR_MASK_DMEM0, \
                            SDL_HWA_SAFETY_ERR_MASK_DMEM0_DISABLE);
            break;
            default:
            break;
        }
    }
}
/********************************************************************************************************
* API to get mask/unmask parity error status
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3893
 */
uint8_t SDL_HWA_getParityErrMaskUnmask(SDL_HWA_MemBlock memBlock)
{
    uint8_t status = 0U;
    switch(memBlock)
    {
        /* FSM Lockstep */
        case SDL_HWA_FSM_LOCKSTEP:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                                     SDL_HWA_SAFETY_ERR_MASK_FSM_LOCKSTEP);
        break;
        /* Window RAM */
        case SDL_HWA_WINDOW_RAM:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                                     SDL_HWA_SAFETY_ERR_MASK_WINDOW_RAM);
        break;
        /* DMEM7 */
        case SDL_HWA_DMEM7:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                                     SDL_HWA_SAFETY_ERR_MASK_DMEM7);
        break;
        /* DMEM6 */
        case SDL_HWA_DMEM6:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                                     SDL_HWA_SAFETY_ERR_MASK_DMEM6);
        break;
        /* DMEM 5 */
        case SDL_HWA_DMEM5:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                                     SDL_HWA_SAFETY_ERR_MASK_DMEM5);
        break;
        /* DMEM 4 */
        case SDL_HWA_DMEM4:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                                     SDL_HWA_SAFETY_ERR_MASK_DMEM4);
        break;
        /* DMEM 3 */
        case SDL_HWA_DMEM3:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                                     SDL_HWA_SAFETY_ERR_MASK_DMEM3);
        break;
        /* DMEM 2 */
        case SDL_HWA_DMEM2:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                                     SDL_HWA_SAFETY_ERR_MASK_DMEM2);
        break;
        /* DMEM 1 */
        case SDL_HWA_DMEM1:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                                     SDL_HWA_SAFETY_ERR_MASK_DMEM1);
        break;
        /* DMEM 0 */
        case SDL_HWA_DMEM0:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_MASK),\
                                     SDL_HWA_SAFETY_ERR_MASK_DMEM0);
        break;
        default:
        break;
    }
    return status;
}

/********************************************************************************************************
* API to get error status
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3894
 */

uint8_t SDL_HWA_getErrStatus(SDL_HWA_MemBlock memBlock)
{
    uint8_t status = 0U;
    SDL_HWA_setParityErrMaskUnmask(memBlock,SDL_HWA_DISABLE_STS);
    switch(memBlock)
    {
        /* FSM Lockstep */
        case SDL_HWA_FSM_LOCKSTEP:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_FSM_LOCKSTEP);
        break;
        /* Window RAM */
        case SDL_HWA_WINDOW_RAM:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_WINDOW_RAM);
        break;
        /* DMEM7 */
        case SDL_HWA_DMEM7:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM7);
        break;
        /* DMEM6 */
        case SDL_HWA_DMEM6:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM6);
        break;
        /* DMEM 5 */
        case SDL_HWA_DMEM5:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM5);
        break;
        /* DMEM 4 */
        case SDL_HWA_DMEM4:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM4);
        break;
        /* DMEM 3 */
        case SDL_HWA_DMEM3:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM3);
        break;
        /* DMEM 2 */
        case SDL_HWA_DMEM2:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM2);
        break;
        /* DMEM 1 */
        case SDL_HWA_DMEM1:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM1);
        break;
        /* DMEM 0 */
        case SDL_HWA_DMEM0:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM0);
        break;
        default:
        break;
    }
    return status;
}

/********************************************************************************************************
* API to used to clear error status
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-3895
 */

void SDL_HWA_clearErrStatus(SDL_HWA_MemBlock memBlock)
{
    uint8_t status = 0U;
    SDL_HWA_setParityErrMaskUnmask(memBlock,SDL_HWA_DISABLE_STS);
    switch(memBlock)
    {
        /* FSM Lockstep */
        case SDL_HWA_FSM_LOCKSTEP:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_FSM_LOCKSTEP);
                      HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_FSM_LOCKSTEP,status);
        break;
        /* Window RAM */
        case SDL_HWA_WINDOW_RAM:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_WINDOW_RAM);
                      HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_FSM_LOCKSTEP,status);
        break;
        /* DMEM7 */
        case SDL_HWA_DMEM7:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM7);
                      HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM7,status);
        break;
        /* DMEM6 */
        case SDL_HWA_DMEM6:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM6);
                      HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM6,status);
        break;
        /* DMEM 5 */
        case SDL_HWA_DMEM5:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM5);
                      HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM5,status);
        break;
        /* DMEM 4 */
        case SDL_HWA_DMEM4:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM4);
                      HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM4,status);
        break;
        /* DMEM 3 */
        case SDL_HWA_DMEM3:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM3);
                      HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM3,status);
        break;
        /* DMEM 2 */
        case SDL_HWA_DMEM2:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM2);
                      HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM2,status);
        break;
        /* DMEM 1 */
        case SDL_HWA_DMEM1:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM1);
                      HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM1,status);
        break;
        /* DMEM 0 */
        case SDL_HWA_DMEM0:
            status =  (uint8_t)HW_RD_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM0);
                      HW_WR_FIELD32((SDL_HWA_CFG+SDL_HWA_SAFETY_ERR_STATUS),\
                                     SDL_HWA_SAFETY_ERR_STATUS_DMEM0,status);
        break;
        default:
        break;
    }
}

/********************************************************************************************************
* API to used to get the base address of different HWA memory
*********************************************************************************************************/

int32_t SDL_HWA_getMemblockBaseaddr(SDL_HWA_MemID memID, SDL_HWA_MemBlock memBlock,\
                             uint32_t *baseAddr)
{
    int32_t status = SDL_PASS;
    if (baseAddr == NULL)
    {
        status = SDL_EBADARGS;
    }
    else
    {
        if (memID == SDL_HWA_DMA0_MEM_ID )
        {
            switch (memBlock)
            {
                case SDL_HWA_DMEM0:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA0_RAM_BANK0_BASE;
                break;
                case SDL_HWA_DMEM1:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA0_RAM_BANK1_BASE;
                break;
                case SDL_HWA_DMEM2:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA0_RAM_BANK2_BASE;
                break;
                case SDL_HWA_DMEM3:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA0_RAM_BANK3_BASE;
                break;
                case SDL_HWA_DMEM4:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA0_RAM_BANK4_BASE;
                break;
                case SDL_HWA_DMEM5:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA0_RAM_BANK5_BASE;
                break;
                case SDL_HWA_DMEM6:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA0_RAM_BANK6_BASE;
                break;
                case SDL_HWA_DMEM7:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA0_RAM_BANK7_BASE;
                break;
                default:
                status = SDL_EBADARGS;
                break;
            }
        }
        else if(memID == SDL_HWA_DMA1_MEM_ID )
        {
            switch (memBlock)
            {
                case SDL_HWA_DMEM0:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA1_RAM_BANK0_BASE;
                break;
                case SDL_HWA_DMEM1:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA1_RAM_BANK1_BASE;
                break;
                case SDL_HWA_DMEM2:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA1_RAM_BANK2_BASE;
                break;
                case SDL_HWA_DMEM3:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA1_RAM_BANK3_BASE;
                break;
                case SDL_HWA_DMEM4:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA1_RAM_BANK4_BASE;
                break;
                case SDL_HWA_DMEM5:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA1_RAM_BANK5_BASE;
                break;
                case SDL_HWA_DMEM6:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA1_RAM_BANK6_BASE;
                break;
                case SDL_HWA_DMEM7:
                *baseAddr = (uint32_t)SDL_DSS_HWA_DMA1_RAM_BANK7_BASE;
                break;
                default:
                status = SDL_EBADARGS;
                break;
            }
        }
        else if( memID == SDL_HWA_WINDOW_RAM_MEM_ID)
        {
            *baseAddr = (uint32_t)SDL_DSS_HWA_WINDOW_RAM_U_BASE;

        }
        else
        {
            status = SDL_EBADARGS;
        }
    }
    return (status);
}

