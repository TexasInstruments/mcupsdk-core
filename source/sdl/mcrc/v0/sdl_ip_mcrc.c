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
 *  \file     sdl_ip_mcrc.c
 *
 *  \brief    This file contains the implementation of the low level API's present in the
 *            device abstraction layer file of MCRC.
 */
 
#include <stdint.h>
#include <stdbool.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include <sdl/dpl/sdl_dpl.h>
#include "sdl_ip_mcrc.h"
#include "sdl_mcrc_hw.h"
#include <sdl/mcrc/v0/soc/sdl_mcrc_soc.h>

/**
 *  Design: PROC_SDL-2098 
 */
int32_t SDL_MCRC_dataBusTracingCtrl(SDL_MCRC_InstType instance,
                                    uint32_t          ctrlFlag,
                                    SDL_MCRC_DataBusMask_t dataBusMask,
                                    SDL_MCRC_DataBusMask_t busEnableMask)
{
    int32_t  status = SDL_PASS;
    uint32_t regVal;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS) ||
        (ctrlFlag > SDL_MCRC_MAX_CTRL_FLAG_VAL)                 ||
        ((dataBusMask & (~SDL_MCRC_DATA_BUS_MASK_ALL)) != 0U)   ||
        ((busEnableMask & (~SDL_MCRC_DATA_BUS_MASK_ALL)) != 0U))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        /* Configure channel 1 data tracing */
        HW_WR_FIELD32(baseAddr + SDL_MCRC_CTRL2,
                      SDL_MCRC_CTRL2_CH1_TRACEEN,
                      ctrlFlag);

        /* Configure data bus tracing control */
        regVal = HW_RD_REG32(baseAddr + SDL_MCRC_MCRC_BUS_SEL);
        regVal &= ~dataBusMask;
        regVal |= busEnableMask;
        HW_WR_REG32(baseAddr + SDL_MCRC_MCRC_BUS_SEL, regVal);
    }

    return (status);
}

/**
 *  Design: PROC_SDL-2100
 */
int32_t SDL_MCRC_getHighestPriorityIntrStatus(SDL_MCRC_InstType instance, uint32_t *pIntVecAddr)
{
    int32_t status;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS)   ||
        (pIntVecAddr == (NULL_PTR)))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        *pIntVecAddr = HW_RD_FIELD32(baseAddr + SDL_MCRC_INT_OFFSET_REG, SDL_MCRC_INT_OFFSET_REG);
        status = SDL_PASS;
    }

    return (status);
}

/**
 *  Design: PROC_SDL-2099 
 */
int32_t SDL_MCRC_verifyBusTracingCtrl(SDL_MCRC_InstType      instance,
                                      uint32_t               ctrlFlag,
                                      SDL_MCRC_DataBusMask_t dataBusMask,
                                      SDL_MCRC_DataBusMask_t busEnableMask)
{
    int32_t  status = SDL_PASS;
    uint32_t traceEnable;
    uint32_t regVal;
    uint32_t baseAddr;

    if ((SDL_MCRC_getBaseaddr(instance, &baseAddr) != SDL_PASS) ||
        (ctrlFlag > SDL_MCRC_MAX_CTRL_FLAG_VAL)                 ||
        ((dataBusMask & (~SDL_MCRC_DATA_BUS_MASK_ALL)) !=  0U)  ||
        ((busEnableMask & (~SDL_MCRC_DATA_BUS_MASK_ALL)) != 0U))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        /* Read channel 1 data tracing enable bit */
        traceEnable = HW_RD_FIELD32(baseAddr + SDL_MCRC_CTRL2,
                                    SDL_MCRC_CTRL2_CH1_TRACEEN);
        if (traceEnable != ctrlFlag)
        {
            status = SDL_EFAIL;
        }
    }

    if (status == SDL_PASS)
    {
        /* Read data bus tracing control enable bits */
        regVal = HW_RD_REG32(baseAddr + SDL_MCRC_MCRC_BUS_SEL);

        if ((regVal & dataBusMask) !=  busEnableMask)
        {
            status = SDL_EFAIL;
        }
    }

    return (status);

}

