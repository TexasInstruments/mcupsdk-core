/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file     sdl_ip_esm.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of ESM.
 *            This also contains some related macros.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
/** \brief This is to disable HW_SYNC_BARRIER for register access */
#define MEM_BARRIER_DISABLE

#include <stdint.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include <sdl/esm/sdlr_esm.h>
#include "esm.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define ESM_NUM_INTR_PER_GRP              (32U)
#define ESM_INTR_GRP_NUM                  (32U)
#define ESM_MAX_NUM_INTRS                 (1024U)
#define ESM_ESM_PIN_CTRL_KEY_RESET_VAL    (0x5U)
#define ESM_SFT_RST_KEY_RESET_VAL         (0xFU)
#define ESM_EN_KEY_MASK                   (0xFU)
#define ESM_EN_KEY_ENBALE_VAL             (0xFU)
#define ESM_EN_KEY_DISABLE_VAL            (0x0U)


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  Design: PROC_SDL-1069,PROC_SDL-1070
 */
int32_t SDL_ESM_setMode(uint32_t baseAddr, esmOperationMode_t mode)
{
    int32_t    retVal = SDL_EBADARGS;
    uint32_t   regVal;

    /* Map the mode passed to supported values */
    if (baseAddr != ((uint32_t) (0u)))
    {
      if (mode == ESM_OPERATION_MODE_NORMAL)
      {
          regVal = ((uint32_t)(ESM_OPERATION_MODE_NORMAL));
      }
      else
      {
          regVal = ((uint32_t)(ESM_OPERATION_MODE_ERROR_FORCE));
      }
        HW_WR_FIELD32(baseAddr + SDL_ESM_PIN_CTRL, SDL_ESM_PIN_CTRL_KEY, regVal);
        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-1071,PROC_SDL-1072
 */
int32_t SDL_ESM_getPinMode(uint32_t baseAddr, esmOperationMode_t *pMode)
{
    int32_t    retVal = SDL_EBADARGS;
    uint32_t   regVal;
    if (baseAddr != ((uint32_t) (0u)))
    {
        if (pMode != NULL)
        {
            regVal = HW_RD_FIELD32(baseAddr + SDL_ESM_PIN_CTRL, SDL_ESM_PIN_CTRL_KEY);
            *pMode = regVal;
            retVal = SDL_PASS;
        }
    }

    return (retVal);
}

/**
 *  Design: PROC_SDL-7406,PROC_SDL-7407
 */
int32_t SDL_ESM_getErrorOutMode(uint32_t baseAddr, esmOperationMode_t *pMode)
{
    int32_t    retVal = SDL_EBADARGS;
    uint32_t   regVal;
    if (baseAddr != ((uint32_t) (0u)))
    {
        if (pMode != NULL)
        {
            regVal = HW_RD_FIELD32(baseAddr + SDL_ESM_PIN_CTRL, SDL_ESM_PIN_CTRL_PWM_EN);
            *pMode = regVal;
            retVal = SDL_PASS;
        }
    }

    return (retVal);
}

/**
 *  Design: PROC_SDL-1073,PROC_SDL-1074
 */
int32_t SDL_ESM_setInfluenceOnErrPin(uint32_t baseAddr, uint32_t intrSrc, bool  enable)
{
    int32_t  retVal ;
    uint32_t regVal = 0;

    if ( (baseAddr == ((uint32_t) (0u))) ||
         (intrSrc  >= ESM_MAX_NUM_INTRS) )
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        retVal = SDL_PASS;
        if ((bool) true == enable)
        {
            regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
            HW_WR_REG32(baseAddr +
                SDL_ESM_ERR_GRP_PIN_EN_SET(intrSrc / ESM_NUM_INTR_PER_GRP),
                regVal);
        }
        else
        {
            regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
            HW_WR_REG32(baseAddr +
                SDL_ESM_ERR_GRP_PIN_EN_CLR(intrSrc / ESM_NUM_INTR_PER_GRP),
                regVal);
        }
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-1075,PROC_SDL-1076
 */
int32_t SDL_ESM_getInfluenceOnErrPin(uint32_t baseAddr, uint32_t intrSrc,
                                     uint32_t *pInfluence)
{
    int32_t  retVal;
    uint32_t regVal, mask;

    if ( (baseAddr == ((uint32_t) (0u))) ||
         (intrSrc  >= ESM_MAX_NUM_INTRS) ||
         (pInfluence == ((void *) 0)))
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        retVal = SDL_PASS;
        regVal  = HW_RD_REG32(baseAddr +
               SDL_ESM_ERR_GRP_PIN_EN_SET(intrSrc / ESM_NUM_INTR_PER_GRP));
        mask   = ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        regVal &= mask;

        if (regVal == ((uint32_t) (0u)))
        {
           *pInfluence = ((uint32_t) (0u));
        }
        else
        {
           *pInfluence = ((uint32_t) (1u));
        }
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-1077,PROC_SDL-1078
 */
int32_t SDL_ESM_setErrPinLowTimePreload(uint32_t baseAddr, uint32_t lowTime)
{
    int32_t retVal = SDL_EBADARGS;

    if (baseAddr != ((uint32_t) (0u)))
    {
        if (SDL_ESM_PIN_CNTR_PRE_COUNT_MAX >= lowTime)
        {
            HW_WR_FIELD32(baseAddr + SDL_ESM_PIN_CNTR_PRE,
                          SDL_ESM_PIN_CNTR_PRE_COUNT,
                          lowTime);
            retVal = SDL_PASS;
        }
    }
    return retVal;
}

/**
 *  Design: PROC_SDL-7396, PROC_SDL-7395
 */
int32_t SDL_ESM_PWML_setErrPinLowTimePreload(uint32_t baseAddr, uint32_t lowTime)
{
    int32_t retVal = SDL_EBADARGS;

    if (baseAddr != ((uint32_t) (0u)))
    {
        if (SDL_ESM_PIN_CNTR_PRE_COUNT_MAX >= lowTime)
        {
            HW_WR_FIELD32(baseAddr + SDL_ESM_PWML_PIN_CNTR_PRE,
                          SDL_ESM_PIN_CNTR_PRE_COUNT,
                          lowTime);
            retVal = SDL_PASS;
        }
    }
    return retVal;
}

/**
 *  Design: PROC_SDL-7398, PROC_SDL-7399
 */
int32_t SDL_ESM_PWMH_setErrPinHighTimePreload(uint32_t baseAddr, uint32_t highTime)
{
    int32_t retVal = SDL_EBADARGS;

    if (baseAddr != ((uint32_t) (0u)))
    {
        if (SDL_ESM_PIN_CNTR_PRE_COUNT_MAX >= highTime)
        {
            HW_WR_FIELD32(baseAddr + SDL_ESM_PWMH_PIN_CNTR_PRE,
                          SDL_ESM_PIN_CNTR_PRE_COUNT,
                          highTime);
            retVal = SDL_PASS;
        }
    }
    return retVal;
}

/**
 *  Design: PROC_SDL-1079,PROC_SDL-1080
 */
int32_t SDL_ESM_getErrPinLowTimePreload(uint32_t baseAddr, uint32_t *pLowTime)
{
    int32_t retVal = SDL_EBADARGS;

    if (baseAddr != ((uint32_t) (0u)))
    {
        if (pLowTime != ((void *) 0))
        {
            *pLowTime = HW_RD_FIELD32(baseAddr + SDL_ESM_PIN_CNTR_PRE,
                          SDL_ESM_PIN_CNTR_PRE_COUNT);
            retVal = SDL_PASS;
        }
    }
    return retVal;
}

/**
 *  Design: PROC_SDL-1081,PROC_SDL-1082
 */
int32_t SDL_ESM_getCurrErrPinLowTimeCnt(uint32_t baseAddr, uint32_t *pPinCntrPre)
{
    int32_t    retVal = SDL_EBADARGS;

    if (baseAddr != ((uint32_t) (0u)))
    {
        if (pPinCntrPre != ((void *) 0))
        {
            *pPinCntrPre = HW_RD_FIELD32(baseAddr + SDL_ESM_PIN_CNTR, SDL_ESM_PIN_CNTR_COUNT);
            retVal =SDL_PASS;
        }
     }
    return retVal;
}

/**
 *  Design: PROC_SDL-7394,PROC_SDL-7392
 */
int32_t SDL_ESM_PWMH_getErrPinHighTimePreload(uint32_t baseAddr, uint32_t *pPinPWMHCntrPre)
{
    int32_t retVal = SDL_EBADARGS;

    if (baseAddr != ((uint32_t) (0u)))
    {
        if (pPinPWMHCntrPre != ((void *) 0))
        {
            *pPinPWMHCntrPre = HW_RD_FIELD32(baseAddr + SDL_ESM_PWMH_PIN_CNTR_PRE,
                          SDL_ESM_PIN_CNTR_PRE_COUNT);
            retVal = SDL_PASS;
        }
    }
    return retVal;
}

/**
 * Design: PROC_SDL-7390,PROC_SDL-7388
 */
int32_t SDL_ESM_PWMH_getCurrErrPinHighTimeCnt(uint32_t baseAddr, uint32_t *pHighPWMHTime)
{
    int32_t    retVal = SDL_EBADARGS;

    if (baseAddr != ((uint32_t) (0u)))
    {
        if (pHighPWMHTime != ((void *) 0))
        {
            *pHighPWMHTime = HW_RD_FIELD32(baseAddr + SDL_ESM_PWMH_PIN_CNTR, SDL_ESM_PIN_CNTR_COUNT);
            retVal =SDL_PASS;
        }
     }
    return retVal;
}

/**
 *  Design: PROC_SDL-7393,PROC_SDL-7391
 */
int32_t SDL_ESM_PWML_getErrPinLowTimePreload(uint32_t baseAddr, uint32_t *pPinPWMLCntrPre)
{
    int32_t retVal = SDL_EBADARGS;

    if (baseAddr != ((uint32_t) (0u)))
    {
        if (pPinPWMLCntrPre!= ((void *) 0))
        {
            *pPinPWMLCntrPre = HW_RD_FIELD32(baseAddr + SDL_ESM_PWML_PIN_CNTR_PRE,
                          SDL_ESM_PIN_CNTR_PRE_COUNT);
            retVal = SDL_PASS;
        }
    }
    return retVal;
}

/**
 * Design: PROC_SDL-7387,PROC_SDL-7386
 */
int32_t SDL_ESM_PWML_getCurrErrPinLowTimeCnt(uint32_t baseAddr, uint32_t *pLowPWMLTime)
{
    int32_t    retVal = SDL_EBADARGS;

    if (baseAddr != ((uint32_t) (0u)))
    {
        if (pLowPWMLTime != ((void *) 0))
        {
            *pLowPWMLTime = HW_RD_FIELD32(baseAddr + SDL_ESM_PWML_PIN_CNTR, SDL_ESM_PIN_CNTR_COUNT);
            retVal =SDL_PASS;
        }
     }
    return retVal;
}

/**
 *  Design: PROC_SDL-1083,PROC_SDL-1084
 */
int32_t SDL_ESM_getErrPinStatus(uint32_t baseAddr, uint32_t *pStatus)
{
    int32_t    retVal = SDL_EBADARGS;
    if (baseAddr != ((uint32_t) (0u)))
    {
        if (pStatus != ((void *) 0))
        {
            *pStatus = HW_RD_FIELD32(baseAddr + SDL_ESM_PIN_STS, SDL_ESM_PIN_STS_VAL);
            retVal =  SDL_PASS;
        }
    }
    return retVal;
}

/**
 *  Design: PROC_SDL-1085,PROC_SDL-1086
 */
int32_t SDL_ESM_resetErrPin(uint32_t baseAddr)
{
    int32_t    retVal = SDL_EBADARGS;
    if (baseAddr != ((uint32_t) (0u)))
    {
        HW_WR_FIELD32(baseAddr + SDL_ESM_PIN_CTRL, SDL_ESM_PIN_CTRL_KEY,
                      ESM_ESM_PIN_CTRL_KEY_RESET_VAL);
        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-2005
 */
int32_t SDL_ESM_isEnableCfgIntr(uint32_t baseAddr, uint32_t group, uint32_t *pEnStatus)
{
    int32_t  retVal = SDL_EBADARGS;
    uint32_t regVal, mask;

    if ( (baseAddr == ((uint32_t) (0u)))   ||
         (group >= ESM_INTR_GRP_NUM) ||
         (pEnStatus == ((void *) 0)) )
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        retVal = SDL_PASS;
    }

    if (retVal == SDL_PASS)
    {
        regVal  = HW_RD_REG32(baseAddr + SDL_ESM_ERR_EN_SET);
        mask  = ((uint32_t) 0x1U << (group));
        regVal &= mask;

        if (regVal == ((uint32_t) (0u)))
        {
           *pEnStatus = ((uint32_t) (0u));
        }
        else
        {
           *pEnStatus = ((uint32_t) (1u));
        }
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-1087,PROC_SDL-1088
 */
int32_t SDL_ESM_isEnableIntr(uint32_t baseAddr, uint32_t intrSrc, uint32_t *pEnStatus)
{
    int32_t  retVal = SDL_EBADARGS;
    uint32_t regVal, mask;

    if ( (baseAddr == ((uint32_t) (0u)))   ||
         (intrSrc  >= ESM_MAX_NUM_INTRS) ||
         (pEnStatus == ((void *) 0)) )
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        retVal = SDL_PASS;
    }

    if (retVal == SDL_PASS)
    {
        regVal  = HW_RD_REG32(baseAddr +
                SDL_ESM_ERR_GRP_INTR_EN_SET(intrSrc / ESM_NUM_INTR_PER_GRP));
        mask  = ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        regVal &= mask;

        if (regVal == ((uint32_t) (0u)))
        {
           *pEnStatus = ((uint32_t) (0u));
        }
        else
        {
           *pEnStatus = ((uint32_t) (1u));
        }
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-7351,PROC_SDL-7352
 */
int32_t SDL_ESM_isEnableCriticalIntr(uint32_t baseAddr, uint32_t intrSrc, uint32_t *pEnStatus)
{
    int32_t  retVal = SDL_EBADARGS;
    uint32_t regVal, mask;

    if ( (baseAddr == ((uint32_t) (0u)))   ||
         (intrSrc  >= ESM_MAX_NUM_INTRS) ||
         (pEnStatus == ((void *) 0)) )
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        retVal = SDL_PASS;
    }

    if (retVal == SDL_PASS)
    {
        regVal  = HW_RD_REG32(baseAddr +
                SDL_ESM_ERR_GRP_CRITICAL_PRI_EN_SET(intrSrc / ESM_NUM_INTR_PER_GRP));
        mask  = ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        regVal &= mask;

        if (regVal == ((uint32_t) (0u)))
        {
           *pEnStatus = ((uint32_t) (0u));
        }
        else
        {
           *pEnStatus = ((uint32_t) (1u));
        }
    }
    return (retVal);
}

/**
 * Design: PROC_SDL-2007
 */
int32_t SDL_ESM_enableCfgIntr(uint32_t baseAddr, uint32_t group)
{
    int32_t retVal;
    uint32_t regVal;

    if ((baseAddr == ((uint32_t) (0u))) ||
        (group >= ESM_INTR_GRP_NUM))
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        regVal = HW_RD_REG32(baseAddr + SDL_ESM_ERR_EN_SET);
        regVal &= ~((uint32_t) 0x1U << (group));
        regVal |= ((uint32_t) 0x1U << (group));
        HW_WR_REG32(baseAddr + SDL_ESM_ERR_EN_SET, regVal);
        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-2006
 */
int32_t SDL_ESM_disableCfgIntr(uint32_t baseAddr, uint32_t group)
{
    int32_t  retVal;
    uint32_t regVal = 0x0U;

    if ( (baseAddr == ((uint32_t) (0u)))   ||
         (group >= ESM_INTR_GRP_NUM) )
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        regVal = ((uint32_t) 0x1U << (group));
        HW_WR_REG32(baseAddr + SDL_ESM_ERR_EN_CLR, regVal);
        retVal = SDL_PASS;
    }
    return retVal;
}

/**
 *  Design: PROC_SDL-1089,PROC_SDL-1090
 */
int32_t SDL_ESM_enableIntr(uint32_t baseAddr, uint32_t intrSrc)
{
    int32_t  retVal;
    uint32_t regVal;

    if ( (baseAddr == ((uint32_t) (0u)))   ||
         (intrSrc  >= ESM_MAX_NUM_INTRS) )
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        retVal = SDL_PASS;
    }

    if (retVal == SDL_PASS)
    {
        regVal  = HW_RD_REG32(baseAddr +
                SDL_ESM_ERR_GRP_INTR_EN_SET(intrSrc / ESM_NUM_INTR_PER_GRP));
        regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        HW_WR_REG32(baseAddr +
                SDL_ESM_ERR_GRP_INTR_EN_SET(intrSrc / ESM_NUM_INTR_PER_GRP),
                regVal);
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-7353,PROC_SDL-7354
 */
int32_t SDL_ESM_enableCriticalIntr(uint32_t baseAddr, uint32_t intrSrc)
{
    int32_t  retVal;
    uint32_t regVal;

    if ( (baseAddr == ((uint32_t) (0u)))   ||
         (intrSrc  >= ESM_MAX_NUM_INTRS) )
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        retVal = SDL_PASS;
    }

    if (retVal == SDL_PASS)
    {
        regVal  = HW_RD_REG32(baseAddr +
                SDL_ESM_ERR_GRP_CRITICAL_PRI_EN_SET(intrSrc / ESM_NUM_INTR_PER_GRP));
        regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        HW_WR_REG32(baseAddr +
                SDL_ESM_ERR_GRP_CRITICAL_PRI_EN_SET(intrSrc / ESM_NUM_INTR_PER_GRP),
                regVal);
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-7355,PROC_SDL-7356
 */
int32_t SDL_ESM_setCriticalIntrDelay(uint32_t baseAddr, uint32_t delay)
{
    int32_t  retVal;

    if (baseAddr == ((uint32_t) (0u)))
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        retVal = SDL_PASS;
    }

    if (retVal == SDL_PASS)
    {
        HW_WR_REG32(baseAddr +
                SDL_ESM_CRITICAL_PRI_INT_DELAY_CNTR_PRE,
                delay);
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-1091,PROC_SDL-1092
 */
int32_t SDL_ESM_disableIntr(uint32_t baseAddr, uint32_t intrSrc)
{
    int32_t  retVal;
    uint32_t regVal = 0x0U;

    if ( (baseAddr == ((uint32_t) (0u)))   ||
         (intrSrc  >= ESM_MAX_NUM_INTRS) )
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        retVal = SDL_PASS;
    }

    if (retVal == SDL_PASS)
    {
        regVal = ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        HW_WR_REG32(baseAddr +
            SDL_ESM_ERR_GRP_INTR_EN_CLR(intrSrc / ESM_NUM_INTR_PER_GRP),
            regVal);
     }
     return retVal;
}

/**
 *  Design: PROC_SDL-7357,PROC_SDL-7358
 */
int32_t SDL_ESM_disableCriticalIntr(uint32_t baseAddr, uint32_t intrSrc)
{
    int32_t  retVal;
    uint32_t regVal = 0x0U;

    if ( (baseAddr == ((uint32_t) (0u)))   ||
         (intrSrc  >= ESM_MAX_NUM_INTRS) )
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        retVal = SDL_PASS;
    }

    if (retVal == SDL_PASS)
    {
        regVal = ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        HW_WR_REG32(baseAddr +
            SDL_ESM_ERR_GRP_CRITICAL_PRI_EN_CLR(intrSrc / ESM_NUM_INTR_PER_GRP),
            regVal);
     }
     return retVal;
}

/**
 *  Design: PROC_SDL-1093,PROC_SDL-1094
 */
int32_t SDL_ESM_setIntrPriorityLvl(uint32_t baseAddr, uint32_t intrSrc,
                                   esmIntrPriorityLvl_t intrPriorityLvl)
{
    int32_t  retVal = SDL_EBADARGS;
    uint32_t regVal;

    if ( (baseAddr != ((uint32_t) (0u)))  &&
         (intrSrc  < ESM_MAX_NUM_INTRS) )
    {
        regVal  = HW_RD_REG32(baseAddr +
                SDL_ESM_ERR_GRP_INT_PRIO(intrSrc / ESM_NUM_INTR_PER_GRP));
        if (intrPriorityLvl == (esmIntrPriorityLvl_t)ESM_INTR_PRIORITY_LEVEL_LOW)
        {
            regVal &= ~((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        }
        else
        {
            regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        }
        HW_WR_REG32(baseAddr +
                SDL_ESM_ERR_GRP_INT_PRIO(intrSrc / ESM_NUM_INTR_PER_GRP),
                regVal);
        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-1095,PROC_SDL-1096
 */
int32_t SDL_ESM_getIntrPriorityLvl(uint32_t baseAddr, uint32_t intrSrc,
                                   esmIntrPriorityLvl_t *pIntrPriorityLvl)
{
    int32_t  retVal = SDL_EBADARGS;
    uint32_t regVal, mask;

    if ( (baseAddr != ((uint32_t) (0u))) &&
         (intrSrc  < ESM_MAX_NUM_INTRS)  &&
         (pIntrPriorityLvl != ((void *) 0)) )
    {
        regVal  = HW_RD_REG32(baseAddr +
                SDL_ESM_ERR_GRP_INT_PRIO(intrSrc / ESM_NUM_INTR_PER_GRP));
        mask = ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        regVal &= mask;
        if (regVal == ((uint32_t) (0u)))
        {
           *pIntrPriorityLvl = ESM_INTR_PRIORITY_LEVEL_LOW;
        }
        else
        {
            *pIntrPriorityLvl = ESM_INTR_PRIORITY_LEVEL_HIGH;
        }

        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-2008
 */
int32_t SDL_ESM_getCfgIntrStatus(uint32_t baseAddr, uint32_t group, uint32_t *pStatus)
{
    int32_t  retVal = SDL_EBADARGS;
    uint32_t regVal;

    if ( (baseAddr != ((uint32_t) (0u))) &&
         (group < ESM_INTR_GRP_NUM) && (pStatus != ((void *) 0)))
    {
        regVal  = HW_RD_REG32(baseAddr + SDL_ESM_ERR_STS);
        regVal &= ((uint32_t) 0x1U << (group));
        *pStatus  = (regVal >> (group));
        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-1097,PROC_SDL-1098
 */
int32_t SDL_ESM_getIntrStatus(uint32_t baseAddr, uint32_t intrSrc, uint32_t *pStatus)
{
    int32_t  retVal = SDL_EBADARGS;
    uint32_t regVal;

    if ( (baseAddr != ((uint32_t) (0u))) &&
         (intrSrc  < ESM_MAX_NUM_INTRS))
    {
        regVal  = HW_RD_REG32(baseAddr +
                    SDL_ESM_ERR_GRP_STS(intrSrc / ESM_NUM_INTR_PER_GRP));
        regVal &= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        if (pStatus != ((void *) 0))
        {
            *pStatus  = (regVal >> (intrSrc % ESM_NUM_INTR_PER_GRP));
            retVal = SDL_PASS;
        }
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-1103,PROC_SDL-1104
 */
int32_t SDL_ESM_getGroupIntrStatus(uint32_t baseAddr, esmIntrPriorityLvl_t intrPrioType,
                                   esmGroupIntrStatus_t *pIntrstatus)
{
    int32_t retVal = SDL_EBADARGS;

    if ((baseAddr != ((uint32_t) (0u))) &&
        (pIntrstatus != ((void *) 0)))
    {
        if (intrPrioType == (esmIntrPriorityLvl_t)ESM_INTR_PRIORITY_LEVEL_LOW)
        {
            pIntrstatus->highestPendPlsIntNum = HW_RD_FIELD32(
                        baseAddr + SDL_ESM_LOW_PRI, SDL_ESM_LOW_PRI_PLS);
            pIntrstatus->highestPendLvlIntNum = HW_RD_FIELD32(
                        baseAddr + SDL_ESM_LOW_PRI, SDL_ESM_LOW_PRI_LVL);
            pIntrstatus->grpIntrStatus = HW_RD_REG32(baseAddr + SDL_ESM_LOW);
        }
        else
        {
            pIntrstatus->highestPendPlsIntNum = HW_RD_FIELD32(
                        baseAddr + SDL_ESM_HI_PRI, SDL_ESM_HI_PRI_PLS);
            pIntrstatus->highestPendLvlIntNum = HW_RD_FIELD32(
                        baseAddr + SDL_ESM_HI_PRI, SDL_ESM_HI_PRI_LVL);
            pIntrstatus->grpIntrStatus = HW_RD_REG32(baseAddr + SDL_ESM_HI);
        }
        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-2010
 */
int32_t SDL_ESM_clearCfgIntrStatus(uint32_t baseAddr, uint32_t group)
{
    int32_t  retVal = SDL_EBADARGS;
    uint32_t regVal = 0U;

    if ( (baseAddr != ((uint32_t) (0u))) &&
         (group < ESM_INTR_GRP_NUM))
    {
        regVal = ((uint32_t) 0x1U << (group));
        HW_WR_REG32(baseAddr + SDL_ESM_ERR_STS, regVal);
        retVal = SDL_PASS;
    }
    return retVal;
}

/**
 *  Design: PROC_SDL-1105,PROC_SDL-1106
 */
int32_t SDL_ESM_clearIntrStatus(uint32_t baseAddr, uint32_t intrSrc)
{
    int32_t  retVal = SDL_EBADARGS;
    uint32_t regVal = 0U;

    if ( (baseAddr != ((uint32_t) (0u))) &&
         (intrSrc  < ESM_MAX_NUM_INTRS))
    {
        regVal = ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        HW_WR_REG32(baseAddr +
                SDL_ESM_ERR_GRP_STS(intrSrc / ESM_NUM_INTR_PER_GRP), regVal);
        retVal = SDL_PASS;
    }
    return retVal;
}

/**
 *  Design: PROC_SDL-2009
 */
int32_t SDL_ESM_setCfgIntrStatusRAW(uint32_t baseAddr, uint32_t group)
{
    uint32_t regVal;
    int32_t  retVal = SDL_EBADARGS;

    if ((baseAddr != ((uint32_t) (0u))) &&
        (group < ESM_INTR_GRP_NUM))
    {
        regVal  = HW_RD_REG32(baseAddr + SDL_ESM_ERR_RAW);
        regVal |= ((uint32_t) 0x1U << (group));
        HW_WR_REG32(baseAddr + SDL_ESM_ERR_RAW, regVal);
        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-1099,PROC_SDL-1100
 */
int32_t SDL_ESM_setIntrStatusRAW(uint32_t baseAddr, uint32_t intrSrc)
{
    uint32_t regVal;
    int32_t  retVal = SDL_EBADARGS;

    if ((baseAddr != ((uint32_t) (0u))) &&
        (intrSrc  < ESM_MAX_NUM_INTRS))
    {
        regVal  = HW_RD_REG32(baseAddr +
                    SDL_ESM_ERR_GRP_RAW(intrSrc / ESM_NUM_INTR_PER_GRP));
        regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        HW_WR_REG32(baseAddr +
                    SDL_ESM_ERR_GRP_RAW(intrSrc / ESM_NUM_INTR_PER_GRP),
                    regVal);
        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-1101,PROC_SDL-1102
 */
int32_t SDL_ESM_getIntrStatusRAW(uint32_t baseAddr, uint32_t intrSrc, uint32_t *pStatus)
{
    uint32_t regVal;
    int32_t  retVal = SDL_EBADARGS;

    if ((baseAddr != ((uint32_t) (0u))) &&
        (intrSrc  < ESM_MAX_NUM_INTRS)  &&
        (pStatus != ((void *) 0)))
    {
        regVal  = HW_RD_REG32(baseAddr +
                    SDL_ESM_ERR_GRP_RAW(intrSrc / ESM_NUM_INTR_PER_GRP));
        regVal &= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
        *pStatus  = (regVal >> (intrSrc % ESM_NUM_INTR_PER_GRP));
        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-1107,PROC_SDL-1108
 */
int32_t SDL_ESM_writeEOI(uint32_t baseAddr, esmIntrType_t intrType)
{
    int32_t  retVal = SDL_EBADARGS;

    if (baseAddr != ((uint32_t) (0u)))
    {
        HW_WR_FIELD32(baseAddr +SDL_ESM_EOI, SDL_ESM_EOI_KEY, intrType);
        retVal = SDL_PASS;
    }
    return retVal;
}

/**
 *  Design: PROC_SDL-1109,PROC_SDL-1110
 */
int32_t SDL_ESM_getRevisionId(uint32_t baseAddr, esmRevisionId_t *revId)
{
    uint32_t regVal;
    int32_t  retVal = SDL_EBADARGS;

    if (baseAddr != ((uint32_t) (0u)))
    {
        regVal  = HW_RD_REG32(baseAddr + SDL_ESM_PID);
        revId->scheme = HW_GET_FIELD(regVal, SDL_ESM_PID_SCHEME);
        revId->func   = HW_GET_FIELD(regVal, SDL_ESM_PID_FUNC);
        revId->rtlRev = HW_GET_FIELD(regVal, SDL_ESM_PID_RTL);
        revId->major  = HW_GET_FIELD(regVal, SDL_ESM_PID_MAJOR);
        revId->custom = HW_GET_FIELD(regVal, SDL_ESM_PID_CUSTOM);
        revId->minor  = HW_GET_FIELD(regVal, SDL_ESM_PID_MINOR);
        retVal = SDL_PASS;
    }
    return retVal;
}

/**
 *  Design: PROC_SDL-1111,PROC_SDL-1112
 */
int32_t SDL_ESM_getInfo(uint32_t baseAddr, esmInfo_t *info)
{
    uint32_t regVal;
    int32_t  retVal = SDL_EBADARGS;

    if (baseAddr != ((uint32_t) (0u)))
    {
        regVal  = HW_RD_REG32(baseAddr + SDL_ESM_INFO);
        info->lastRstType = HW_GET_FIELD(regVal, SDL_ESM_INFO_LAST_RESET);
        info->plsGrpNum   = HW_GET_FIELD(regVal, SDL_ESM_INFO_PULSE_GROUPS);
        info->lvlGrpNum   = (HW_GET_FIELD(regVal, SDL_ESM_INFO_GROUPS) -
                             HW_GET_FIELD(regVal, SDL_ESM_INFO_PULSE_GROUPS));
        retVal = SDL_PASS;
    }
    return retVal;
}

/**
 *  Design: PROC_SDL-1113,PROC_SDL-1114
 */
int32_t SDL_ESM_getGlobalIntrEnabledStatus(uint32_t baseAddr, uint32_t *pStatus)
{
    int32_t  retVal = SDL_EBADARGS;
    uint32_t regVal;

    if ( (baseAddr != ((uint32_t) (0u))) &&
         (pStatus  != ((void *) 0)) )
    {
        regVal = HW_RD_FIELD32(baseAddr +SDL_ESM_EN,
                      SDL_ESM_EN_KEY);

       *pStatus = regVal & ESM_EN_KEY_MASK;
        retVal = SDL_PASS;
    }
    return retVal;
}

/**
 *  Design: PROC_SDL-1115,PROC_SDL-1116
 */
int32_t SDL_ESM_enableGlobalIntr(uint32_t baseAddr)
{
    int32_t  retVal = SDL_EBADARGS;

    if (baseAddr != ((uint32_t) (0u)))
    {
        HW_WR_FIELD32(baseAddr +SDL_ESM_EN,
                      SDL_ESM_EN_KEY,
                      ESM_EN_KEY_ENBALE_VAL);
        retVal = SDL_PASS;
    }
    return retVal;
}

/**
 *  Design: PROC_SDL-1115,PROC_SDL-1116
 */
int32_t SDL_ESM_disableGlobalIntr(uint32_t baseAddr)
{
    int32_t  retVal = SDL_EBADARGS;

    if (baseAddr != ((uint32_t) (0u)))
    {
        HW_WR_FIELD32(baseAddr +SDL_ESM_EN,
                      SDL_ESM_EN_KEY,
                      ESM_EN_KEY_DISABLE_VAL);
        retVal = SDL_PASS;
     }
     return retVal;
}

/**
 *  Design: PROC_SDL-1117,PROC_SDL-1118
 */
int32_t SDL_ESM_reset(uint32_t baseAddr)
{
    int32_t  retVal = SDL_EBADARGS;

    if (baseAddr != ((uint32_t) (0u)))
    {
        HW_WR_FIELD32(baseAddr +SDL_ESM_SFT_RST,
                      SDL_ESM_SFT_RST_KEY,
                      ESM_SFT_RST_KEY_RESET_VAL);
        retVal = SDL_PASS;
    }
    return retVal;
}