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
#include <sdl/esm/v1/v1_0/sdlr_esm.h>
#include "esm.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

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
      if (mode == SDL_ESM_OPERATION_MODE_NORMAL)
      {
          regVal = ((uint32_t)(SDL_ESM_OPERATION_MODE_NORMAL));
      }
      else
      {
          regVal = ((uint32_t)(SDL_ESM_OPERATION_MODE_ERROR_FORCE));
      }
		HW_WR_FIELD32(baseAddr + SDL_ESM_ESMEKR, SDL_ESM_ESMEKR_EKEY, regVal);

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
			regVal = HW_RD_FIELD32(baseAddr + ESM_ESMEKR, ESM_ESMEKR_EKEY);
            *pMode = regVal;
            retVal = SDL_PASS;
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
      if (SDL_ESM_ESMLTCPR_LTCP_MAX >= lowTime)
        {		  
			HW_WR_FIELD32(baseAddr + ESM_ESMLTCPR, ESM_ESMLTCPR_LTCPR, lowTime);
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
			*pLowTime = HW_RD_FIELD32(baseAddr + ESM_ESMLTCPR, ESM_ESMLTCPR_LTCPR);
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
			*pPinCntrPre = HW_RD_FIELD32(baseAddr + ESM_ESMLTCR, ESM_ESMLTCR_LTC);
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
			*pStatus = HW_RD_FIELD32(baseAddr + ESM_ESMEPSR, ESM_ESMEPSR_EPSF);
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
		HW_WR_FIELD32(baseAddr + ESM_ESMEKR, ESM_ESMEKR_EKEY,
                  ESM_ESMEKR_EKEY_ERROR_PIN_RESET);
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

    if ((baseAddr != (uint32_t) (0u)) &&
	   (intrSrc  <= SDL_ESM_NUM_INTR_PER_GROUP) 
	   && (pStatus!=NULL))
    {			
		regVal  = HW_RD_REG32(baseAddr + SDL_ESM_ESMSR(intrSrc));	
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
int32_t SDL_ESM_getGroupIntrStatus(uint32_t baseAddr, uint32_t grpNum,
									SDL_ESM_GroupIntrStatus *intrstatus)
{
    int32_t retVal = SDL_EBADARGS;
    uint32_t loopCnt = 0;

    if ((SDL_ESM_NUM_GROUP_MAX < grpNum) &&
		(intrstatus != NULL))
    {
        retVal = SDL_EFAIL;
    }
    else
    {
        switch (grpNum)
        {
            case 1U:
                for (loopCnt = 0; loopCnt < SDL_ESM_ESMSR_NUM_ELEMS; loopCnt++)
                {
                    intrstatus->grpIntrStatus[loopCnt] =
                        HW_RD_REG32(baseAddr + SDL_ESM_ESMSR(loopCnt * 32U));
                }
                retVal = SDL_PASS;
                break;
            case 2U:
                retVal = SDL_PASS;
                intrstatus->grpIntrStatus[0] = HW_RD_REG32(baseAddr + ESM_ESMSR2);
                break;
            case 3U:
                retVal = SDL_EFAIL;
                intrstatus->grpIntrStatus[0] = HW_RD_REG32(baseAddr + ESM_ESMSR3);
                break;
            default:
                retVal = SDL_EBADARGS;
                break;
        }
    }
    return retVal;
}

/**
 *  Design: PROC_SDL-1073,PROC_SDL-1074
 */
int32_t SDL_ESM_setInfluenceOnErrPin(uint32_t baseAddr, uint32_t intrSrc, bool  enable)
{
    int32_t  retVal ;
    uint32_t regVal = 0;

    if ( (baseAddr == ((uint32_t) (0u))) ||
         (intrSrc  >= SDL_ESM_NUM_INTR_PER_GROUP) )
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        retVal = SDL_PASS;
        if ((bool) true == enable)
		{			
			regVal  = HW_RD_REG32(baseAddr + SDL_ESM_ESMIEPSR(intrSrc));
			regVal &= ~((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
			regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
			HW_WR_REG32(baseAddr + SDL_ESM_ESMIEPSR(intrSrc), regVal);
        }
        else
        {				
			regVal  = HW_RD_REG32(baseAddr + SDL_ESM_ESMIEPCR(intrSrc));
			regVal &= ~((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
			regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
			HW_WR_REG32(baseAddr + SDL_ESM_ESMIEPCR(intrSrc), regVal);		
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
		(intrSrc  >= SDL_ESM_NUM_INTR_PER_GROUP) ||
         (pInfluence == ((void *) 0)))
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        retVal = SDL_PASS;
        regVal  = HW_RD_REG32(baseAddr + SDL_ESM_ESMIEPSR(intrSrc));
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
 *  Design: PROC_SDL-1089,PROC_SDL-1090
 */
int32_t SDL_ESM_enableIntr(uint32_t baseAddr, uint32_t intrSrc)
{
    int32_t  retVal;
    uint32_t regVal;

    if ( (baseAddr == ((uint32_t) (0u)))   ||
         (intrSrc  >= SDL_ESM_NUM_INTR_PER_GROUP) )
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        retVal = SDL_PASS;
    }

    if (retVal == SDL_PASS)
    {		
		regVal  = HW_RD_REG32(baseAddr + SDL_ESM_ESMIESR(intrSrc));
		regVal &= ~((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
		regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
		HW_WR_REG32(baseAddr + SDL_ESM_ESMIESR(intrSrc), regVal);
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


    if((baseAddr == ((uint32_t) (0u))) ||
	  (intrSrc  >= SDL_ESM_NUM_INTR_PER_GROUP))
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        retVal = SDL_PASS;
    }

    if (retVal == SDL_PASS)
    {
		regVal  = HW_RD_REG32(baseAddr + SDL_ESM_ESMIECR(intrSrc));
		regVal &= ~((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
		regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
		HW_WR_REG32(baseAddr + SDL_ESM_ESMIECR(intrSrc), regVal);
     }
     return retVal;
}

/**
 *  Design: PROC_SDL-1087,PROC_SDL-1088
 */
int32_t SDL_ESM_isEnableIntr(uint32_t baseAddr, uint32_t intrSrc, uint32_t *pEnStatus)
{
    int32_t  retVal = SDL_EBADARGS;
    uint32_t regVal, mask;

    if ( (baseAddr == ((uint32_t) (0u)))   ||
         (intrSrc  >= SDL_ESM_NUM_INTR_PER_GROUP) ||
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
		regVal = HW_RD_REG32 (baseAddr + SDL_ESM_ESMIESR(intrSrc));
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
 *  Design: PROC_SDL-1093,PROC_SDL-1094
 */
int32_t SDL_ESM_setIntrPriorityLvl(uint32_t baseAddr, uint32_t intrSrc,
                                   esmIntrPriorityLvl_t intrPriorityLvl)
{
    int32_t  retVal = SDL_EBADARGS;
    uint32_t regVal;
    
	if ( (baseAddr != ((uint32_t) (0u)))   &&
        (intrSrc  <= SDL_ESM_NUM_INTR_PER_GROUP))
	{
		retVal = SDL_PASS;
		if (intrPriorityLvl == (esmIntrPriorityLvl_t)SDL_ESM_INTR_PRIORITY_LEVEL_LOW)
		{
			regVal  = HW_RD_REG32(baseAddr + SDL_ESM_ESMILCR(intrSrc));
			regVal &= ~((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
			regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
			HW_WR_REG32(baseAddr + SDL_ESM_ESMILCR(intrSrc), regVal);
		}
		else
		{
			regVal  = HW_RD_REG32(baseAddr + SDL_ESM_ESMILSR(intrSrc));
			regVal &= ~((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
			regVal |= ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
			HW_WR_REG32(baseAddr + SDL_ESM_ESMILSR(intrSrc), regVal);
		}		
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
         (pIntrPriorityLvl != ((void *) 0u)) )
    {				
		regVal  = HW_RD_REG32(baseAddr + SDL_ESM_ESMILCR(intrSrc));
		mask = ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
		regVal &= mask;
		if (regVal == ((uint32_t) (0u)))
        {
           *pIntrPriorityLvl = SDL_ESM_INTR_PRIORITY_LEVEL_LOW;
        }
		else
		{
			regVal  = HW_RD_REG32(baseAddr + SDL_ESM_ESMILSR(intrSrc));
			mask = ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));
			regVal &= mask;
			if (regVal != ((uint32_t) (0u)))
			{
				*pIntrPriorityLvl = SDL_ESM_INTR_PRIORITY_LEVEL_HIGH;
			}
				
		}	
        retVal = SDL_PASS;
    }
    return (retVal);
}

/**
 *  Design: PROC_SDL-1105,PROC_SDL-1106
 */
int32_t SDL_ESM_clearIntrStatus(uint32_t baseAddr, uint32_t intrSrc)
{
    int32_t  retVal = SDL_EBADARGS;
    uint32_t regVal = 0U;
    if((baseAddr != ((uint32_t) (0u))) &&
	  (intrSrc  <= SDL_ESM_NUM_INTR_PER_GROUP))
    {
		regVal = ((uint32_t) 0x1U << (intrSrc % ESM_NUM_INTR_PER_GRP));			
		HW_WR_REG32(baseAddr + SDL_ESM_ESMSR(intrSrc), regVal);		
        retVal = SDL_PASS;
    }
    return retVal;
}

int32_t SDL_ESM_clearGroupIntrStatus(uint32_t baseAddr, uint32_t grpNum)
{
    int32_t  retVal = SDL_PASS;
    uint32_t loopCnt = 0;
    uint32_t regVal  = 0xFFFFFFFFU;

    if (SDL_ESM_NUM_GROUP_MAX < grpNum)
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        switch (grpNum)
        {
            case 1U:
                for (loopCnt = 0; loopCnt < SDL_ESM_ESMSR_NUM_ELEMS; loopCnt++)
                {
                    HW_WR_REG32(baseAddr + SDL_ESM_ESMSR(loopCnt * 32U), regVal);
                }
                retVal = SDL_PASS;
                break;
            case 2U:
                retVal = SDL_PASS;
                HW_WR_REG32(baseAddr + ESM_ESMSR2, regVal);
                break;
            case 3U:
                retVal = SDL_PASS;
                HW_WR_REG32(baseAddr + ESM_ESMSR3, regVal);
                break;
            default:
                retVal = SDL_EBADARGS;
                break;
        }
    }
    return retVal;
}

int32_t SDL_ESM_getLowPriorityLvlIntrStatus(uint32_t baseAddr, uint32_t *pstatus)
{
    int32_t  retVal = SDL_EBADARGS;
    if(baseAddr != ((uint32_t) (0u)))
    {
		*pstatus = HW_RD_FIELD32(baseAddr + ESM_ESMIOFFLR, ESM_ESMIOFFLR_INTOFFL);	
        retVal = SDL_PASS;
    }
    return retVal;
}

int32_t SDL_ESM_getHighPriorityLvlIntrStatus(uint32_t baseAddr, uint32_t *pstatus)
{
    int32_t  retVal = SDL_EBADARGS;
    if(baseAddr != ((uint32_t) (0u)))
    {
		*pstatus = HW_RD_FIELD32(baseAddr + ESM_ESMIOFFHR, ESM_ESMIOFFHR_INTOFFH);	
        retVal = SDL_PASS;
    }
    return retVal;
}

/* Nothing past from this point */
