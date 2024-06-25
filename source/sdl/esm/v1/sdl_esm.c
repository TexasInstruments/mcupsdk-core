/*
 * sdl_esm.c
 *
 * Software Diagnostics Reference module for Error Signaling Module
 *
 *  Copyright (c) Texas Instruments Incorporated 2022
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
 *  \file sdl_esm.c
 *
 *  \brief File containing ESM Module SDL APIs implementation for version v1.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
/* This is needed for memset/memcpy */
#include <string.h>
#include <stdint.h>
#include "sdl_esm.h"
#include <sdl/include/hw_types.h>
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define SDL_ESM_EN_KEY_ENBALE_VAL (0xFU)
#define ARRAY_SIZE (32u)
#define INIT_VAL (0u)
#define MASK_BIT (8u)
#define STATUS_NUM (1u)
#define GROUP_TWO (2u)
#define GROUP_THREE (2u)
#define GROUP_ONE (1u)


static pSDL_DPL_HwipHandle SDL_ESM_HiHwiPHandle;
static pSDL_DPL_HwipHandle SDL_ESM_LoHwiPHandle;
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/* ESM Module handles */
SDL_ESM_Handle gEsmHandle;
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/* SDL internal functions */
static void SDL_ESM_highpriority_interrupt(void *args);
static void SDL_ESM_lowpriority_interrupt(void *args);
static void SDL_ESM_processInterrupt (void *arg,
                                  uint32_t vec,
                                  int32_t  *groupNum,
                                  int32_t* vecNum);
static int32_t SDL_ESM_configErrorGating(SDL_ESM_Handle gHandle,
										uint8_t groupNumber,
										uint8_t errorNumber, uint8_t gating);
static void SDL_ESM_selfTestCallback(void);
static void SDL_ESM_memcpy(void *dest, void *src, size_t n);
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/* ========================================================================== */
/*                          Static Function Definitions                       */
/* ========================================================================== */

static void SDL_ESM_selfTestCallback(void)
{
	SDL_ESM_Config             *ptrESMConfig;
	SDL_ESM_Object          *object;
	/* Get the ESM Configuration: */
    ptrESMConfig = (SDL_ESM_Config*)gEsmHandle;
    object  = (SDL_ESM_Object*)ptrESMConfig->object;
    object->selfTestFlag  = (bool)true;

    return;
}

static int32_t SDL_ESM_configErrorGating(SDL_ESM_Handle gHandle, uint8_t groupNumber, uint8_t errorNumber, uint8_t gating)
{
    uint32_t            regVal;
    uint32_t            regIndex;
    uint32_t            regAddr;
    int32_t             retVal = SDL_PASS;
	SDL_ESM_Config             *ptrESMConfig;
    const SDL_ESM_Params        *esmConfig;
	SDL_mss_ctrlRegs    *ptrCtrlRegs;
    ptrESMConfig = (SDL_ESM_Config*)gHandle;
    esmConfig = ptrESMConfig->esmConfig;
    ptrCtrlRegs = esmConfig->ptrCtrlRegs;

    /* Error gating is for Group 2  and Group 3 errors only */
    if ((groupNumber < GROUP_TWO) || (groupNumber > GROUP_THREE))
    {
        retVal = SDL_EFAIL;
    }

    /* For Group 2 and Group 3, there are up to 32 errors for each*/
    if (errorNumber > BITS_PER_WORD)
    {
        retVal = SDL_EFAIL;
    }

    if (SDL_PASS == retVal)
    {
        regIndex = (errorNumber / (uint8_t)ESM_NUM_EVTS_PER_GATING_REG) + ((groupNumber-2U) * (uint8_t)ESM_GATING_GROUP);
        regAddr  = ((uint32_t)(&ptrCtrlRegs->ESM_GATING0)) + (regIndex * (sizeof(uint32_t)));

        regVal = HW_RD_REG32((volatile uint32_t *)regAddr);
        regVal &= (uint32_t)(~((uint8_t)ESM_GATING_MASK << ((uint8_t)ESM_GATING_SHIFT*(errorNumber % (uint8_t)ESM_NUM_EVTS_PER_GATING_REG))));
        if (gating !=0U)
        {
            regVal |= (uint32_t)((uint8_t)ESM_GATING_MASK << ((uint8_t)ESM_GATING_SHIFT*(errorNumber % (uint8_t)ESM_NUM_EVTS_PER_GATING_REG)));
        }
        HW_WR_REG32((volatile uint32_t *)regAddr, regVal);
    }

    return retVal;
}

/*
 * Design: PROC_SDL-1047
 */

static void SDL_ESM_highpriority_interrupt(void *args)
{
    uint32_t            esmioffhr, vec;
    int32_t             groupNum = -1, vecNum = -1;
	uint32_t esm_base_addr;

    /*Passing the instance and using that to get the base_addr*/
    SDL_ESM_Inst instance = (SDL_ESM_Inst)(uint32_t)args;
    SDL_ESM_getBaseAddr(instance, &esm_base_addr);

	/* Check on the highest priority event and handle it */

	SDL_ESM_getHighPriorityLvlIntrStatus(esm_base_addr,&esmioffhr);
	vec = esmioffhr - 1U;

	SDL_ESM_processInterrupt(args, vec, &groupNum, &vecNum);

}

/*
 * Design: PROC_SDL-1047
 */

static void SDL_ESM_lowpriority_interrupt(void *args)
{
    uint32_t            esmioffhr, vec;
    int32_t             groupNum = -1, vecNum = -1;
	uint32_t esm_base_addr;

    /*Passing the instance and using that to get the base_addr*/
    SDL_ESM_Inst instance = (SDL_ESM_Inst)(uint32_t)args;
    SDL_ESM_getBaseAddr(instance, &esm_base_addr);

	/* Check on the highest priority event and handle it */
	SDL_ESM_getLowPriorityLvlIntrStatus(esm_base_addr,&esmioffhr);
	vec = esmioffhr - 1U;
	SDL_ESM_processInterrupt(args, vec, &groupNum, &vecNum);
}

static void SDL_ESM_processInterrupt (void *arg, uint32_t vec, int32_t* groupNum, int32_t *vecNum)
{
    SDL_ESM_Object          *object;
    SDL_ESM_Config          *ptrESMConfig;
	SDL_ESM_Inst instance = (SDL_ESM_Inst)(uint32_t)arg;
    uint32_t esm_base_addr;
    int32_t isHandled = (int32_t)FLAG_NO ;
    SDL_ESM_getBaseAddr(instance, &esm_base_addr);
    uint32_t index=0U;
	bool  loopexit=(bool)false;

    /* Get the ESM Configuration: */
    ptrESMConfig = (SDL_ESM_Config*)gEsmHandle;
    object  = (SDL_ESM_Object*)ptrESMConfig->object;

    /* Error numbers are extracted from the INTOFFH/INTOFFL depending upon the
     * processing of high or low priority interrupts
     * below info provides extracting the error number and group from these registers
     * Reg Val : Group
     * 1-32    : GRP1 (0 - 31)
     * 33-64   : GRP2 (0 - 31)
     * 65-96   : GRP1 (32 - 63)
     * 97-128  : Reserved
     * 129-160 : GRP1 (64-95)
     * 161-192 : Reserved
     * 193-224 : GRP1 (96-127)
     * >224    : Reserved
    */
    if (vec < 32u)
    {
        /* group 1 0-31 errors */
        object->debugEsmISRCount[0u]++;
        *groupNum = GROUP_ONE;
        *vecNum = vec;
    }
    else if (vec < 64u)
    {
        /* group 2 0-31 errors */
        object->debugEsmISRCount[1u]++;
        vec = vec - 32u;
        *groupNum = GROUP_TWO;
        *vecNum = vec;
    }
    else if (vec < 96u)
    {
        /* group 1 32-63 errors */
        object->debugEsmISRCount[((vec/32u) % SDL_ESM_MAX_ISR_COUNT)]++;
        vec = vec - 32u;
        *groupNum = GROUP_ONE;
        *vecNum = vec;
    }
    else if (vec < 160u)
    {
        /* group 1 64-95 errors */
        object->debugEsmISRCount[((vec/64u) % SDL_ESM_MAX_ISR_COUNT)]++;
        vec = vec - 64u;
        *groupNum = GROUP_ONE;
        *vecNum = vec;
    }
    else if (vec < 224u)
    {
        /* group 1 96-127 errors */
        object->debugEsmISRCount[((vec/96u) % SDL_ESM_MAX_ISR_COUNT)]++;
        vec = vec - 96u;
        *groupNum = GROUP_ONE;
        *vecNum = vec;
    }
    else
    {
        /*Required for MISRA C*/
    }

    if (*groupNum != -1)
    {
        /* Clear the error status flag for group 1 errors. There is no need to clear group 2 errors,
         * since the error status in ESMSR2 has been cleared when reading the appropriate vector
         * in the ESMIOFFHR offset register (via ESM_getHighPriorityLvlIntrStatus()).
         */
        if (*groupNum == GROUP_ONE)
        {
            SDL_ESM_clearIntrStatus(esm_base_addr, *vecNum);
        }
        for (index = 0u; index < SDL_ESM_MAX_NOTIFIERS; index++)
        {
			if(loopexit == (bool)false)
			{
			   if((object->eccenableEventBitmap[index] == *vecNum) && (object->eccenableEventBitmap[index] != 0U))
			   {
				   isHandled = object->eccCallBackFunction[index](instance, *groupNum,*vecNum,object->eccCallBackFunctionArg);
				   if (isHandled == (int32_t)FLAG_YES)
				   {
					   loopexit=(bool)true;
				   }
			   }
			   if((object->ccmenableBitmap[index] == *vecNum) && (object->ccmenableBitmap[index] != 0U) && (loopexit == (bool)false))
			   {
				   isHandled = object->ccmCallBackFunction[index](instance, *groupNum,*vecNum,object->ccmCallBackFunctionArg);
				   if (isHandled == (int32_t)FLAG_YES)
				   {
					   loopexit=(bool)true;
				   }
			   }
			   if ((*vecNum == (int32_t)(object->notifyParams[index].errorNumber)) &&
				  (*groupNum == (int32_t)(object->notifyParams[index].groupNumber)) &&
				  (loopexit == (bool)false))
			   {
				   /* Check if this is due to self test */
				   if((*groupNum == (int32_t)(object->notifyParams[index].esmErrorConfig.groupNumber)) &&
					  (*vecNum ==  (int32_t)(object->notifyParams[index].esmErrorConfig.eventNumber)))
				   {
						SDL_ESM_selfTestCallback();
						loopexit=(bool)true;
				   }
				   else if(isHandled == (int32_t)FLAG_NO)
				   {
						object->notifyParams[index].callBackFunction(instance,*groupNum,*vecNum,arg);
						loopexit=(bool)true;
				   }
				   else
				   {
						/*Nothing*/
				   }
				}
			}
        }
    }
}

static void SDL_ESM_memcpy(void *dest, void *src, size_t n)
{
    uint8_t *csrc = (uint8_t *)src;
    uint8_t *cdest = (uint8_t *)dest;
    uint32_t i=0U;

    /* Copy contents of src[] to dest[] */
    for(i=0U; i<n; i++)
    {
        cdest[i] = csrc[i];
    }
}
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * Design: PROC_SDL-1049, PROC_SDL-1048
 */

static SDL_Result SDL_esmgHandlerInit(SDL_ESM_Inst esmInstType, const SDL_ESM_Params  *hwAttrs, SDL_ESM_Object *object)
{
    int32_t                result = SDL_PASS;
    uint32_t               count;

	int32_t intNumHi, intNumLo;
    SDL_DPL_HwipParams intrParams;
    void (*pHiInterruptHandler)(void *);
    void (*pLoInterruptHandler)(void *);

	pHiInterruptHandler = &SDL_ESM_highpriority_interrupt;
	pLoInterruptHandler = &SDL_ESM_lowpriority_interrupt;

	/* Get the interrupt number corresponding to ESM HI interrupt */
	intNumHi = hwAttrs->highPrioIntNum;

	/* Get the interrupt number corresponding to ESM LO interrupt */
	intNumLo = hwAttrs->lowPrioIntNum;

	/* Clear all interrupts first */
	(void)SDL_DPL_disableInterrupt(intNumHi);
	(void)SDL_DPL_disableInterrupt(intNumLo);

	intrParams.intNum      = intNumHi;
	intrParams.callback    = (*pHiInterruptHandler);
	intrParams.callbackArg = (uintptr_t)esmInstType;

	/* Register call back function for ESM Hi Interrupt */
	result = SDL_DPL_registerInterrupt(&intrParams, &SDL_ESM_HiHwiPHandle);

	if (result == SDL_PASS)
	{
		intrParams.intNum = intNumLo;
		intrParams.callback = (*pLoInterruptHandler);
		intrParams.callbackArg = (uintptr_t)esmInstType;

		/* Register call back function for ESM Lo Interrupt */
		result = SDL_DPL_registerInterrupt(&intrParams, &SDL_ESM_LoHwiPHandle);
	}

	/* Enable all ESM Interrupts */
	if (result == SDL_PASS)
	{
		result = SDL_DPL_enableInterrupt(intNumHi);
		if (result == SDL_PASS)
		{
			result = SDL_DPL_enableInterrupt(intNumLo);
		}
	}
	if (object->params.bClearErrors == TRUE)
	{
		/* Clear ESM Group 1, 2, 3 errors */
		for (count=INIT_VAL; count<SDL_ESM_NUM_GROUP_MAX; count++)
		{
			SDL_ESM_clearGroupIntrStatus(object->esmBaseAddr, count+(uint32_t)1U);
		}
	}
    return result;
}

SDL_Result SDL_ESM_init (const SDL_ESM_Inst esmInstType,
                         SDL_ESM_NotifyParams* params,
						 SDL_ESM_OpenParams *esmOpenParams,
                         void *arg)
{
    SDL_ESM_Config          *ptrESMConfig;
    SDL_ESM_Object          *object;
    int32_t             	retVal = SDL_PASS;
	bool 					result = (bool)false;
	uint32_t				esmBaseAddr;
    static uint16_t          notifierIndex=0u;
    const SDL_ESM_Params    *hwAttrs;

    /*HSM instance is not supported */
    /* Check for valid esmInstConfig and esmInstType, and initialize appropriate
    smInstBaseAddr for register base.*/
    if((esmInstType >= SDL_ESM_INST_HSM_ESM) || (params == NULL))
    {
        retVal = SDL_EBADARGS;
    }
    else if (notifierIndex == SDL_ESM_MAX_NOTIFIERS)
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        gEsmHandle = NULL;   /* Init to NULL so that we can exit gracefully */

        ptrESMConfig = (SDL_ESM_Config*)(&gEsmConfig[esmInstType-1U]);
        if (retVal == SDL_PASS)
        {
            hwAttrs = ptrESMConfig->esmConfig;
            object  = ptrESMConfig->object;

            /* Initialize the memory */
            result = SDL_ESM_getBaseAddr (esmInstType,&esmBaseAddr);

            object->esmBaseAddr = esmBaseAddr;
            gEsmHandle = (SDL_ESM_Handle) ptrESMConfig;
            /* Init state */
            object->notifyParams[notifierIndex].esmErrorConfig.eventNumber = params->esmErrorConfig.eventNumber;
            object->notifyParams[notifierIndex].esmErrorConfig.groupNumber = params->esmErrorConfig.groupNumber;
            object->numGroup1Err = hwAttrs->numGroup1Err;
            object->selfTestFlag = (bool)false;

            if(NULL != esmOpenParams)
            {
                /* copy params into the driver object structure */
                SDL_ESM_memcpy(&object->params, esmOpenParams, sizeof(SDL_ESM_OpenParams));
            }
            else
            {
                /* Init with default if NULL is passed */
                SDL_ESM_Params_init(&object->params);
            }
            if ((gEsmHandle == NULL) || (result != ((bool)true)) ||
                (params == NULL))
            {
                retVal = SDL_EFAIL;
            }
            else
            {
                /* Check if the notifier gHandles group 1 or group 2 errors.
                Group 2 errors are enabled by default. Group 1 errors have to be explicitly enabled.
                Also, user can configure the interrupt priority level and influence on ERROR pin for group 1 errors.
                For group 2 errors, the interrupt priority level is always high, and the influence on ERROR pin is on always.
                */
                if (params->groupNumber == GROUP_ONE)
                {
                    SDL_ESM_enableIntr(object->esmBaseAddr, params->errorNumber);
                    /* Configure the interrupt priority level */
                    SDL_ESM_setIntrPriorityLvl(object->esmBaseAddr, params->errorNumber, params->setIntrPriorityLvl);
                    /* Configure the failure influence on ERROR pin */
                    SDL_ESM_setInfluenceOnErrPin(object->esmBaseAddr, params->errorNumber, params->enableInfluenceOnErrPin);
                }
                /* Unmask Group 2 ESM errors to enable the generation of NMI. */
                if (params->groupNumber == GROUP_TWO)
                {
                    retVal = SDL_ESM_configErrorGating(gEsmHandle, params->groupNumber, params->errorNumber, 0U);
                }

                SDL_ESM_memcpy((void *)&object->notifyParams[notifierIndex], (void *)params, sizeof (SDL_ESM_NotifyParams));
            }
        }
    }
    if (retVal == SDL_PASS)
    {
        retVal = SDL_esmgHandlerInit(esmInstType, hwAttrs, object);
        notifierIndex++;
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1062,PROC_SDL-1063
 */
/* Verifies the written config against the provided configuration */
int32_t SDL_ESM_verifyConfig(SDL_ESM_Inst instance, SDL_ESM_NotifyParams* params)
{
    uint32_t intStatus;
    uint32_t esmInstBaseAddr;
    esmIntrPriorityLvl_t intrPriorityLvlRd;
    uint32_t influence;
    int32_t result = SDL_EBADARGS;
    int32_t SDLRet= SDL_PASS;
    uint32_t enableRd;

    esmIntrPriorityLvl_t intrPriorityLvlWr;
    uint32_t enableWr;

    if((SDL_ESM_getBaseAddr(instance, &esmInstBaseAddr) == ((bool)false)) ||
	 (params->groupNumber > GROUP_ONE))
    {
            result = SDL_EBADARGS;
    }
    else
    {
        SDLRet = SDL_ESM_isEnableIntr(esmInstBaseAddr, params->errorNumber, &intStatus);
        enableWr = STATUS_NUM;

        if (intStatus == enableWr)
        {
            intrPriorityLvlWr = params->setIntrPriorityLvl;
            (void)SDL_ESM_getIntrPriorityLvl(esmInstBaseAddr,
                                                        params->errorNumber,
                                                        &intrPriorityLvlRd);

            if (intrPriorityLvlWr != intrPriorityLvlRd)
            {
                SDLRet = SDL_EFAIL;
            }

        }
        else
        {
            SDLRet = SDL_EFAIL;
        }
		if (SDLRet == SDL_PASS)
        {
            enableWr = (uint32_t)(params->enableInfluenceOnErrPin);
            SDLRet = SDL_ESM_getInfluenceOnErrPin(esmInstBaseAddr,
                                                          params->errorNumber,
                                                          &influence);
        }
        if (SDLRet == SDL_PASS)
        {
            enableRd = (influence != 0u)?1u:0u;

			if (enableWr != enableRd)
			{
				SDLRet = SDL_EFAIL;
			}
        }
        result = SDLRet;
    }
    return result;
}

/**
 * Design: PROC_SDL-1058,PROC_SDL-1059
 */
int32_t SDL_ESM_getNErrorStatus(SDL_ESM_Inst instance, uint32_t *pStatus)
{
    uint32_t esmInstBaseAddr;
    int32_t  retVal = SDL_EBADARGS;
	if (instance <= SDL_ESM_INST_DSS_ESM)
    {
        /*This function returns the Base address of ESM Instance Based on esmInstype*/
        if (SDL_ESM_getBaseAddr(instance, &esmInstBaseAddr) == ((bool)true))
        {
            if (pStatus != ((void *) 0))
            {
                /* This Function return the value of ESM_PIN_STS register bit0(status of nERROR pin)
                   of specified ESM instances*/
				*pStatus = HW_RD_FIELD32(esmInstBaseAddr + ESM_ESMEPSR, ESM_ESMEPSR_EPSF);
                retVal =  SDL_PASS;
            }
        }
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1056,PROC_SDL-1057
 */
/* Function used to clear the nERROR pin */
int32_t SDL_ESM_clrNError(SDL_ESM_Inst esmInstType)
{
    int32_t            result;
    uint32_t           esmInstBaseAddr;
    uint32_t           status;

    if (SDL_ESM_getBaseAddr(esmInstType, &esmInstBaseAddr) == ((bool)false))
    {
        result = SDL_EBADARGS;
    }
    else
    {
        result = SDL_ESM_resetErrPin((uint32_t)esmInstBaseAddr);
        (void)SDL_ESM_getErrPinStatus((uint32_t)esmInstBaseAddr, &status);

        if (status != ((uint32_t)1U))
        {
            result = SDL_EFAIL;
        }
    }

    return result;
}

/**
 * Design: PROC_SDL-1054,PROC_SDL-1055
 */
/*      Function sets the nERROR pin active.    */
int32_t SDL_ESM_setNError(SDL_ESM_Inst esmInstType)
{
    uint32_t esmInstBaseAddr;
    int32_t retVal;

    if (((esmInstType != SDL_ESM_INSTANCE_MAX) && (SDL_ESM_getBaseAddr(esmInstType, &esmInstBaseAddr) == ((bool)true))))
    {
        /* Set Force Error output */
        retVal = SDL_ESM_setMode(esmInstBaseAddr, SDL_ESM_OPERATION_MODE_ERROR_FORCE);
    }
    else
    {
        retVal = SDL_EBADARGS;
    }

    return retVal;
}

/* Read back the static config API implementation*/
/*Returns the static register configuration for the ESM module for the specified instance.*/
/**
 * Design: PROC_SDL-1060,PROC_SDL-1061
 */
/*This API can be used by the application to read back the static config. */
int32_t SDL_ESM_getStaticRegisters(SDL_ESM_Inst instance, SDL_ESM_staticRegs *pStaticRegs)
{
    int32_t    retVal = SDL_EBADARGS;
    uint32_t esmInstBaseAddr;

	if (instance <= SDL_ESM_INST_DSS_ESM)
    {

        if (SDL_ESM_getBaseAddr(instance, &esmInstBaseAddr) == ((bool)true))
        {
            /*Base address will assign to the Structure variable and Each structure variable 32bit wide*/
            SDL_esmRegs *esmRegs = (SDL_esmRegs *) (uintptr_t)esmInstBaseAddr;

            if(pStaticRegs != ((void *) 0))
            {

                pStaticRegs->ESMIEPSR1 = esmRegs->ESMIEPSR1;
                pStaticRegs->ESMIEPCR1 = esmRegs->ESMIEPCR1;
                pStaticRegs->ESMIESR1 = esmRegs->ESMIESR1;
                pStaticRegs->ESMIECR1 = esmRegs->ESMIECR1;
                pStaticRegs->ESMILSR1 = esmRegs->ESMILSR1;
                pStaticRegs->ESMILCR1 = esmRegs->ESMILCR1;
                pStaticRegs->ESMSR1   = esmRegs->ESMSR1;
                pStaticRegs->ESMSR2   = esmRegs->ESMSR2;
                pStaticRegs->ESMSR3   = esmRegs->ESMSR3;
                pStaticRegs->ESMEPSR  = esmRegs->ESMEPSR;
                pStaticRegs->ESMIOFFHR = esmRegs->ESMIOFFHR;
                pStaticRegs->ESMIOFFLR = esmRegs->ESMIOFFLR;
				pStaticRegs->ESMLTCR  = esmRegs->ESMLTCR;
                pStaticRegs->ESMLTCPR = esmRegs->ESMLTCPR;
                pStaticRegs->ESMEKR   = esmRegs->ESMEKR;
                pStaticRegs->ESMSSR2  = esmRegs->ESMSSR2;

                retVal = SDL_PASS;
            }
        }
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1064,PROC_SDL-1065
 */
int32_t SDL_ESM_registerECCCallback(SDL_ESM_Inst esmInstType,uint32_t eccEvent,
                                    SDL_ESM_CallBack callBack,
                                    void *callbackArg)
{
    SDL_Result result = SDL_PASS;
    static uint8_t callbackcount = 0;
    SDL_ESM_Object          *object;
    object = &gEsmObjects[CONFIG_ESM0];

    object->eccenableEventBitmap[callbackcount] = eccEvent;
    object->eccCallBackFunction[callbackcount] = callBack;
    object->eccCallBackFunctionArg[callbackcount] = callbackArg;
    callbackcount++;

    return result;
}

/**
 * Design: PROC_SDL-1066,PROC_SDL-1067
 */
int32_t SDL_ESM_registerCCMCallback(SDL_ESM_Inst esmInstType,uint32_t ccmEvent,
                                      SDL_ESM_CallBack callBack,
                                      void *callbackArg)
{
    static uint8_t callbackcount1 = 0U;
    SDL_Result result = SDL_PASS;
	SDL_ESM_Object          *object;

	if(callbackcount1 <= 255U)
	{
		object = &gEsmObjects[CONFIG_ESM0];
		object->ccmenableBitmap[callbackcount1] = ccmEvent;
		object->ccmCallBackFunction[callbackcount1] = callBack;
		object->ccmCallBackFunctionArg[callbackcount1] = callbackArg;
		callbackcount1++;
    }
	else
	{
		result = SDL_EFAIL;
	}
    return result;
}
/* Nothing past from this point */
