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

#include <stdint.h>
#include <sdl/esm/soc/sdl_esm_soc.h>
#include "sdl_esm.h"
#include <sdl/esm/sdlr_esm.h>
#include <sdl/esm/v2/v2_0/sdl_esm_priv.h>
#include <sdl/include/hw_types.h>
#include <sdl/sdlr.h>
#include <sdl/dpl/sdl_dpl.h>

#define BITS_PER_WORD (32u)

#define SDL_ESM_EN_KEY_ENBALE_VAL (0xFU)
#define ARRAY_SIZE (32u)
#define INIT_VAL (0u)
#define MASK_BIT (1u)
#define STATUS_NUM (1u)

/* Below macros are used to read the warm reset resaon. */
#define SDL_TOP_RCM_U_BASE                        (0x53200000U)
#define SDL_TOP_RCM_LOCK0_KICK0                   (0x00001008U)
#define SDL_TOP_RCM_LOCK0_KICK1                   (0x0000100CU)
#define KICK_LOCK_VAL                             (0x00000000U)
#define KICK0_UNLOCK_VAL                          (0x01234567U)
#define KICK1_UNLOCK_VAL                          (0x0FEDCBA8U)
#define SDL_MSS_TOPRCM_WARM_RST_CAUSE             (0x5320000CU)
#define SDL_MSS_TOPRCM_WARM_RST_CAUSE_CLR         (0x53200010U)
#define SDL_MSS_TOPRCM_WARM_RST_CAUSE_ESM         (0x00400000U)
#define SDL_MSS_TOPRCM_WARM_RST_CAUSE_CLR1_MASK   (0x7U)
#define SDL_MSS_TOPRCM_WARM_RST_CAUSE_CLR1_SHIFT  (0x2U)
#define SDL_MSS_TOPRCM_WARM_RST_CAUSE_ESM_MASK    (0x00400000U)
#define SDL_MSS_TOPRCM_WARM_RST_CAUSE_ESM_SHIFT   (22U)

/* Below macros are used to enable ESM warm reset. */
#define SDL_MSS_TOPRCM_WARM_RST_CFGE_MISC                    (0x53200044U)
#define SDL_MSS_TOPRCM_WARM_RST_CFGE_MISC_ESM_RST_EN_MASK    (0x7U)
#define SDL_MSS_TOPRCM_WARM_RST_CFGE_MISC_ESM_RST_EN_SHIFT   (0x0U)
#define SDL_MSS_TOPRCM_WARM_RST_CFGE_MISC_ESM_RST_EN         (0x7U)


static pSDL_DPL_HwipHandle SDL_ESM_HiHwiPHandle;
static pSDL_DPL_HwipHandle SDL_ESM_LoHwiPHandle;
static pSDL_DPL_HwipHandle SDL_ESM_CfgHwiPHandle;
/**
 * Design: PROC_SDL-1058,PROC_SDL-1059
 */
int32_t SDL_ESM_getNErrorStatus(SDL_ESM_Inst instance, uint32_t *pStatus)
{
    uint32_t esmInstBaseAddr;
    int32_t  retVal = SDL_EBADARGS;

    if (instance < SDL_ESM_INSTANCE_MAX)
    {
        /*This function returns the Base address of ESM Instance Based on esmInstype*/
        if (SDL_ESM_getBaseAddr(instance, &esmInstBaseAddr) == ((bool)true))
        {
            if (pStatus != ((void *) 0))
            {
                /* This Function return the value of ESM_PIN_STS register bit0(status of nERROR pin)
                   of specified ESM instances*/
                *pStatus = HW_RD_FIELD32(esmInstBaseAddr + SDL_ESM_PIN_STS, SDL_ESM_PIN_STS_VAL);
                retVal =  SDL_PASS;
            }
        }
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

    if (instance < SDL_ESM_INSTANCE_MAX)
    {
        if (SDL_ESM_getBaseAddr(instance, &esmInstBaseAddr) == ((bool)true))
        {
            /*Base address will assign to the Structure variable and Each structure variable 32bit wide*/
            SDL_esmRegs *esmRegs = (SDL_esmRegs *) (uintptr_t)esmInstBaseAddr;
            uint32_t  i;

            if(pStaticRegs != ((void *) 0))
            {
                pStaticRegs->PID           = esmRegs->PID;
                pStaticRegs->INFO          = esmRegs->INFO;
                pStaticRegs->EN            = esmRegs->EN;
                pStaticRegs->ERR_EN_SET    = esmRegs->ERR_EN_SET;
                pStaticRegs->ERR_EN_CLR    = esmRegs->ERR_EN_CLR;
                pStaticRegs->LOW_PRI       = esmRegs->LOW_PRI;
                pStaticRegs->HI_PRI        = esmRegs->HI_PRI;
                pStaticRegs->LOW           = esmRegs->LOW;
                pStaticRegs->HI            = esmRegs->HI;
                pStaticRegs->PIN_CTRL      = esmRegs->PIN_CTRL;
                pStaticRegs->PIN_CNTR      = esmRegs->PIN_CNTR;
                pStaticRegs->PIN_CNTR_PRE  = esmRegs->PIN_CNTR_PRE;
                pStaticRegs->PWMH_PIN_CNTR_PRE          = esmRegs->PWMH_PIN_CNTR_PRE;
                pStaticRegs->PWML_PIN_CNTR_PRE          = esmRegs->PWML_PIN_CNTR_PRE;

                /*It reads Error group Register of ESM instances*/

                for (i = ((uint32_t) (0u)); i < ESM_NUMBER_OF_GROUP_REGS; i++)
                {
                    pStaticRegs->ERR_GRP[i].RAW  = esmRegs->ERR_GRP[i].RAW;
                    pStaticRegs->ERR_GRP[i].INTR_EN_SET  = esmRegs->ERR_GRP[i].INTR_EN_SET;
                    pStaticRegs->ERR_GRP[i].INTR_EN_CLR  = esmRegs->ERR_GRP[i].INTR_EN_CLR;
                    pStaticRegs->ERR_GRP[i].INT_PRIO    = esmRegs->ERR_GRP[i].INT_PRIO;
                    pStaticRegs->ERR_GRP[i].PIN_EN_SET  = esmRegs->ERR_GRP[i].PIN_EN_SET;
                    pStaticRegs->ERR_GRP[i].PIN_EN_CLR  = esmRegs->ERR_GRP[i].PIN_EN_CLR;
                }
                retVal = SDL_PASS;
            }
        }
    }
    return retVal;
}

/**
 * Design: PROC_SDL-1062,PROC_SDL-1063
 */
/* Verifies the written config against the provided configuration */
int32_t SDL_ESM_verifyConfig(SDL_ESM_Inst instance, const SDL_ESM_config *pConfig)
{
    uint32_t intStatus;
    uint32_t esmMaxNumevents;
    uint32_t i,j;
    uint32_t esmInstBaseAddr;
    esmIntrPriorityLvl_t intrPriorityLvlRd;
    uint32_t influence;
    int32_t result = SDL_EBADARGS;
    uint32_t intNum;
    int32_t SDLRet= SDL_PASS;
    uint32_t enableRd;
    SDL_ESM_Instance_t *SDL_ESM_Instance;

    esmIntrPriorityLvl_t intrPriorityLvlWr;
    uint32_t enableWr;

    /* Check for valid esmInstConfig and esmInstType, and initialize appropriate
     * esmInstBaseAddr for register base and SDM_ESM_instance for SW instance structure. Also get Maximum number of events corresponding to the instance */
    if ((SDL_ESM_selectEsmInst(instance, &SDL_ESM_Instance) == ((bool)false)) ||
        (SDL_ESM_getBaseAddr(instance, &esmInstBaseAddr) == ((bool)false)))
    {
            result = SDL_EBADARGS;
    }
    else
    {
        SDL_ESM_getMaxNumEvents(instance, &esmMaxNumevents);
        /* ESM reset and configure */
        for (i=0; i < (esmMaxNumevents/BITS_PER_WORD); i++)
        {
            uint32_t remainingBits = esmMaxNumevents - (i*BITS_PER_WORD);

            if (remainingBits > BITS_PER_WORD)
            {
                remainingBits = BITS_PER_WORD;
            }

            for (j=((uint32_t)NULL); j< remainingBits; j++)
            {
                intNum = (i*BITS_PER_WORD)+j;

                SDLRet = SDL_ESM_isEnableIntr(esmInstBaseAddr, intNum, &intStatus);

                enableWr = ((pConfig->enableBitmap[i] & (((uint32_t)MASK_BIT)<<j)) != 0u)?1u:0u;

                if (intStatus == enableWr)
                {
                    intrPriorityLvlWr = ((pConfig->priorityBitmap[i]
                                                                  & (((uint32_t)1u)<<j)) != 0u)?1u:0u;
                    (void)SDL_ESM_getIntrPriorityLvl(esmInstBaseAddr,
                                                        intNum,
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
                    enableWr = ((pConfig->errorpinBitmap[i]
                                 & (((uint32_t)MASK_BIT)<<j)) != 0u)?1u:0u;
                    SDLRet = SDL_ESM_getInfluenceOnErrPin(esmInstBaseAddr,
                                                          intNum,
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
                if (SDLRet != SDL_PASS)
                {
                    break;
                }
            }
            if (SDLRet != SDL_PASS)
            {
                break;
            }

        }
        result = SDLRet;
    }
    return result;
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
        retVal = SDL_ESM_setMode(esmInstBaseAddr, ESM_OPERATION_MODE_ERROR_FORCE);
    }
    else
    {
        retVal = SDL_EBADARGS;
    }

    return retVal;
}

/*
* @fn      EsmInitHandlerInit
*
* @brief   Register all callbacks for an ESM instance
*
* @param   esmInstType: Instance type of ESM
*
* @return    0 : Success; < 0 for failures
*/
/*
 * Design: PROC_SDL-1049
 */

static SDL_Result Esmhandlerinit(SDL_ESM_Inst esmInstType)
{
    SDL_Result result = SDL_EBADARGS;
    int32_t intNumHi, intNumLo, intNumCfg;
    SDL_DPL_HwipParams intrParams;
    uint32_t esm_base_addr;
    void (*pHiInterruptHandler)(void *);
    void (*pLoInterruptHandler)(void *);
    void (*pConfigInterruptHandler)(void *);

    SDL_ESM_getBaseAddr(esmInstType, &esm_base_addr);

    pHiInterruptHandler = &SDL_ESM_hiInterruptHandler;
    pLoInterruptHandler = &SDL_ESM_loInterruptHandler;
    pConfigInterruptHandler = &SDL_ESM_configInterruptHandler;

    /* Get the interrupt number corresponding to ESM HI interrupt */
    intNumHi = SDL_ESM_getIntNumber(esmInstType,
                                    SDL_ESM_INT_TYPE_HI);

    /* Get the interrupt number corresponding to ESM LO interrupt */
    intNumLo = SDL_ESM_getIntNumber(esmInstType,
                                    SDL_ESM_INT_TYPE_LO);

    /* Get the interrupt number corresponding to ESM Config interrupt */
    intNumCfg = SDL_ESM_getIntNumber(esmInstType,
                                     SDL_ESM_INT_TYPE_CFG);

    /* Clear all interrupts first */
    (void)SDL_DPL_disableInterrupt(intNumHi);
    (void)SDL_DPL_disableInterrupt(intNumLo);
    (void)SDL_DPL_disableInterrupt(intNumCfg);

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

        if (result == SDL_PASS)
        {
            intrParams.intNum = intNumCfg;
            intrParams.callback = (*pConfigInterruptHandler);
            intrParams.callbackArg = (uintptr_t)esmInstType;

            /* Register call back function for ESM Config Interrupt */
            result = SDL_DPL_registerInterrupt(&intrParams, &SDL_ESM_CfgHwiPHandle);
        }
    }

    /* Enable all ESM Interrupts */
    if (result == SDL_PASS)
    {
        result = SDL_DPL_enableInterrupt(intNumHi);
        if (result == SDL_PASS)
        {
            result = SDL_DPL_enableInterrupt(intNumLo);
            if (result == SDL_PASS)
            {
                result = SDL_DPL_enableInterrupt(intNumCfg);
            }
        }
    }

    return result;
}

/** ============================================================================
 *
 * \brief   Initializes ESM module for SDL
 *
 * \param   esmInstType: Instance of ESM
 * \param   esmInitConfig: Configuration for ESM
 *
 * \return  SDL_RETURN_PASS : Success; SDL_RETURN_FAIL for failures
 */
static SDL_Result ESM_init (const SDL_ESM_Inst esmInstType,
                            const SDL_ESM_config *esmInitConfig,
                            SDL_ESM_applicationCallback callback,
                            void *arg)
{
    SDL_Result result = SDL_PASS;
    int32_t SDLRet;
    uint32_t intNum;
    uint32_t i,j;
    uint32_t intStatus;
    esmIntrPriorityLvl_t intrPriorityLvlWr, intrPriorityLvlRd;
    bool enableWr, enableRd;
    uint32_t influence;
    uint32_t esmInstBaseAddr;
    uint32_t esmMaxNumevents;
    SDL_ESM_Instance_t *SDL_ESM_Instance;

    if(SDL_ESM_selectEsmInst(esmInstType, &SDL_ESM_Instance) == ((bool)false)){
        result = SDL_EBADARGS;
    }
    /* Check for valid esmInstConfig and esmInstType, and initialize appropriate
     * esmInstBaseAddr for register base and SDM_ESM_instance for SW instance structure. Also get Maximum number of events corresponding to the instance */
    if (( esmInitConfig == ((void *)0u)) ||
        ( SDL_ESM_getBaseAddr(esmInstType, &esmInstBaseAddr) == ((bool)false)))
    {
            result = SDL_EBADARGS;
    }
    else
    {
        SDL_ESM_getMaxNumEvents(esmInstType, &esmMaxNumevents);
        /* Record init config in instance */
        SDL_ESM_Instance->esmInitConfig.esmErrorConfig.bitNumber = esmInitConfig->esmErrorConfig.bitNumber;
        SDL_ESM_Instance->esmInitConfig.esmErrorConfig.groupNumber = esmInitConfig->esmErrorConfig.groupNumber;
        for(i=INIT_VAL;i<ARRAY_SIZE;i++)
        {
            SDL_ESM_Instance->esmInitConfig.enableBitmap[i] = esmInitConfig->enableBitmap[i];
            SDL_ESM_Instance->esmInitConfig.priorityBitmap[i] = esmInitConfig->priorityBitmap[i];
            SDL_ESM_Instance->esmInitConfig.errorpinBitmap[i] = esmInitConfig->errorpinBitmap[i];
            SDL_ESM_Instance->esmInitConfig.enableCriticalBitmap[i] = esmInitConfig->enableCriticalBitmap[i];
        }
        SDL_ESM_Instance->callback = callback;
        SDL_ESM_Instance->arg = arg;

        /* ESM reset and configure */
        (void)SDL_ESM_reset(esmInstBaseAddr);

        /* Set the Critical Priority Interrupt Delay Counter  */
        (void)SDL_ESM_setCriticalIntrDelay(esmInstBaseAddr, esmInitConfig->criticalInterruptDelayCounter);

        /* Enable interrupt for all events from init configuration*/
        for(i=((uint32_t)NULL); i <= (esmMaxNumevents/BITS_PER_WORD); i++)
        {
            uint32_t remainingBits = esmMaxNumevents - (i*BITS_PER_WORD);

            /* Enable the configuration interrupt for this group */
            SDLRet = SDL_ESM_clearCfgIntrStatus(esmInstBaseAddr, i);
            if (SDLRet == SDL_PASS)
            {
                SDLRet = SDL_ESM_getCfgIntrStatus(esmInstBaseAddr, i, &intStatus);
            }
            if (SDLRet == SDL_PASS)
            {
                if (intStatus != ((uint32_t)0))
                {
                    SDLRet = SDL_EFAIL;
                }
            }
            if (SDLRet == SDL_PASS)
            {
                SDLRet = SDL_ESM_enableCfgIntr(esmInstBaseAddr, i);
                if (SDLRet == SDL_PASS)
                {
                    SDLRet = SDL_ESM_isEnableCfgIntr(esmInstBaseAddr, i, &intStatus);
                }
            }

            if (remainingBits > BITS_PER_WORD){
                remainingBits = BITS_PER_WORD;
            }

            for(j=((uint32_t)NULL); j< remainingBits; j++)
            {
                intNum = (i*BITS_PER_WORD)+j;

                /* Clear interrupt status, so that we start with clean state */
                (void)SDL_ESM_clearIntrStatus(esmInstBaseAddr, intNum);

                (void)SDL_ESM_getIntrStatus(esmInstBaseAddr, intNum, &intStatus);

                /* Depending on the bitmap configuration enable critical interrupt */
                if(((esmInitConfig->enableCriticalBitmap[i]) & (((uint32_t)MASK_BIT)<<j)) != 0u)
                {
                    /* Enable Critical interrupt and verifiy if interrupt status is enabled */
                    (void)SDL_ESM_enableCriticalIntr(esmInstBaseAddr, intNum);

                    (void)SDL_ESM_isEnableCriticalIntr(esmInstBaseAddr, intNum, &intStatus);

                    if (intStatus != ((uint32_t)STATUS_NUM))
                    {
                        SDLRet = SDL_EFAIL;
                    }
                }

                /* Depending on the bitmap configuration enable interrupt and set priority level */
                if(((esmInitConfig->enableBitmap[i]) & (((uint32_t)MASK_BIT)<<j)) != 0u)
                {
                    /* Enable interrupt and verifiy if interrupt status is enabled */

                    (void)SDL_ESM_enableIntr(esmInstBaseAddr, intNum);

                    (void)SDL_ESM_isEnableIntr(esmInstBaseAddr, intNum, &intStatus);

                    if (intStatus != ((uint32_t)STATUS_NUM))
                    {
                        SDLRet = SDL_EFAIL;
                    }

                    /* Set interrupt priority level and verifiy if priority level is set */
                    if (SDLRet == SDL_PASS)
                    {
                        intrPriorityLvlWr = ((esmInitConfig->priorityBitmap[i]
                                            & (((uint32_t)1u)<<j)) != 0u)?1u:0u;
                        SDLRet = SDL_ESM_setIntrPriorityLvl(esmInstBaseAddr,
                                                            intNum,
                                                            intrPriorityLvlWr);
                    }
                    if (SDLRet == SDL_PASS)
                    {
                        SDLRet = SDL_ESM_getIntrPriorityLvl(esmInstBaseAddr,
                                                            intNum,
                                                            &intrPriorityLvlRd);
                    }
                    if (SDLRet == SDL_PASS)
                    {
                        if (intrPriorityLvlWr != intrPriorityLvlRd)
                        {
                            SDLRet = SDL_EFAIL;
                        }
                    }
                }

                /*
                 * Depending on the bitmap configuration set error output
                 * and verify if error output is set */
                if (SDLRet == SDL_PASS)
                {
                    enableWr = ((esmInitConfig->errorpinBitmap[i]
                                 & (((uint32_t)MASK_BIT)<<j)) != 0u)?(bool)true:(bool)false;
                    SDLRet = SDL_ESM_setInfluenceOnErrPin(esmInstBaseAddr,
                                                          intNum,
                                                          enableWr);
                }
                if (SDLRet == SDL_PASS)
                {
                    SDLRet = SDL_ESM_getInfluenceOnErrPin(esmInstBaseAddr,
                                                          intNum,
                                                          &influence);
                }
                if (SDLRet == SDL_PASS)
                {
                    enableRd = (influence != 0u)?(bool)true:(bool)false;
                    if (enableWr != enableRd)
                    {
                        SDLRet = SDL_EFAIL;
                    }
                }
                if (SDLRet != SDL_PASS)
                {
                    break;
                }
            }
        }

        /* Enable Global interrupt and verify if global interrupt is enabled for ESM */
        if (SDLRet == SDL_PASS)
        {
            SDLRet = SDL_ESM_enableGlobalIntr(esmInstBaseAddr);
        }
        if (SDLRet == SDL_PASS)
        {
            SDLRet = SDL_ESM_getGlobalIntrEnabledStatus(esmInstBaseAddr, &intStatus);
        }
        if (SDLRet == SDL_PASS)
        {
            if (intStatus != SDL_ESM_EN_KEY_ENBALE_VAL)
            {
                SDLRet = SDL_EFAIL;
            }
        }
        if (SDLRet != SDL_PASS)
        {
            result = SDL_EFAIL;
        }
    }
    return result;
}

/**
 * Design: PROC_SDL-1064,PROC_SDL-1065
 */
int32_t SDL_ESM_registerECCCallback(SDL_ESM_Inst esmInstType,uint32_t eventBitmap[],
                                    SDL_ESM_applicationCallback callBack,
                                    void *callbackArg)
{
    uint8_t i;
    SDL_Result result = SDL_PASS;
    SDL_ESM_Instance_t *SDL_ESM_Instance;

    if (SDL_ESM_selectEsmInst(esmInstType, &SDL_ESM_Instance) == ((bool)false))
    {
        result = SDL_EFAIL;
    }

    for(i=INIT_VAL;i<SDL_ESM_MAX_EVENT_MAP_NUM_WORDS;i++)
    {
        SDL_ESM_Instance->eccenableBitmap[i] = eventBitmap[i];
    }
    SDL_ESM_Instance->eccCallBackFunction = callBack;
    SDL_ESM_Instance->eccCallBackFunctionArg = callbackArg;

    return result;
}
/**
 * Design: PROC_SDL-1066,PROC_SDL-1067
 */
int32_t SDL_ESM_registerCCMCallback(SDL_ESM_Inst esmInstType,uint32_t eventBitmap[],
                                      SDL_ESM_applicationCallback callBack,
                                      void *callbackArg)
{
    uint32_t i;
    SDL_Result result = SDL_PASS;
    SDL_ESM_Instance_t *SDL_ESM_Instance_CCM;

    if(SDL_ESM_selectEsmInst(esmInstType, &SDL_ESM_Instance_CCM) == ((bool)false))
    {
        result = SDL_EFAIL;
    }

    for(i=0U;i<SDL_ESM_MAX_EVENT_MAP_NUM_WORDS;i++)
    {
        SDL_ESM_Instance_CCM->ccmenableBitmap[i] = eventBitmap[i];
    }
    SDL_ESM_Instance_CCM->ccmCallBackFunction = callBack;
    SDL_ESM_Instance_CCM->ccmCallBackFunctionArg = callbackArg;

    return result;
}

/**
 * Design: PROC_SDL-1050,PROC_SDL-1051,PROC_SDL-1052,PROC_SDL-1053,PROC_SDL-1119
 */
 int32_t SDL_ESM_init(SDL_ESM_Inst instance, const SDL_ESM_config *pConfig, SDL_ESM_applicationCallback applicationCallback, void *appArg)
 {
     int32_t retval;

     if (applicationCallback == NULL)
     {
         retval = SDL_EBADARGS;
     }
     else
     {
         retval = ESM_init(instance,pConfig,applicationCallback,appArg);
     }

    if (retval == SDL_PASS)
    {
        retval = Esmhandlerinit(instance);
    }
     return retval;
}

/**
 * Design: PROC_SDL-7257
 */
int32_t SDL_ESM_getWarmResetReason(void)
{
    uint32_t            resetCause = 0U;
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;
    int32_t             retval;

    /* Unlock CONTROLSS_CTRL registers */
    /* Unlock TOP_RCM */
    baseAddr = (uint32_t) SDL_TOP_RCM_U_BASE;
    kickAddr = (volatile uint32_t *) (baseAddr + SDL_TOP_RCM_LOCK0_KICK0);
    SDL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);      /* KICK 0 */
    kickAddr = (volatile uint32_t *) (baseAddr + SDL_TOP_RCM_LOCK0_KICK1);
    SDL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);      /* KICK 1 */

    /* Read the Reset Cause Register bits */
    resetCause = (uint32_t)HW_RD_FIELD32 (SDL_MSS_TOPRCM_WARM_RST_CAUSE, SDL_MSS_TOPRCM_WARM_RST_CAUSE_ESM);

    /* clear the reset cause */
    HW_WR_FIELD32(SDL_MSS_TOPRCM_WARM_RST_CAUSE_CLR, SDL_MSS_TOPRCM_WARM_RST_CAUSE_CLR1, 0x7U);

    /* Lock CONTROLSS_CTRL registers */
    /* Lock TOP_RCM */
    kickAddr = (volatile uint32_t *) (baseAddr + SDL_TOP_RCM_LOCK0_KICK0);
    SDL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
    kickAddr = (volatile uint32_t *) (baseAddr + SDL_TOP_RCM_LOCK0_KICK1);
    SDL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */

    if(1U == resetCause)
    {
        /* Warm Reset was by ESM */
        retval = SDL_PASS;
    }
    else
    {
        /* Warm Reset was not by ESM */
        retval = SDL_EFAIL;
    }

    return (retval);
}

/**
 * Design: PROC_SDL-7256
 */
void SDL_ESM_enableESMWarmReset(void)
{
    uint32_t            baseAddr;
    volatile uint32_t  *kickAddr;

    /* Unlock CONTROLSS_CTRL registers */
    /* Unlock TOP_RCM */
    baseAddr = (uint32_t) SDL_TOP_RCM_U_BASE;
    kickAddr = (volatile uint32_t *) (baseAddr + SDL_TOP_RCM_LOCK0_KICK0);
    SDL_REG32_WR(kickAddr, KICK0_UNLOCK_VAL);      /* KICK 0 */
    kickAddr = (volatile uint32_t *) (baseAddr + SDL_TOP_RCM_LOCK0_KICK1);
    SDL_REG32_WR(kickAddr, KICK1_UNLOCK_VAL);      /* KICK 1 */

    /* Enable the warm reset */
    HW_WR_FIELD32(SDL_MSS_TOPRCM_WARM_RST_CFGE_MISC, \
                  SDL_MSS_TOPRCM_WARM_RST_CFGE_MISC_ESM_RST_EN, \
                  SDL_MSS_TOPRCM_WARM_RST_CFGE_MISC_ESM_RST_EN);

    /* Lock CONTROLSS_CTRL registers */
    /* Lock TOP_RCM */
    kickAddr = (volatile uint32_t *) (baseAddr + SDL_TOP_RCM_LOCK0_KICK0);
    SDL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 0 */
    kickAddr = (volatile uint32_t *) (baseAddr + SDL_TOP_RCM_LOCK0_KICK1);
    SDL_REG32_WR(kickAddr, KICK_LOCK_VAL);      /* KICK 1 */

}

/**
 * Design: PROC_SDL-7376
 */
int32_t SDL_ESM_setPinOutMode(SDL_ESM_Inst instance, esmErrOutMode_t pinOutMode)
{
    int32_t    retVal = SDL_EBADARGS;
    int32_t    sdlRet;
    uint32_t   intStatus;
    uint32_t   baseAddr;
    uint32_t   pinOutVal;

    if (((instance != SDL_ESM_INSTANCE_MAX) && (SDL_ESM_getBaseAddr(instance, &baseAddr) == ((bool)true))))
    {
        /* In SDL_ESM_init API, global interrupt are enabled so here
           it should get disabled before change outPut mode.
        */
        /* Disable global interrupt */
        sdlRet = SDL_ESM_disableGlobalIntr(baseAddr);

        if (sdlRet == SDL_PASS)
        {
            if (pinOutMode == SDL_ESM_PWM_PINOUT)
            {
                /* Enable PWM error Output */
                HW_WR_FIELD32(baseAddr + SDL_ESM_PIN_CTRL, SDL_ESM_PIN_CTRL_PWM_EN, SDL_ESM_PWM_PINOUT);
                /* Verify PWM error Output */
                pinOutVal = HW_RD_FIELD32(baseAddr + SDL_ESM_PIN_CTRL, SDL_ESM_PIN_CTRL_PWM_EN);

                if (pinOutVal == SDL_ESM_PWM_PINOUT)
                {
                    retVal = SDL_PASS;
                }
            }
            else if(pinOutMode == SDL_ESM_LVL_PINOUT)
            {
                /* Enable LVL error Output */
                HW_WR_FIELD32(baseAddr + SDL_ESM_PIN_CTRL, SDL_ESM_PIN_CTRL_PWM_EN, SDL_ESM_LVL_PINOUT);
                /* Verify PWM error Output */
                pinOutVal = HW_RD_FIELD32(baseAddr + SDL_ESM_PIN_CTRL, SDL_ESM_PIN_CTRL_PWM_EN);

                if (pinOutVal == SDL_ESM_LVL_PINOUT)
                {
                    retVal = SDL_PASS;
                }
            }
            else
            {
                /* For MISRA C Compliance */
            }
            /* global interrupt are disabled, enable again global interrupt */
            sdlRet = SDL_ESM_enableGlobalIntr(baseAddr);

            if (sdlRet == SDL_PASS)
            {
                sdlRet = SDL_ESM_getGlobalIntrEnabledStatus(baseAddr, &intStatus);
            }
            if (sdlRet == SDL_PASS)
            {
                if (intStatus != SDL_ESM_EN_KEY_ENBALE_VAL)
                {
                    sdlRet = SDL_EFAIL;
                }
            }
            if (sdlRet != SDL_PASS)
            {
                retVal = SDL_EFAIL;
            }
        }
        else
        {
            retVal = SDL_EFAIL;
        }
    }

    return (retVal);
}