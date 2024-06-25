/*
 * SDL ESM
 *
 * Software Diagnostics Reference module for Error Signaling Module
 *
 *  Copyright (c) Texas Instruments Incorporated 2022-2023
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
#include <stddef.h>
#include <stdbool.h>
#include <sdl/esm/v0/sdl_esm.h>
#include <sdl/esm/soc/sdl_esm_soc.h>
#include "sdl_esm_priv.h"

/* Local Defines */
#define BITS_PER_WORD (32u)
#define ESM_INTR_GRP_NUM (32U)
#define GROUP_NUMBER_BIT_SHIFT  (5u)
#define NO_EVENT_VALUE (0xffffu)
#define SDL_ESM_EN_KEY_ENBALE_VAL (0xFU)
#define INVALID_BIT (0u)
#define FLAG_NO (0u)
#define FLAG_YES (1u)
#define MASK_BIT (1u)

/* Local functions  */
static void SDL_ESM_interruptHandler (uint32_t esmInstBaseAddr,
                                      esmIntrPriorityLvl_t esmIntrPriorityLvlType,
                                      void *arg);
static void SDL_ESM_selfTestCallback(SDL_ESM_Instance_t *esmInst);
static inline void SDL_ESM_getGroupNumberIndex(uint32_t intSrc,
                                               uint32_t *groupNumber,
                                               uint32_t *intIndex);
static void SDL_ESM_processInterruptSource(uint32_t esmInstBaseAddr,
                                           SDL_ESM_IntType esmIntType,
                                           uint32_t intSrc, void *arg);

/** ============================================================================
 *
 * \brief   Call back for ESM test error
 *
 * \param   esmInst: ESM instance for the callback
 *
 * \return  None
 */
static void SDL_ESM_selfTestCallback(SDL_ESM_Instance_t *esmInst)
{
    esmInst->selfTestFlag  = (bool)true;

    return;
}

/** ============================================================================
 *
 * \brief   Get the Group number and Index for the given interrrupt
 *          source
 *
 * \param2  registered argument
 * \param2  Interrupt source
 *
 * \return  None
 */
static inline void SDL_ESM_getGroupNumberIndex(uint32_t intSrc, uint32_t *groupNumber,
                                    uint32_t *intIndex)
{
    *groupNumber = intSrc >> GROUP_NUMBER_BIT_SHIFT;
    *intIndex = intSrc-((*groupNumber) << GROUP_NUMBER_BIT_SHIFT);

    return;
}

/** ============================================================================
 *
 * \brief   Process Interrupt source and call callback function
 *
 * \param2  registered argument
 * \param2  Interrupt source
 *
 * \return  None
 */
static void SDL_ESM_processInterruptSource(uint32_t esmInstBaseAddr,
                                           SDL_ESM_IntType esmIntType,
                                           uint32_t intSrc, void *arg)
{
    SDL_ESM_Instance_t *SDL_ESM_instance;
    SDL_ESM_Inst esmInstType = (SDL_ESM_Inst)(uint32_t)arg;
    uint32_t groupNumber, intIndex;
    int32_t isHandled = (int32_t)FLAG_NO;

    SDL_ESM_selectEsmInst(esmInstType,&SDL_ESM_instance);

    if (intSrc != NO_EVENT_VALUE) {
        if (intSrc < (BITS_PER_WORD*SDL_ESM_MAX_EVENT_MAP_NUM_WORDS)) {
            SDL_ESM_getGroupNumberIndex(intSrc, &groupNumber, &intIndex);
            if((SDL_ESM_instance->esmInitConfig.enableBitmap[groupNumber]
               & (((uint32_t)MASK_BIT)<<intIndex)) != INVALID_BIT) {
                /* Check if this is due to self test */
                if((groupNumber
                    == SDL_ESM_instance->esmInitConfig.esmErrorConfig.groupNumber) &&
                   (intIndex ==  SDL_ESM_instance->esmInitConfig.esmErrorConfig.bitNumber)){
                    SDL_ESM_selfTestCallback(SDL_ESM_instance);
                    isHandled = (int32_t)FLAG_YES;
                }
                else if((SDL_ESM_instance->eccenableBitmap[groupNumber]
                        & (((uint32_t)MASK_BIT)<<intIndex)) != INVALID_BIT) {
                    isHandled = SDL_ESM_instance->eccCallBackFunction(esmInstType, esmIntType,
                                                                      groupNumber, intIndex,
                                                                      intSrc, SDL_ESM_instance->eccCallBackFunctionArg);
                }
                else if((SDL_ESM_instance->ccmenableBitmap[groupNumber]
                        & (((uint32_t)MASK_BIT)<<intIndex)) != INVALID_BIT) {
                    isHandled = SDL_ESM_instance->ccmCallBackFunction(esmInstType, esmIntType,
                                                                      groupNumber, intIndex,
                                                                      intSrc, SDL_ESM_instance->ccmCallBackFunctionArg);
                }
                else
                {
                    isHandled = (int32_t)FLAG_NO;
                }
            }
            if (isHandled != (int32_t)FLAG_YES)
            {
                (void)SDL_ESM_instance->callback(esmInstType, esmIntType,
                                                 groupNumber, intIndex,
                                                 intSrc, SDL_ESM_instance->arg);
            }
            (void)SDL_ESM_clearIntrStatus(esmInstBaseAddr, intSrc);
        }
    }
    return;
}

/** ============================================================================
 *
 * \brief   Interrupt handler for Hi ESM interrupt
 *
 * \param1  esmInstType: Instance of ESM
 * \param2  esmIntrPriorityLvlType: Interrupt priority level
 * \param3  arg: Argument to callback function
 *
 * \return  None
 */
/*
 * Design: PROC_SDL-1048
 */
static void SDL_ESM_interruptHandler (uint32_t esmInstBaseAddr,
                                      esmIntrPriorityLvl_t esmIntrPriorityLvlType,
                                      void *arg)
{
    uint32_t intSrc1, intSrc2;
    esmGroupIntrStatus_t localEsmGroupIntrStatus;
    SDL_ESM_IntType esmIntType;
    uint32_t base_addr;
    SDL_ESM_Inst esm_inst;
    bool isCfgIntr;

    if (esmIntrPriorityLvlType == ESM_INTR_PRIORITY_LEVEL_HIGH) {
        esmIntType = SDL_ESM_INT_TYPE_HI;
    }
    else {
        esmIntType = SDL_ESM_INT_TYPE_LO;
    }
    /* Check on the highest priority event and handle it */
    do {
        (void)SDL_ESM_getGroupIntrStatus((uint32_t)esmInstBaseAddr,
                                         (uint32_t)esmIntrPriorityLvlType,
                                         &localEsmGroupIntrStatus);
        intSrc1 = localEsmGroupIntrStatus.highestPendPlsIntNum;
        SDL_ESM_processInterruptSource((uint32_t)esmInstBaseAddr, esmIntType, intSrc1, arg);
        intSrc2 = localEsmGroupIntrStatus.highestPendLvlIntNum;

        SDL_ESM_processInterruptSource((uint32_t)esmInstBaseAddr, esmIntType, intSrc2, arg);

        if (SDL_ESM_checkSpecialEvent(esmInstBaseAddr, (uint32_t)esmIntrPriorityLvlType, &base_addr,
                                      &esm_inst, &isCfgIntr) == true)
        {
            break;
        }
    } while ((intSrc1 != (uint32_t)(NO_EVENT_VALUE)) || (intSrc2 != (uint32_t)(NO_EVENT_VALUE)));

    return;
}

static void SDL_ESM_handleCfgIntr(uint32_t baseAddr, SDL_ESM_Inst esmInstType)
{
    SDL_ESM_Instance_t *SDL_ESM_instance;
    uint32_t group;
    uint32_t status;

    SDL_ESM_selectEsmInst(esmInstType, &SDL_ESM_instance);

    /* Get the proper ESM SW instance here based on esmInstBaseAddr register base */
    for (group = 0; group < ESM_INTR_GRP_NUM; group++)
    {
        (void)SDL_ESM_getCfgIntrStatus(baseAddr, group, &status);
        if (status == 0x1u)
        {
            break;
        }
    }

    (void)SDL_ESM_instance->callback(esmInstType,
                                     SDL_ESM_INT_TYPE_CFG,
                                     group, 0, 0, SDL_ESM_instance->arg);

    /* Clear the pending interrupt(s) */
    (void)SDL_ESM_clearCfgIntrStatus(baseAddr, group);

    return;
}

/** ============================================================================
 *
 * \brief   Esm Hi Interrupt Handler for MCU ESM Instance
 *
 * \param1  arg: argument for handler
 *
 * \return  None
 */
/*
 * Design: PROC_SDL-1047
 */
void SDL_ESM_hiInterruptHandler (void *arg)
{
    uint32_t esm_base_addr;
    /*Passing the instance and using that to get the base_addr*/
    SDL_ESM_Inst instance = (SDL_ESM_Inst)(uint32_t)arg;
    uint32_t base_addr = 0x0;
    SDL_ESM_Inst esm_inst;
    bool isCfgIntr = (bool)false;

    SDL_ESM_getBaseAddr(instance, &esm_base_addr);

    if (SDL_ESM_checkSpecialEvent(esm_base_addr, ESM_INTR_PRIORITY_LEVEL_HIGH, &base_addr, &esm_inst, &isCfgIntr) == true)
    {
        if (isCfgIntr == (bool)true)
        {
            SDL_ESM_handleCfgIntr(base_addr, esm_inst);
        }
        else
        {
            SDL_ESM_interruptHandler((uint32_t)base_addr, ESM_INTR_PRIORITY_LEVEL_HIGH, (void *)esm_inst);
        }
    }
    /* Call common Interrupt handler */
    SDL_ESM_interruptHandler((uint32_t)esm_base_addr, ESM_INTR_PRIORITY_LEVEL_HIGH, arg);

    /* Write end of interrupt */
    (void)SDL_ESM_writeEOI(((uint32_t)esm_base_addr), (uint32_t)ESM_INTR_TYPE_HIGH_PRIO_ERROR);

    return ;
}


/** ============================================================================
 *
 * \brief   Esm Lo Interrupt Handler for MCU ESM Instance
 *
 * \param1  arg: argument for handler
 *
 * \return  None
 */
/*
 * Design: PROC_SDL-1047
 */
void SDL_ESM_loInterruptHandler (void *arg)
{
    uint32_t esm_base_addr;
    /*Passing the instance and using that to get the base_addr*/
    SDL_ESM_Inst instance = (SDL_ESM_Inst)(uint32_t)arg;
    uint32_t base_addr = 0x0;
    SDL_ESM_Inst esm_inst;
    bool isCfgIntr = (bool)false;

    SDL_ESM_getBaseAddr(instance, &esm_base_addr);

    if (SDL_ESM_checkSpecialEvent(esm_base_addr, ESM_INTR_PRIORITY_LEVEL_LOW, &base_addr, &esm_inst, &isCfgIntr) == true)
    {
        if (isCfgIntr == (bool)true)
        {
            SDL_ESM_handleCfgIntr(base_addr, esm_inst);
        }
        else
        {
            SDL_ESM_interruptHandler((uint32_t)base_addr, ESM_INTR_PRIORITY_LEVEL_LOW, arg);
        }
    }
    /* Call common Interrupt handler */
    SDL_ESM_interruptHandler((uint32_t)esm_base_addr, ESM_INTR_PRIORITY_LEVEL_LOW, arg);

    /* Write end of interrupt */
    (void)SDL_ESM_writeEOI(((uint32_t)esm_base_addr), (uint32_t)ESM_INTR_TYPE_LOW_PRIO_ERROR);
    return;
}


/** ============================================================================
 *
 * \brief   Esm Config Interrupt Handler for MCU Instance
 *
 * \param1  arg: argument for handler
 *
 * \return  None
 */
/*
 * Design: PROC_SDL-1047
 */
void SDL_ESM_configInterruptHandler(void *arg)
{
    SDL_ESM_Instance_t *SDL_ESM_instance;
    SDL_ESM_Inst esmInstType;
    uint32_t esm_base_addr;
    esmInstType = (SDL_ESM_Inst)(uint32_t)arg;
    esm_base_addr = 0x0;
    SDL_ESM_selectEsmInst(esmInstType,&SDL_ESM_instance);
    SDL_ESM_getBaseAddr(esmInstType, &esm_base_addr);

    SDL_ESM_handleCfgIntr(esm_base_addr, esmInstType);

    /* Write end of interrupt */
    (void)SDL_ESM_writeEOI(((uint32_t)esm_base_addr), (uint32_t)ESM_INTR_TYPE_CONFIG_ERROR);

    return;
}
