/*
 * SDL ESM
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

#include <stdbool.h>
#include <stddef.h>

#include <sdl/esm/soc/sdl_esm_soc.h>
#include <sdl/esm/v0/sdl_esm.h>
#include "sdl_esm_core.h"
#include <sdl/esm/v0/v0_0/sdl_esm_priv.h>
#include <sdl/dpl/sdl_dpl.h>


/*
 * Design: PROC_SDL-1068
 */
static SDL_ESM_Instance_t SDL_ESM_instance_MCU;
static SDL_ESM_Instance_t SDL_ESM_instance_MAIN;

/** ================================================================================
 *
 * \brief        Check that ESM instance type is valid for this device, and fill the
 *               ESM base address
 *
 * \param [in]   esmInstType: ESM instance type
 * \param [out]  esmBaseAddr: Base address for ESM instance's registers
 *
 * \return       true: if valid instance type; false if not valid instance type
 */
bool SDL_ESM_getBaseAddr(const SDL_ESM_Inst esmInstType, uint32_t *esmBaseAddr)
{
    bool instValid = ((bool)false);
    uint32_t size = 0u;

    if (esmBaseAddr != NULL)
    {
        switch(esmInstType)
        {
            case SDL_ESM_INST_MCU_ESM0:
                instValid = ((bool)true);
                *esmBaseAddr = SOC_MCU_ESM_BASE;
                size = SDL_MCU_ESM0_CFG_SIZE;
                break;

            case SDL_ESM_INST_MAIN_ESM0:
                instValid = ((bool)true);
                *esmBaseAddr = SOC_MAIN_ESM_BASE;
                size = SDL_ESM0_CFG_SIZE;
                break;

            default:
                break;
        }
	    *esmBaseAddr = (uint32_t)SDL_DPL_addrTranslate(*esmBaseAddr, size);
    }
    return (instValid);
}

/** ================================================================================
 *
 * \brief        Get the max number of ESM events supported for a given ESM instance
 *
 * \param [in]   esmInstType: ESM instance type
 * \param [out]  esmMaxNumEvents: Maximum number of ESM events supported
 *
 * \return       true: if valid instance type; false if not valid instance type
 */
bool SDL_ESM_getMaxNumEvents(const SDL_ESM_Inst esmInstType,
                             uint32_t *esmMaxNumEvents)
{
    bool instValid = ((bool)false);

    if (esmMaxNumEvents != NULL)
    {
        switch(esmInstType)
        {
            case SDL_ESM_INST_MCU_ESM0:
                instValid = ((bool)true);
                *esmMaxNumEvents = SOC_MCU_ESM_MAX_NUM_EVENTS;
                break;

            case SDL_ESM_INST_MAIN_ESM0:
                instValid = ((bool)true);
                *esmMaxNumEvents = SOC_MAIN_ESM_MAX_NUM_EVENTS;
                break;

            default:
                break;
        }
    }

    return (instValid);
}

/** ================================================================================
 *
 * \brief        Check that ESM instance type or ESM base address is valid for this
 *               device, and fill the SDL_ESM instance
 *
 * \param [in]   esmInstType:     ESM instance type
 * \param [out]  pEsmInstancePtr: Pointer to location of ESM instance structure
 *
 * \return       true: if valid instance type; false if not valid instance type
 */
bool SDL_ESM_selectEsmInst(const SDL_ESM_Inst esmInstType,
                           SDL_ESM_Instance_t **pEsmInstancePtr)
{
    bool instValid = ((bool)true);

    switch(esmInstType)
    {
        case SDL_ESM_INST_MCU_ESM0:
            *pEsmInstancePtr = (SDL_ESM_Instance_t *)(&SDL_ESM_instance_MCU);
            break;

        case SDL_ESM_INST_MAIN_ESM0:
            *pEsmInstancePtr = (SDL_ESM_Instance_t *)(&SDL_ESM_instance_MAIN);
            break;

        default:
            /* Invalid instance input parameter */
            instValid = ((bool)false);
            break;
    }

    return (instValid);
}

/** ============================================================================
 *
 * \brief   Esm get Interrupt Number corresponding to the
 *          input interrupt type
 *
 * \param   esmInstType: Instance of ESM
 * \param   esmIntType: Interrupt type
 *
 * \return  Interrupt Number, SDL_ESM_INST_INVALID,
 *          or SDL_ESM_INTNUMBER_INVALID error
 */
int32_t SDL_ESM_getIntNumber(const SDL_ESM_Inst esmInstType,
                              SDL_ESM_IntType esmIntType)
{
    uint32_t intNum = SDL_ESM_INST_INVALID;
#if defined (M4F_CORE)
    if (esmInstType == SDL_ESM_INST_MCU_ESM0) {
        switch(esmIntType)
        {
            case SDL_ESM_INT_TYPE_HI:
                intNum = SDL_MCU_ESM_HI_INTNO;
                break;

            case SDL_ESM_INT_TYPE_CFG:
                intNum = SDL_MCU_ESM_CFG_INTNO;
                break;

            case SDL_ESM_INT_TYPE_LO:
                intNum = SDL_MCU_ESM_LO_INTNO;
                break;

            default:
                intNum = SDL_ESM_INTNUMBER_INVALID;
                break;
        }
    }
    else
    {
        intNum = SDL_ESM_INTNUMBER_INVALID;
    }
#endif
#if defined (R5F_CORE)
    if (esmInstType == SDL_ESM_INST_MAIN_ESM0) {
        switch(esmIntType)
        {
            case SDL_ESM_INT_TYPE_HI:
                intNum = SDL_MAIN_ESM_HI_INTNO;
                break;

            case SDL_ESM_INT_TYPE_CFG:
                intNum = SDL_MAIN_ESM_CFG_INTNO;
                break;

            case SDL_ESM_INT_TYPE_LO:
                intNum = SDL_MAIN_ESM_LO_INTNO;
                break;

            default:
                intNum = SDL_ESM_INTNUMBER_INVALID;
                break;
        }
    }
    else
    {
        intNum = SDL_ESM_INTNUMBER_INVALID;
    }
#endif
    return (int32_t)intNum;
}

/** ============================================================================
 *
 * \brief   Interrupt Checker for ESM interrupts
 *
 * \param2  registered argument
 * \param2  esmIntrPriorityLvlType: Interrupt priority level
 *
 * \return  True/False
 */
bool SDL_ESM_checkSpecialEvent(uint32_t esm_base_addr, uint32_t priority, uint32_t *base_addr, SDL_ESM_Inst *esmInst, bool *isCfgEvt)
{
    bool ret = (bool)false;
    uint32_t intSrc;
    esmGroupIntrStatus_t localEsmGroupIntrStatus;

    *isCfgEvt = (bool)false;
      #if defined (M4F_CORE)
    SDL_ESM_getBaseAddr(SDL_ESM_INST_MAIN_ESM0, base_addr);
    if (*base_addr != esm_base_addr)
    {
        (void)SDL_ESM_getGroupIntrStatus(esm_base_addr, priority, &localEsmGroupIntrStatus);
        intSrc = localEsmGroupIntrStatus.highestPendLvlIntNum;
        if ((intSrc == SDLR_MCU_ESM0_ESM_LVL_EVENT_ESM0_ESM_INT_HI_LVL_0) ||
            (intSrc == SDLR_MCU_ESM0_ESM_LVL_EVENT_ESM0_ESM_INT_LOW_LVL_0) ||
            (intSrc == SDLR_MCU_ESM0_ESM_LVL_EVENT_ESM0_ESM_INT_CFG_LVL_0))
        {
            SDL_ESM_getBaseAddr(SDL_ESM_INST_MAIN_ESM0, base_addr);
            *esmInst = SDL_ESM_INST_MAIN_ESM0;
            if (intSrc == SDLR_MCU_ESM0_ESM_LVL_EVENT_ESM0_ESM_INT_CFG_LVL_0)
            {
                *isCfgEvt = (bool)true;
            }
            ret = (bool)true;
        }
    }
    #endif

    #if defined (R5F_CORE)
    SDL_ESM_getBaseAddr(SDL_ESM_INST_MCU_ESM0, base_addr);
    if (*base_addr != esm_base_addr)
    {
        (void)SDL_ESM_getGroupIntrStatus(esm_base_addr, priority, &localEsmGroupIntrStatus);
        intSrc = localEsmGroupIntrStatus.highestPendLvlIntNum;
        if ((intSrc == SDLR_ESM0_ESM_LVL_EVENT_MCU_ESM0_ESM_INT_HI_LVL_0) ||
            (intSrc == SDLR_ESM0_ESM_LVL_EVENT_MCU_ESM0_ESM_INT_LOW_LVL_0) ||
            (intSrc == SDLR_ESM0_ESM_LVL_EVENT_MCU_ESM0_ESM_INT_CFG_LVL_0))
        {
            SDL_ESM_getBaseAddr(SDL_ESM_INST_MCU_ESM0, base_addr);
            *esmInst = SDL_ESM_INST_MCU_ESM0;
            if (intSrc == SDLR_ESM0_ESM_LVL_EVENT_MCU_ESM0_ESM_INT_CFG_LVL_0)
            {
                *isCfgEvt = (bool)true;
            }
            ret = (bool)true;
        }
    }
    #endif
    return ret;
}
