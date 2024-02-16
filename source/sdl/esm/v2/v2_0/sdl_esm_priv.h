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

#ifndef INCLUDE_SDL_ESM_PRIV_H_
#define INCLUDE_SDL_ESM_PRIV_H_

#include <stddef.h>
#include <stdbool.h>

#include <sdl/esm/v2/v2_0/sdl_ip_esm.h>
#include <sdl/esm/v2/sdl_esm.h>
#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
     /**< selfTest Flag */
     volatile bool selfTestFlag;
     /**< Store the ECC callback function arg */
     void *eccCallBackFunctionArg;
     /**< Store the CCM callback function arg */
     void *ccmCallBackFunctionArg;
     /**< Store application argument */
     void *arg;
     uint32_t eccenableBitmap[SDL_ESM_MAX_EVENT_MAP_NUM_WORDS];
     uint32_t ccmenableBitmap[SDL_ESM_MAX_EVENT_MAP_NUM_WORDS];
     SDL_ESM_config esmInitConfig;
     /**< Store ECC callback function */
     SDL_ESM_applicationCallback eccCallBackFunction;
     /**< Store the CCM callback function */
     SDL_ESM_applicationCallback ccmCallBackFunction;
     /**< Store applicataion callback function */
     SDL_ESM_applicationCallback callback;
}SDL_ESM_Instance_t;


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
bool SDL_ESM_getBaseAddr(SDL_ESM_Inst esmInstType, uint32_t *esmBaseAddr);

/** ================================================================================
 *
 * \brief        Check that ESM instance type is valid for this device, and fill the
 *               SDL_ESM instance
 *
 * \param [in]   esmInstType: ESM instance type
 * \param [out]  pEsmInstancePtr: Pointer to location of ESM instance structure
 *
 * \return       true: if valid instance type; false if not valid instance type
 */
bool SDL_ESM_selectEsmInst(SDL_ESM_Inst esmInstType,
                           SDL_ESM_Instance_t **pEsmInstancePtr);

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
                             uint32_t *esmMaxNumEvents);

/** ============================================================================
 *
 * \brief   Check for special ESM events that need to be handled by a differen
 *          ESM instance for this SoC
 *
 * \param [in]   ESM instance base address to check for special event
 * \param [in]   ESM event priority
 * \param [out]  Base address of ESM instance to use to handle the event
 * \param [out]  ESM instance to use to handle the event
 * \param [out]  Indicates if the event is a ESM Config error or not
 *
 * \return       true: if special event did occur. base_addr, esmInst and isCfgEvt
 *                     will be valid in this case
 *               false: no special event occurred. base_addr is not valid.
 */
bool SDL_ESM_checkSpecialEvent(uint32_t esm_base_addr, uint32_t priority,
                               uint32_t *base_addr, SDL_ESM_Inst *esmInst, bool *isCfgEvt);

#ifdef __cplusplus
}
#endif
#endif /* INCLUDE_SDL_ESM_PRIV_H_ */
