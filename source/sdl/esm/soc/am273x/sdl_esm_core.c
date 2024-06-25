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

#include "sdl_esm_core.h"
/*
 * Design: PROC_SDL-1068
 */
#define SDL_ESM0_CFG_SIZE (1000U)

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
            case SDL_ESM_INST_MSS_ESM:
                instValid = ((bool)true);
                *esmBaseAddr = SDL_MSS_ESM_U_BASE ;
                size = SDL_ESM0_CFG_SIZE;
                break;
			case SDL_ESM_INST_DSS_ESM:
                instValid = ((bool)true);
                *esmBaseAddr = SDL_DSS_ESM_U_BASE ;
                size = SDL_ESM0_CFG_SIZE;
                break;
			case SDL_ESM_INST_HSM_ESM:
                instValid = ((bool)true);
                *esmBaseAddr = SDL_HSM_ESM_U_BASE ;
                size = SDL_ESM0_CFG_SIZE;
                break;
            default:
                break;
        }
    }

    *esmBaseAddr = (uint32_t)SDL_DPL_addrTranslate(*esmBaseAddr, size);

    return (instValid);
}

/* Nothing past this point */
