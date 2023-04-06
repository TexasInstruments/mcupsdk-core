/*
 * @file  sdl_ip_pok.h
 *
 * @brief
 *  C implementation interface header file for the POK module SDL-FL.
 *
 *  Translates POK ID to POK Address. This is a SOC specific source file.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2023, Texas Instruments, Inc.
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
#ifndef SDL_IP_POK_H
#define SDL_IP_POK_H

#ifdef __cplusplus
    extern "C" {
#endif





#include <sdl/pok/v1/sdl_pok_def.h>
#include <sdl/pok/v1/soc/sdl_soc_pok.h>

#if defined (SOC_AM243X)
#if defined (R5F_CORE)
#include <sdl/pok/v1/sdl_pok_def.h>
#include <sdl/pok/v1/soc/am243x/sdl_soc_pok.h>
#endif
#endif



/**
 *
 * @ingroup  SDL_IP_MODULE
 * @defgroup SDL_IP_POK_API POK Low-Level API
 * Provides the APIs for POK.
 *
 */

/**
@defgroup SDL_IP_POK_FUNCTION  POK Functions
@ingroup SDL_IP_POK_API
*/



int32_t SDL_pok_GetShiftsAndMasks(SDL_mcuCtrlRegsBase_t     *pBaseAddress,
                                     SDL_POK_Inst             instance,
                                     SDL_pokShiftsAndMasks_t *pShMasks);
/**
 *  \addtogroup SDL_IP_POK_FUNCTION
 *  @{
 */
/* This function set the POK register values */
/**
 *  \brief write the POK configuration for the specified POK control register
 *
 *         This API supports the enable/disable of the POK hysterisis and
 *         Voltage Detection for a given POK control register
 *
 *  \param pBaseAddress      [IN]     Pointer to the Wakeup Control Register
 *  \param pPokCfg           [IN]     Pointer to the POK control register to be written
 *  \param instance          [IN]     POK ID of which POK to be updated
 *
 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Fail         : SDL_EFAIL
 *                                 Invalid Args : SDL_EBADARGS
 */


int32_t SDL_pokSetControl (SDL_mcuCtrlRegsBase_t         *pBaseAddress,
                           const SDL_POK_config           *pPokCfg,
                           SDL_POK_Inst                    instance);
/**
 *  \brief Read the POK configuration for the specified POK control register
 *
 *         This API supports the enable/disable of the POK hysterisis and
 *         Voltage Detection for a given POK control register
 *
 *  \param pBaseAddress      [IN]     Pointer to the Wakeup Control Register
 *  \param pPokCfg           [IN]     Pointer to the POK control register values to be read
 *  \param pPokVal           [OUT]    Pointer to the POK control register to be read
 *  \param instance          [IN]     POK ID of which POK to be updated
 *
 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Fail         : SDL_EFAIL
 *                                 Invalid Args : SDL_EBADARGS
 */

int32_t SDL_pokGetControl   (SDL_mcuCtrlRegsBase_t           *pBaseAddress,
                             const SDL_POK_config             *pPokCfg,
                             SDL_pokVal_t                     *pPokVal,
                             SDL_POK_Inst                      instance);

/**
 *  \brief write the POR configuration for the specified POK control register
 *
 *         This API supports the enable/disable of the POK hysterisis and
 *         Voltage Detection for a given POK control register
 *
 *  \param pBaseAddress      [IN]    Pointer to the Wakeup Control Register
 *  \param pPorCfg           [IN]    Pointer to the POR configuration
 *
 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Fail         : SDL_EFAIL
 *                                 Invalid Args : SDL_EBADARGS
 */

int32_t SDL_porSetControl (SDL_mcuCtrlRegsBase_t           *pBaseAddress,
                           const SDL_pokPorCfg_t            *pPorCfg);


/* @} */

#ifdef __cplusplus
}
#endif
#endif /* SDL_IP_POK_H */
/* Nothing past this point */
