/********************************************************************
 * Copyright (C) 2022 Texas Instruments Incorporated.
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

#ifndef SDL_IP_HWA_H_
#define SDL_IP_HWA_H_

#include <stddef.h>
#include <stdbool.h>
#include "sdl_hwa.h"
#ifdef __cplusplus
extern "C" {
#endif


/**
 *  \defgroup SDL_IP_SDL_API API's for HWA Low-Level
 *  \ingroup SDL_HWA_MODULE
 *
 *  This module contains the Low-Level APIs to program and use the HWA module.
 *
 *  @{
 */

/**
 * \brief  Macro's that can be for Parity Error Check APIs
 */
/* Enable HWA */
#define SDL_HWA_ENABLE_STS                      (1U)
/* Disable HWA */
#define SDL_HWA_DISABLE_STS                     (0U)
/* Parity for HWA Data memories */
#define SDL_HWA_DMEM_PARITY                     (1U)
/* Parity for  Window RAM memories */
#define SDL_HWA_DMEM_WINDOW_RAM_PARITY          (2U)
/* HWA FSM Lockstep  Enable Inverted */
#define SDL_HWA_DMEM_FSM_LOCKSTEP_INV_EN        (3U)
/* HWA FSM Lockstep  Enable */
#define SDL_HWA_DMEM_FSM_LOCKSTEP_EN            (4U)

/**
 * \brief  The status for HWA accelerator and memory block clock status. These
 *         status returned in #SDL_HWA_getHwaEnableDisable function
 */

typedef struct Sdl_HWA_Status_t
{
    /* Clock status for hwa memory block */
    uint8_t SDL_HWA_enableHwaClkSTS;
    /* HWA accelerator status for memory block */
    uint8_t SDL_HWA_enableHwaSTS;
}SDL_HWA_Status_s;

/* ========================================================================== */
/*                         Function Declarations                              */
/* ========================================================================== */
/**
 * \brief   This API is used to enable or disable the parity for memory block
 *          and lockstep logic
 *
 * \param   memBlockParity  HWA memory block parity
 *
 * \param   enableSts       parity status for HWA memory block
 *                          SDL_HWA_ENABLE enable parity
 *                          SDL_HWA_DISABLE disable parity
 *
 */
void SDL_HWA_setParityEnableDisable(uint8_t memBlockParity,uint8_t enableSts);
/**
 * \brief   This API is used to get status for parity for memory block
 *
 * \param   memBlockParity  HWA memory block parity
 *
 * \return  paritySts       parity status for HWA memory block
 *                          SDL_HWA_ENABLE enable parity
 *                          SDL_HWA_DISABLE disable parity
 *
 */
uint8_t SDL_HWA_getParityStatus(uint8_t memBlockParity);

/**
 * \brief   This API is used to enable/disable Radar Hardware Accelerator
 *          and respective clock
 *
 * \param   enableSts       status for enable/disable Radar Hardware Accelerator
 *                          and respective clock
 *                          SDL_HWA_ENABLE enable
 *                          Radar Hardware Accelerator and respective clock
 *                          SDL_HWA_DISABLE disable
 *                          Radar Hardware Accelerator and respective clock
 *
 */
void SDL_HWA_setHwaEnableDisable(uint8_t enableSts);

/**
 * \brief   This API is used to get enable/disable Radar Hardware Accelerator
 *          and respective clock
 *
 * \param   pHWAStats  pointer to get the status of Radar Hardware Accelerator
 *                     and clock of type SDL_HWA_Status_s
 *
 */
void SDL_HWA_getHwaEnableDisable(SDL_HWA_Status_s *pHWAStats);

/**
 * \brief   This API is used to initialize the memory
 *
 * \param   memBlock  indicates the memory block to be initialized
 *                    and of type SDL_HWA_MemBlock
 *
 */
void SDL_HWA_memoryInitStart(SDL_HWA_MemBlock memBlock);

/**
 * \brief   This API is used to check the initialize done status
 *
 * \param   memBlock  indicates the memory block whose done status
 *                    to be checked and of type SDL_HWA_MemBlock
 *
 * \return  status    init done status
 *
 */
uint8_t SDL_HWA_memoryInitDone(SDL_HWA_MemBlock memBlock);

/**
 * \brief   This API is used to check the initialize status
 *
 * \param   memBlock  indicates the memory block whose status
 *                    to be checked and of type SDL_HWA_MemBlock
 *
 * \return  status    init status
 *
 */
uint8_t SDL_HWA_memoryInitStatus(SDL_HWA_MemBlock memBlock);

/**
 * \brief   This API is used to mask/unmask parity error status
 *
 * \param   memBlock  indicates the memory block whose status
 *                    to be checked and of type SDL_HWA_MemBlock
 * \param   enableSts status for Masking the error status
 *                    SDL_HWA_ENABLE mask the error status
 *                    others unmask the error status
 *
 */
void SDL_HWA_setParityErrMaskUnmask(SDL_HWA_MemBlock memBlock, uint8_t enableSts);

/**
 * \brief   This API is used get  mask/unmask parity error status
 *
 * \param   memBlock  indicates the memory block whose status
 *                    to be checked and of type SDL_HWA_MemBlock
 *
 * \return  status    status for Masking the error status
 *
 */
uint8_t SDL_HWA_getParityErrMaskUnmask(SDL_HWA_MemBlock memBlock);

/**
 * \brief   This API is used to get error status
 *
 * \param   memBlock  indicates the memory block and of type SDL_HWA_MemBlock
 *
 * \return  status    memory block error status
 */
uint8_t SDL_HWA_getErrStatus(SDL_HWA_MemBlock memBlock);
/**
 * \brief   This API is used to clear error status
 *
 * \param   memBlock  indicates the memory block and of type SDL_HWA_MemBlock
 *
 */
void SDL_HWA_clearErrStatus(SDL_HWA_MemBlock memBlock);

#ifdef __cplusplus
}
#endif
#endif /* SDL_IP_HWA_H_ */

/** @} */
