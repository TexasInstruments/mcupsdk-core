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
 *  @ingroup SDL_HWA_MODULE
 *  @defgroup SDL_HWA_API API's for HWA
 *  @section HWA Overview
 *         HWA Test API's:
 *         1. SDL_HWA_DMA0_secExecute()
 *         2. SDL_HWA_DMA1_secExecute()
 *         3. SDL_HWA_DMA0_dedExecute()
 *         4. SDL_HWA_DMA1_dedExecute()
 *         5. SDL_HWA_DMA0_redExecute()
 *         6. SDL_HWA_DMA1_redExecute()
 *         7. SDL_HWA_memParityExecute()
 *         8. SDL_HWA_fsmLockStepExecute()
 *
 *  @{
 */
/**
 *  \file     sdl_hwa.h
 *
 *  \brief    This file contains the prototypes of the APIs present in the
 *            device abstraction layer file of HWA.
 *            This also contains some related macros.
 */
#ifndef SDL_HWA_H_
#define SDL_HWA_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "sdl_hwa_hw.h"
#include <sdl/hwa/v0/soc/sdl_hwa_soc.h>

#ifdef _cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
typedef enum
{
    SDL_HWA_FI_MAIN =0,
    SDL_HWA_FI_SAFE = 1,
    SDL_HWA_FI_GLOBAL_MAIN =2,
    SDL_HWA_FI_GLOBAL_SAFE=3,
    SDL_HWA_FI_INVALID = 4,
} SDL_HWA_busSftyFiType;

typedef enum
{
    SDL_HWA_MAIN_CMD_INTERFACE =0,
    SDL_HWA_MAIN_WRITE_INTERFACE =1,
    SDL_HWA_MAIN_WRITE_STATUS_INTERFACE=2,
    SDL_HWA_MAIN_READ_INTERFACE =3,
    SDL_HWA_FI_TYPE_INVALID =4,
}SDL_HWA_busSftyFiRedType;

/* ========================================================================== */
/*                         Variable Declarations                             */
/* ========================================================================== */
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * \brief   This API is used  for SEC test on DMA0
 *
 */
void SDL_HWA_DMA0_secExecute(void);

/**
 * \brief   This API is used  for SEC test on DMA1
 *
 */
void SDL_HWA_DMA1_secExecute(void);

/**
 * \brief   This API is used  for DED test on DMA0
 *
 */

void SDL_HWA_DMA0_dedExecute(void);

/**
 * \brief   This API is used  for DED test on DMA1
 *
 */

void SDL_HWA_DMA1_dedExecute(void);

/**
 * \brief   This API is used  for RED test on DMA0
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 *                    SDL_EFAIL:    failure, indicate verify failed
 *
 */
int32_t SDL_HWA_DMA0_redExecute(SDL_HWA_busSftyFiType fiType, SDL_HWA_busSftyFiRedType redType);
/**
 * \brief   This API is used  for RED test on DMA1
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_HWA_DMA1_redExecute(SDL_HWA_busSftyFiType fiType, SDL_HWA_busSftyFiRedType redType);

/**
 * \brief   This API is used to clear SEC error from DMA0
 *
 */
void SDL_HWA_DMA0_secErrorClear(void);

 /**
 * \brief   This API is used to clear SEC error from DMA1
 *
 */
void SDL_HWA_DMA1_secErrorClear(void);

 /**
 *  \brief   This API is used to clear DED error from DMA0
 *
 *
 */
void SDL_HWA_DMA0_dedErrorClear(void);

 /**
 *  \brief   This API is used to clear DED error from DMA1
 *
 */
void SDL_HWA_DMA1_dedErrorClear(void);

 /**
 *  \brief   This API is used to clear RED error from DMA0
 *
 */
void SDL_HWA_DMA0_redErrorClear(void);

 /**
 *  \brief   This API is used to clear red error from DMA1
 *
 */
 void SDL_HWA_DMA1_redErrorClear(void);

/**
 * \brief   This API is used to get SEC error status from DMA0
 *
 */
uint32_t SDL_HWA_DMA0_secErrorStatus(void);

 /**
 * \brief   This API is used to get SEC error status from DMA1
 *
 * \return  status    returns error status.
 */
uint32_t SDL_HWA_DMA1_secErrorStatus(void);

 /**
 *  \brief   This API is used to get DED error status from DMA0
 *
 * \return  status    returns error status.
 */
uint32_t SDL_HWA_DMA0_dedErrorStatus(void);

 /**
 *  \brief   This API is used to get DED error status from DMA1
 *
 * \return  status    returns error status.
 */
uint32_t SDL_HWA_DMA1_dedErrorStatus(void);

 /**
 *  \brief   This API is used to get RED error status from DMA0
 *
 * \return  status    returns error status.
 */
uint32_t SDL_HWA_DMA0_redErrorStatus(void);

 /**
 *  \brief   This API is used to get RED error status from DMA1
 *
 * \return  status    returns error status.
 */
 uint32_t SDL_HWA_DMA1_redErrorStatus(void);

/**
 * \brief   This API is used  for configuring and testing parity of the HWA memory
 *
 * \param   memID     indicates the memory in the HWA
 *
 * \param   memBlock  indicates the memory block in the HWA memory
 *
 * \return  status    return the base address of th instance.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 *                    SDL_EFAIL:    failure, indicate verify failed
 *
 */

int32_t SDL_HWA_memParityExecute( SDL_HWA_MemID memID , SDL_HWA_MemBlock memBlock);

/**
 * \brief   This API is used to induce the error in the fsm lockstep for HWA
 *
 *
 * \return  status    return the base address of th instance.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 *                    SDL_EFAIL:    failure, indicate verify failed
 *
 */
int32_t SDL_HWA_fsmLockStepExecute( void);

#ifdef _cplusplus
}
#endif /*extern "C" */
#endif

/** @} */
