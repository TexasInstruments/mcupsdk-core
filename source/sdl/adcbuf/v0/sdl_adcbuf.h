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
 *  @ingroup SDL_ADCBUF_MODULE
 *  @defgroup SDL_ADCBUF_API API's for ADCBUF
 *  @section Overview
 *         ADCBUF Test API's:
 *         1. SDL_ADCBUF_WR_secExecute()
 *         2. SDL_ADCBUF_WR_dedExecute()
 *         3. SDL_ADCBUF_WR_redExecute()
 *         4. SDL_ADCBUF_RD_redExecute()
 *
 *  @{
 */
/**
 *  \file     sdl_adcbuf.h
 *
 *  \brief    This file contains the prototypes of the APIs present in the
 *            device abstraction layer file of ADCBU.
 *            This also contains some related macros.
 */
#ifndef SDL_ADCBUF_H_
#define SDL_ADCBUF_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "sdl_adcbuf_hw.h"
#include <sdl/adcbuf/v0/soc/sdl_adcbuf_soc.h>

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
    SDL_ADCBUF_FI_MAIN =0,
    SDL_ADCBUF_FI_SAFE = 1,
    SDL_ADCBUF_FI_GLOBAL_MAIN =2,
    SDL_ADCBUF_FI_GLOBAL_SAFE=3,
    SDL_ADCBUF_FI_INVALID = 4,
} SDL_ADCBUF_busSftyFiType;

typedef enum
{
    SDL_ADCBUF_MAIN_CMD_INTERFACE =0,
    SDL_ADCBUF_MAIN_WRITE_INTERFACE =1,
    SDL_ADCBUF_MAIN_WRITE_STATUS_INTERFACE=2,
    SDL_ADCBUF_MAIN_READ_INTERFACE =3,
    SDL_ADCBUF_FI_TYPE_INVALID =4,
}SDL_ADCBUF_busSftyFiRedType;

/* ========================================================================== */
/*                         Variable Declarations                             */
/* ========================================================================== */
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * \brief   This API is used  for SEC test on ADCBUF WR
 *
 */
void SDL_ADCBUF_WR_secExecute(void);

/**
 * \brief   This API is used  for DED test on ADCBUF WR
 *
 */

void SDL_ADCBUF_WR_dedExecute(void);


/**
 * \brief   This API is used  for RED test on ADCBUF WR
 *
 * \param   fiType     indicates the Fi type
 * \param   redType   indicates interface type
 * \return  status    return the test status
 *                    SDL_PASS: success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 */
int32_t SDL_ADCBUF_WR_redExecute(SDL_ADCBUF_busSftyFiType fiType, SDL_ADCBUF_busSftyFiRedType redType);
/**
 * \brief   This API is used  for RED test on ADCBUF RD
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 * \return  status    return the test status
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_ADCBUF_RD_redExecute(SDL_ADCBUF_busSftyFiType fiType, SDL_ADCBUF_busSftyFiRedType redType);


 /**
 * \brief   This API is used to clear SEC error from ADCBUF_WR
 *
 *
 */
void SDL_ADCBUF_wrSecErrorClear(void);

 /**
 *  \brief   This API is used to clear DED error from ADCBUF_WR
 *
 *
 */
void SDL_ADCBUF_wrDedErrorClear(void);

 /* \brief   This API is used to clear RED error from ADCBUF_WR
 *
 */
void SDL_ADCBUF_wrRedErrorClear(void);

 /**
 *  \brief   This API is used to clear red error from ADCBUF_RD
 *
 */
void SDL_ADCBUF_rdRedErrorClear(void);

 /**
 *  \brief   This API is used to get red error ststus from ADCBUF_RD
 *
 * \return  status    returns error status.
 */
uint32_t SDL_ADCBUF_rdRedErrorStatus(void);

 /* \brief   This API is used to get RED error status from ADCBUF_WR
 *
 * \return  status    returns error status.
 */
uint32_t SDL_ADCBUF_wrRedErrorStatus(void);

 /**
 *  \brief   This API is used to get DED error status from ADCBUF_WR
 *
 * \return  status    returns error status.
 */
uint32_t SDL_ADCBUF_wrDedErrorStatus(void);

 /**
 * \brief   This API is used to get SEC error status from ADCBUF_WR
 *
 *
 */
uint32_t SDL_ADCBUF_wrSecErrorStatus(void);

#ifdef _cplusplus
}
#endif /*extern "C" */
#endif

/** @} */
