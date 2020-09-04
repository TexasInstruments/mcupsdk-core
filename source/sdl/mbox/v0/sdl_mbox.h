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
 *  @ingroup SDL_MBOX_MODULE
 *  @defgroup SDL_MBOX_API API's for MBOX
 *  @section MBOX Overview
 *         MBOX Test API's:
 *         1. SDL_MSS_MBOX_secExecute()
 *         2. SDL_DSS_MBOX_secExecute()
 *         3. SDL_RSS_MBOX_secExecute()
 *         4. SDL_MSS_MBOX_dedExecute()
 *         5. SDL_DSS_MBOX_dedExecute()
 *         6. SDL_RSS_MBOX_dedExecute()
 *         7. SDL_MSS_MBOX_redExecute()
 *         8. SDL_DSS_MBOX_redExecute()
 *         9. SDL_RSS_MBOX_redExecute()
 *
 *  @{
 */
/**
 *  \file     sdl_mbox.h
 *
 *  \brief    This file contains the prototypes of the APIs present in the
 *            device abstraction layer file of MBOX.
 *            This also contains some related macros.
 */
#ifndef SDL_MBOX_H_
#define SDL_MBOX_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "sdl_mbox_hw.h"
#include <sdl/mbox/v0/soc/sdl_mbox_soc.h>

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
    SDL_MBOX_FI_MAIN =0,
    SDL_MBOX_FI_SAFE = 1,
    SDL_MBOX_FI_GLOBAL_MAIN =2,
    SDL_MBOX_FI_GLOBAL_SAFE=3,
    SDL_MBOX_FI_INVALID = 4,
}SDL_MBOX_busSftyFiType;

typedef enum
{
    SDL_MBOX_MAIN_CMD_INTERFACE =0,
    SDL_MBOX_MAIN_WRITE_INTERFACE =1,
    SDL_MBOX_MAIN_WRITE_STATUS_INTERFACE=2,
    SDL_MBOX_MAIN_READ_INTERFACE =3,
    SDL_MBOX_FI_TYPE_INVALID =4,
}SDL_MBOX_busSftyFiRedType;

/* ========================================================================== */
/*                         Variable Declarations                             */
/* ========================================================================== */
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * \brief   This API is used  for SEC test on MSS MBOX
 *
 */
void SDL_MSS_MBOX_secExecute(void);

/**
 * \brief   This API is used  for DED test on MSS MBOX
 *
 */
void SDL_MSS_MBOX_dedExecute(void);

/**
 * \brief   This API is used  for RED test on MSS MBOX
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
int32_t SDL_MSS_MBOX_redExecute(SDL_MBOX_busSftyFiType fiType, SDL_MBOX_busSftyFiRedType redType);

/**
 * \brief   This API is used to clear SEC error from MSS MBOX
 *
 */
void SDL_MSS_MBOX_secErrorClear(void);

 /**
 *  \brief   This API is used to clear DED error from MSS MBOX
 *
 *
 */
void SDL_MSS_MBOX_dedErrorClear(void);

 /**
 *  \brief   This API is used to clear RED error from MSS MBOX
 *
 */
void SDL_MSS_MBOX_redErrorClear(void);

/**
 * \brief   This API is used  for SEC test on DSS MBOX
 *
 */
void SDL_DSS_MBOX_secExecute(void);

/**
 * \brief   This API is used  for DED test on DSS MBOX
 *
 */
void SDL_DSS_MBOX_dedExecute(void);

/**
 * \brief   This API is used  for RED test on DSS MBOX
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
int32_t SDL_DSS_MBOX_redExecute(SDL_MBOX_busSftyFiType fiType, SDL_MBOX_busSftyFiRedType redType);

/**
 * \brief   This API is used to clear SEC error from DSS MBOX
 *
 */
void SDL_DSS_MBOX_secErrorClear(void);

 /**
 *  \brief   This API is used to clear DED error from DSS MBOX
 *
 *
 */
void SDL_DSS_MBOX_dedErrorClear(void);

/**
 *  \brief   This API is used to clear RED error from DSS MBOX
 *
 */
void SDL_DSS_MBOX_redErrorClear(void);

/**
 * \brief   This API is used  for SEC test on RSS MBOX
 *
 */
void SDL_RSS_MBOX_secExecute(void);

/**
 * \brief   This API is used  for DED test on RSS MBOX
 *
 */
void SDL_RSS_MBOX_dedExecute(void);

/**
 * \brief   This API is used  for RED test on RSS MBOX
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
int32_t SDL_RSS_MBOX_redExecute(SDL_MBOX_busSftyFiType fiType, SDL_MBOX_busSftyFiRedType redType);

/**
 * \brief   This API is used to clear SEC error from RSS MBOX
 *
 */
void SDL_RSS_MBOX_secErrorClear(void);

 /**
 *  \brief   This API is used to clear DED error from RSS MBOX
 *
 *
 */
void SDL_RSS_MBOX_dedErrorClear(void);

 /**
 *  \brief   This API is used to clear RED error from RSS MBOX
 *
 */
void SDL_RSS_MBOX_redErrorClear(void);

/**
 * \brief   This API is used to get the SEC error status on DSS MBOX
 *
 * \return  status    returns error status.
 *
 */
uint32_t SDL_DSS_MBOX_secErrorStatus(void);

/**
 * \brief   This API is used to get the SEC error status on RSS MBOX
 *
 * \return  status    returns error status.
 *
 */
uint32_t SDL_RSS_MBOX_secErrorStatus(void);

/**
 * \brief   This API is used to get the DED error status on MSS MBOX
 *
 * \return  status    returns error status.
 *
 */
uint32_t SDL_DSS_MBOX_dedErrorStatus(void);

/**
 * \brief   This API is used to get the DED error status on RSS MBOX
 *
 * \return  status    returns error status.
 *
 */
uint32_t SDL_RSS_MBOX_dedErrorStatus(void);

/**
 * \brief   This API is used to get the RED error status on MSS MBOX
 *
 * \return  status    returns error status.
 *
 */
uint32_t SDL_DSS_MBOX_redErrorStatus(void);

/**
 * \brief   This API is used to get the RED error status on RSS MBOX
 *
 * \return  status    returns error status.
 *
 */
uint32_t SDL_RSS_MBOX_redErrorStatus(void);


#ifdef _cplusplus
}
#endif /*extern "C" */
#endif

/** @} */


