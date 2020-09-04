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
 *  @ingroup SDL_DSS_L3_MODULE
 *  @defgroup SDL_DSS_L3_API DSS_L3 API
 *  @section  DSS_L3 Overview
 *         DSS L3 Test API's:
 *         1. SDL_DSS_L3_BANKA_secExecute()
 *         2. SDL_DSS_L3_BANKB_secExecute()
 *         3. SDL_DSS_L3_BANKC_secExecute()
 *         4. SDL_DSS_L3_BANKD_secExecute()
 *         5. SDL_DSS_L3_BANKA_dedExecute()
 *         6. SDL_DSS_L3_BANKB_dedExecute()
 *         7. SDL_DSS_L3_BANKC_dedExecute()
 *         8. SDL_DSS_L3_BANKD_dedExecute()
 *         9. SDL_DSS_L3_BANKA_redExecute()
 *         10. SDL_DSS_L3_BANKB_redExecute()
 *         11. SDL_DSS_L3_BANKC_redExecute()
 *         12. SDL_DSS_L3_BANKD_redExecute()
 *
 *  @{
 */
/**
 *  \file     sdl_dss_l3.h
 *
 *  \brief    This file contains the prototypes of the APIs present in the
 *            device abstraction layer file of DSS L3.
 *            This also contains some related macros.
 */
#ifndef SDL_DSS_L3_H_
#define SDL_DSS_L3_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "sdl_dss_l3_hw.h"
#include <sdl/dss_l3/v0/soc/sdl_dss_l3_soc.h>

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
    SDL_DSS_L3_FI_MAIN =0,
    SDL_DSS_L3_FI_SAFE = 1,
    SDL_DSS_L3_FI_GLOBAL_MAIN =2,
    SDL_DSS_L3_FI_GLOBAL_SAFE=3,
    SDL_DSS_L3_FI_INVALID = 4,
} SDL_DSS_L3_busSftyFiType;

typedef enum
{
    SDL_DSS_L3_MAIN_CMD_INTERFACE =0,
    SDL_DSS_L3_MAIN_WRITE_INTERFACE =1,
    SDL_DSS_L3_MAIN_WRITE_STATUS_INTERFACE=2,
    SDL_DSS_L3_MAIN_READ_INTERFACE =3,
    SDL_DSS_L3_FI_TYPE_INVALID =4,
}SDL_DSS_L3_busSftyFiRedType;

/* ========================================================================== */
/*                         Variable Declarations                             */
/* ========================================================================== */
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * \brief   This API is used  for SEC test on DSS L3 Bank A
 */
void SDL_DSS_L3_BANKA_secExecute(void);

/**
 * \brief   This API is used  for SEC test on DSS L3 Bank B
 */
void SDL_DSS_L3_BANKB_secExecute(void);

/**
 * \brief   This API is used  for SEC test on DSS L3 Bank C
 */
void SDL_DSS_L3_BANKC_secExecute(void);

/**
 * \brief   This API is used  for SEC test on DSS L3 Bank D
 */
void SDL_DSS_L3_BANKD_secExecute(void);

/**
 * \brief   This API is used  for DED test on DSS L3 Bank A
 */
void SDL_DSS_L3_BANKA_dedExecute(void);

/**
 * \brief   This API is used  for DED test on DSS L3 Bank B
 */

void SDL_DSS_L3_BANKB_dedExecute(void);

/**
 * \brief   This API is used  for DED test on DSS L3 Bank C
 */

void SDL_DSS_L3_BANKC_dedExecute(void);

/**
 * \brief   This API is used  for DED test on DSS L3 Bank D
 */

void SDL_DSS_L3_BANKD_dedExecute(void);

/**
 * \brief   This API is used  for RED test on DSS L3 Bank A
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 * \return  status    return the test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_DSS_L3_BANKA_redExecute(SDL_DSS_L3_busSftyFiType fiType, SDL_DSS_L3_busSftyFiRedType redType);

/**
 * \brief   This API is used  for RED test on DSS L3 Bank B
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 * \return  status    return the test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_DSS_L3_BANKB_redExecute(SDL_DSS_L3_busSftyFiType fiType, SDL_DSS_L3_busSftyFiRedType redType);

/**
 * \brief   This API is used  for RED test on DSS L3 Bank C
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 * \return  status    return the test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_DSS_L3_BANKC_redExecute(SDL_DSS_L3_busSftyFiType fiType, SDL_DSS_L3_busSftyFiRedType redType);

/**
 * \brief   This API is used  for RED test on DSS L3 Bank D
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 * \return  status    return the test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 *
 */
int32_t SDL_DSS_L3_BANKD_redExecute(SDL_DSS_L3_busSftyFiType fiType, SDL_DSS_L3_busSftyFiRedType redType);

/**
 * \brief   This API is used to clear SEC error from DSS_L3 Bank A
 *
 */
void SDL_DSS_L3_BankA_secErrorClear(void);

 /* \brief   This API is used to clear SEC error from DSS L3 Bank B
 *
 */
void SDL_DSS_L3_BankB_secErrorClear(void);

 /**
 *  \brief   This API is used to clear SEC error from DSS L3 Bank C
 *
 */

void SDL_DSS_L3_BankC_secErrorClear(void);

 /**
 *\brief   This API is used to clear SEC error from DSS L3 Bank D
 *
 */

void SDL_DSS_L3_BankD_secErrorClear(void);


 /**
 *  \brief   This API is used to clear DED error from Bank A
 *
 */
void SDL_DSS_L3_BankA_dedErrorClear(void);

/**
 *  \brief   This API is used to clear DED error from Bank B
 *
 */
void SDL_DSS_L3_BankB_dedErrorClear(void);

/**
 *  \brief   This API is used to clear DED error from Bank C
 *
 */
void SDL_DSS_L3_BankC_dedErrorClear(void);

/**
 *  \brief   This API is used to clear DED error from Bank D
 *
 */
void SDL_DSS_L3_BankD_dedErrorClear(void);


 /**
 *  \brief   This API is used to clear RED error from Bank A
 *
 */
void SDL_DSS_L3_BankA_redErrorClear(void);

 /**
 *  \brief   This API is used to clear RED error from Bank B
 *
 */
void SDL_DSS_L3_BankB_redErrorClear(void);

 /**
 *  \brief   This API is used to clear RED error from Bank C
 *
 */
void SDL_DSS_L3_BankC_redErrorClear(void);

 /**
 *  \brief   This API is used to clear RED error from Bank D
 *
 */
void SDL_DSS_L3_BankD_redErrorClear(void);

 /**
 *  \brief   This API is used to get SEC error status from Bank A
 *
 * \return  status    returns error status.
 */
uint32_t SDL_DSS_L3_BankA_secErrorStatus(void);

 /**
 *  \brief   This API is used to get SEC error status from Bank B
 *
 * \return  status    returns error status.
 */
uint32_t SDL_DSS_L3_BankB_secErrorStatus(void);

 /**
 *  \brief   This API is used to get SEC error status from Bank C
 *
 * \return  status    returns error status.
 */
uint32_t SDL_DSS_L3_BankC_secErrorStatus(void);

 /**
 *  \brief   This API is used to get SEC error status from Bank D
 *
 * \return  status    returns error status.
 */
uint32_t SDL_DSS_L3_BankD_secErrorStatus(void);


 /**
 *  \brief   This API is used to get DED error status from Bank A
 *
 * \return  status    returns error status.
 */
uint32_t SDL_DSS_L3_BankA_dedErrorStatus(void);

 /**
 *  \brief   This API is used to get DED error status from Bank B
 *
 * \return  status    returns error status.
 */
uint32_t SDL_DSS_L3_BankB_dedErrorStatus(void);

 /**
 *  \brief   This API is used to get DED error status from Bank C
 *
 * \return  status    returns error status.
 */
uint32_t SDL_DSS_L3_BankC_dedErrorStatus(void);

 /**
 *  \brief   This API is used to get DED error status from Bank D
 *
 * \return  status    returns error status.
 */
uint32_t SDL_DSS_L3_BankD_dedErrorStatus(void);

 /**
 *  \brief   This API is used to get RED error status from Bank A
 *
 * \return  status    returns error status.
 */
uint32_t SDL_DSS_L3_BankA_redErrorStatus(void);

 /**
 *  \brief   This API is used to get RED error status from Bank B
 *
 * \return  status    returns error status.
 */
uint32_t SDL_DSS_L3_BankB_redErrorStatus(void);

 /**
 *  \brief   This API is used to get RED error status from Bank C
 *
 * \return  status    returns error status.
 */
uint32_t SDL_DSS_L3_BankC_redErrorStatus(void);

 /**
 *  \brief   This API is used to get RED error status from Bank D
 *
 * \return  status    returns error status.
 */
uint32_t SDL_DSS_L3_BankD_redErrorStatus(void);

#ifdef _cplusplus
}
#endif /*extern "C" */

#endif

/** @} */
