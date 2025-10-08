/*
 *   Copyright (c) 2022-23 Texas Instruments Incorporated
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
 *  @defgroup SDL_ECC_BUS_SAFETY_API API's for Ecc Bus Safety on MSS and DSS
 *  @ingroup SDL_ECC_BUS_SAFETY_MODULE
 *  @section ECC_BUS_SAFETY Overview
 *
 *   Provides the APIs for ECC Bus Safety.
 *  @{
 */

/**
 *  \file     sdl_ecc_bus_safety.h
 *
 *  \brief    This file contains the declaration of the APIs , Macros, structures and Enums for
 *            device abstraction layer file of ECC Bus Safety.
 *
 */
/** @} */
#ifndef SDL_ECC_BUS_SAFETY_H_
#define SDL_ECC_BUS_SAFETY_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "sdl_ecc_bus_safety_hw.h"
#include <sdl/ecc_bus_safety/v0/soc/sdl_ecc_bus_safety_soc.h>

#ifdef _cplusplus
extern "C" {
#endif
/**
@defgroup SDL_ECC_BUS_SAFETY_MACROS ECC Bus Safety Macros
@ingroup SDL_ECC_BUS_SAFETY_API
*/

/**
@defgroup SDL_ECC_BUS_SAFETY_ENUM ECC Bus Safety Enumerated Data Types
@ingroup SDL_ECC_BUS_SAFETY_API
*/

/**
@defgroup SDL_ECC_BUS_SAFETY_FUNCTIONS ECC Bus Safety Functions
@ingroup SDL_ECC_BUS_SAFETY_API
*/

/**
@defgroup SDL_ECC_BUS_SAFETY_DATASTRUCT ECC Bus Safety Data Structures
@ingroup SDL_ECC_BUS_SAFETY_API
*/
/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/**
@addtogroup SDL_ECC_BUS_SAFETY_DATASTRUCT
@{
*/
typedef struct
{
    /* Base address of the Core Control Register */
    uint32_t baseAddr;
    /* Node Start address */
    uint32_t nodeStartAddr;
    /* Node End address */
    uint32_t nodeEndAddr;
    /* offset of bus safety Control Register */
    uint32_t busSftyCtrl;
    /* offset of bus safety FI Register */
    uint32_t busSftyFi;
    /* offset of bus safety Error Register */
    uint32_t busSftyErr;
#if defined (SOC_AM261X)
    /* offset of bus safety Err Status Register */
    uint32_t busSftyErrStat;
#endif
    /* offset of bus safety Err Stat CMD Register */
    uint32_t busSftyErrStatCmd;
    /* offset of bus safety Err Stat Write Register */
    uint32_t busSftyErrStatWr;
    /* offset of bus safety Err Stat Read Register */
    uint32_t busSftyErrStatRd;
    /* offset of bus safety Err Stat Write Resp Register */
    uint32_t busSftyErrStatWrResp;
}SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S;


typedef struct
{
    /*enable the bus safety for core(mss/dss) */
    uint32_t coreSftyCtrl;
    /* offset of bus safety Control Register */
    uint32_t busSftyCtrl;
    /* offset of bus safety FI Register */
    uint32_t busSftyFi;
    /* offset of bus safety Error Register */
}SDL_ECC_BUS_SAFETY_staticRegs;

/** @} */

/**
@addtogroup SDL_ECC_BUS_SAFETY_ENUM
@{
*/

typedef enum
{
    SDL_ECC_BUS_SAFETY_FI_MAIN =0,
    SDL_ECC_BUS_SAFETY_FI_SAFE = 1,
    SDL_ECC_BUS_SAFETY_FI_GLOBAL_MAIN =2,
    SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE=3,
    SDL_ECC_BUS_SAFETY_FI_INVALID = 4,
} SDL_ECC_BUS_SAFETY_busSftyFiType;

typedef enum
{
    SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE =0,
    SDL_ECC_BUS_SAFETY_MAIN_WRITE_INTERFACE =1,
    SDL_ECC_BUS_SAFETY_MAIN_WRITE_STATUS_INTERFACE=2,
    SDL_ECC_BUS_SAFETY_MAIN_READ_INTERFACE =3,
    SDL_ECC_BUS_SAFETY_FI_TYPE_INVALID =4,
}SDL_ECC_BUS_SAFETY_busSftyFiRedType;

/** @} */

/* ========================================================================== */
/*                         Variable Declarations                             */
/* ========================================================================== */
/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
@addtogroup SDL_ECC_BUS_SAFETY_FUNCTIONS
@{
*/

/**
 * \brief   This API is used  for SEC test on DSS
 *
 * \param   busSftyNode Node identifier
 *
 * \param   addr        address to which data to be written
 *
 * \param   wr_data     data to be be written on the address
 *
 * \return  status      return the Test status.
 *                      SDL_PASS:     success
 *                      SDL_EBADARGS: failure, indicate the bad input arguments
 */
int32_t SDL_ECC_BUS_SAFETY_DSS_secExecute(uint32_t busSftyNode,uint32_t addr, uint32_t wr_data);

/**
 * \brief   This API is used  for DED test on DSS
 *
 * \param   busSftyNode Node identifier
 *
 * \param   addr        address to which data to be written
 *
 * \param   wr_data     data to be be written on the address
 *
 * \return  status      return the Test status.
 *                      SDL_PASS:     success
 *                      SDL_EBADARGS: failure, indicate the bad input arguments
 */
int32_t SDL_ECC_BUS_SAFETY_DSS_dedExecute(uint32_t busSftyNode, uint32_t addr, uint32_t wr_data);

/**
 * \brief   This API is used  for RED test on DSS
 *
 * \param   busSftyNode Node identifier
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 *
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 */
int32_t SDL_ECC_BUS_SAFETY_DSS_redExecute(uint32_t busSftyNode,\
         SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType );


/**
 * \brief   This API is used to clear SEC error on DSS
 *
 * \param   busSftyNode Node identifier
 *
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 */
int32_t SDL_ECC_BUS_SAFETY_DSS_secErrorClear(uint32_t busSftyNode);


/**
 *  \brief   This API is used to get SEC error Status on DSS
 *
 * \param   busSftyNode Node identifier
 *
 * \param   status     pointer to which status to be updated
 *
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 */
int32_t SDL_ECC_BUS_SAFETY_DSS_getSecErrorStatus(uint32_t busSftyNode , uint32_t *status);

/**
 * \brief   This API is used to clear DED error on DSS
 *
 * \param   busSftyNode Node identifier
 *
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 */

int32_t SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(uint32_t busSftyNode);

/**
 *  \brief   This API is used to get DED error Status on DSS
 *
 * \param   busSftyNode Node identifier
 *
 * \param   status     pointer to which status to be updated
 *
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 */

int32_t SDL_ECC_BUS_SAFETY_DSS_getDedErrorStatus(uint32_t busSftyNode , uint32_t *status);

/**
 * \brief   This API is used to clear RED error on DSS
 *
 * \param   busSftyNode Node identifier
 *
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 *                    SDL_EFAIL:    failure, indicate verify failed
 */
int32_t SDL_ECC_BUS_SAFETY_DSS_redErrorClear(uint32_t busSftyNode);

/**
 *  \brief   This API is used to get RED error Status on DSS
 *
 * \param   busSftyNode Node identifier
 *
 * \param   status     pointer to which status to be updated
 *
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 */
int32_t SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(uint32_t busSftyNode , uint32_t *status);

/**
 *  \brief   This API is used to get DSS static register values
 *
 * \param   busSftyNode Node identifier
 *
 * \param   pStaticRegs pointer to staic register
 *
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 */

int32_t SDL_ECC_BUS_SAFETY_DSS_readStaticRegs(uint32_t busSftyNode ,\
                                               SDL_ECC_BUS_SAFETY_staticRegs *pStaticRegs);

/**
* \brief   This API is used  for SEC test on MSS
*
* \param   busSftyNode Node identifier
*
* \param   addr        address to which data to be written
*
* \param   wr_data     data to be be written on the address
*
* \return  status      return the Test status.
*                      SDL_PASS:     success
*                      SDL_EBADARGS: failure, indicate the bad input arguments
*/
int32_t SDL_ECC_BUS_SAFETY_MSS_secExecute(uint32_t busSftyNode,uint32_t addr, uint32_t wr_data);

/**
 * \brief   This API is used  for DED test on MSS
 *
 * \param   busSftyNode Node identifier
 *
 * \param   addr        address to which data to be written
 *
 * \param   wr_data     data to be be written on the address
 *
 * \return  status      return the Test status.
 *                      SDL_PASS:     success
 *                      SDL_EBADARGS: failure, indicate the bad input arguments
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_dedExecute(uint32_t busSftyNode, uint32_t addr, uint32_t wr_data);

/**
 * \brief   This API is used  for RED test on MSS
 *
 * \param   busSftyNode Node identifier
 *
 * \param   fiType     indicates the Fi type
 *
 * \param   redType   indicates interface type
 *
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_redExecute(uint32_t busSftyNode,\
         SDL_ECC_BUS_SAFETY_busSftyFiType fiType, SDL_ECC_BUS_SAFETY_busSftyFiRedType redType );

/**
 * \brief   This API is used to clear SEC error on MSS
 *
 * \param   busSftyNode Node identifier
 *
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_secErrorClear(uint32_t busSftyNode);


/**
 *  \brief   This API is used to get SEC error Status on MSS
 *
 * \param   busSftyNode Node identifier
 *
 * \param   status     pointer to which status to be updated
 *
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_getSecErrorStatus(uint32_t busSftyNode , uint32_t *status);

/**
 * \brief   This API is used to clear DED error on MSS
 *
 * \param   busSftyNode Node identifier
 *
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 */

int32_t SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(uint32_t busSftyNode);

/**
 *  \brief   This API is used to get DED error Status on MSS
 *
 * \param   busSftyNode Node identifier
 *
 * \param   status     pointer to which status to be updated
 *
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 */

int32_t SDL_ECC_BUS_SAFETY_MSS_getDedErrorStatus(uint32_t busSftyNode , uint32_t *status);

/**
 * \brief   This API is used to clear RED error on MSS
 *
 * \param   busSftyNode Node identifier
 *
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 *                    SDL_EFAIL:    failure, indicate verify failed
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_redErrorClear(uint32_t busSftyNode);

/**
 *  \brief   This API is used to get RED error Status on MSS
 *
 * \param   busSftyNode Node identifier
 *
 * \param   status     pointer to which status to be updated
 *
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 */
int32_t SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(uint32_t busSftyNode , uint32_t *status);

/**
 *  \brief   This API is used to get MSS static register values
 *
 * \param   busSftyNode Node identifier
 *
 * \param   pStaticRegs pointer to staic register
 *
 * \return  status    return the Test status.
 *                    SDL_PASS:     success
 *                    SDL_EBADARGS: failure, indicate the bad input arguments
 */

int32_t SDL_ECC_BUS_SAFETY_MSS_readStaticRegs(uint32_t busSftyNode ,\
                                               SDL_ECC_BUS_SAFETY_staticRegs *pStaticRegs);



 /**
 *  \brief   This API is used to detect the device AWR2944/ Awr2943
 *
 * \return  status    returns status to detect awr2943 or awr2944 soc
 *
  */
uint32_t SDL_ECC_BUS_SAFETY_DSS_AWR2944_AWR2943_Detect(void);

/** @} */


#ifdef _cplusplus
}
#endif /*extern "C" */

#endif
