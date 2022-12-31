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
 *  @defgroup SDL_ECC_BUS_SAFETY_API API's for Ecc Bus Safety on MSS, DSS, and RSS
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
/**
@addtogroup SDL_ECC_BUS_SAFETY_MACROS
@{
*/

/* Macro defines Ecc Bus Safety Nodes in the DSS Subsystem */

#define SDL_ECC_BUS_SAFETY_DSS_DSP_MDMA        0U
#define SDL_ECC_BUS_SAFETY_DSS_L3_BANKA        1U
#define SDL_ECC_BUS_SAFETY_DSS_L3_BANKB        2U
#define SDL_ECC_BUS_SAFETY_DSS_L3_BANKC        3U
#define SDL_ECC_BUS_SAFETY_DSS_L3_BANKD        4U
#define SDL_ECC_BUS_SAFETY_DSS_DSP_SDMA        5U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_RD      6U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD      7U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_RD      8U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_RD      9U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_RD      10U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_RD      11U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_RD      12U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_RD      13U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_RD      14U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_RD      15U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR      16U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_WR      17U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_B0_WR      18U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_B1_WR      19U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C0_WR      20U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C1_WR      21U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C2_WR      22U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C3_WR      23U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C4_WR      24U
#define SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR      25U
#define SDL_ECC_BUS_SAFETY_DSS_CBUFF_FIFO      26U
#define SDL_ECC_BUS_SAFETY_DSS_MCRC            27U
#define SDL_ECC_BUS_SAFETY_DSS_PCR             28U
#define SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0        29U
#define SDL_ECC_BUS_SAFETY_DSS_HWA_DMA1        30U
#define SDL_ECC_BUS_SAFETY_DSS_MBOX            31U
#define SDL_ECC_BUS_SAFETY_RSS_TPTCA0_RD       32U
#define SDL_ECC_BUS_SAFETY_RSS_TPTCA0_WR       33U
#define SDL_ECC_BUS_SAFETY_RSS_CSI2A_MDMA      34U
#define SDL_ECC_BUS_SAFETY_RSS_PCR             35U
#define SDL_ECC_BUS_SAFETY_RSS_CQ_MEM_RD       36U
#define SDL_ECC_BUS_SAFETY_RSS_CQ_MEM_WR       37U
#define SDL_ECC_BUS_SAFETY_RSS_STATIC_MEM      38U
#define SDL_ECC_BUS_SAFETY_RSS_BSS_MST         39U
#define SDL_ECC_BUS_SAFETY_RSS_BSS_SLV         40U

/* Node Sufforted only in AM273X */
#define SDL_ECC_BUS_SAFETY_DSS_MDO_FIFO        41U

/* Node Sufforted only in AWR294X */
#define SDL_ECC_BUS_SAFETY_RSS_MBOX            41U
#define SDL_ECC_BUS_SAFETY_RSS_ADCBUF_RD       42U
#define SDL_ECC_BUS_SAFETY_RSS_ADCBUF_WR       43U

#define SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_RD      0U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD      1U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_RD      2U
#define SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_RD        3U
#define SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_RD        4U
#define SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_RD        5U
#define SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_RD        6U
#define SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_S        7U
#define SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_S        8U
#define SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_S        9U
#define SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_S        10U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_A0_WR      11U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_WR      12U
#define SDL_ECC_BUS_SAFETY_MSS_TPTC_B0_WR      13U
#define SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB        14U
#define SDL_ECC_BUS_SAFETY_MSS_CR5B_AHB        15U
#define SDL_ECC_BUS_SAFETY_MSS_CR5C_AHB        16U
#define SDL_ECC_BUS_SAFETY_MSS_CR5D_AHB        17U
#define SDL_ECC_BUS_SAFETY_MSS_CR5A_AXI_WR        18U
#define SDL_ECC_BUS_SAFETY_MSS_CR5B_AXI_WR        19U
#define SDL_ECC_BUS_SAFETY_MSS_CR5C_AXI_WR        20U
#define SDL_ECC_BUS_SAFETY_MSS_CR5D_AXI_WR        21U

#define SDL_ECC_BUS_SAFETY_MSS_MBOX            22U

/** @} */

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
    /* offset of bus safety Err Stat CMD Register */
    uint32_t busSftyErrStatCmd;
    /* offset of bus safety Err Stat Write Register */
    uint32_t busSftyErrStatWr;
    /* offset of bus safety Err Stat Read Register */
    uint32_t busSftyErrStatRd;
    /* offset of bus safety Err Stat Write Resp Register */
    uint32_t busSftyErrStatWrResp;
}SDL_ECC_BUS_SAFETY_Base_Addr_Offset_S;

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

/** @} */


#ifdef _cplusplus
}
#endif /*extern "C" */

#endif
