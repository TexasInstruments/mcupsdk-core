/******************************************************************************
 * Copyright (c) 2021-2022 Texas Instruments Incorporated - http://www.ti.com
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
 *****************************************************************************/

/**
 *  \defgroup DRV_DDR_MODULE APIs for DDR
 *  \ingroup DRV_MODULE
 *
 *  This module contains APIs to program and use the DDR module.
 *  See \ref DRIVERS_DDR_PAGE for more details.
 *
 *  @{
 */

/** \file v0/ddr.h
 *
 *   \brief This file contains DDR APIs.
 */

#ifndef DDR_DRIVER_H_
#define DDR_DRIVER_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor DDR_ECCErrorType
 *  \name DDR ECC error type
 *
 *  @{
 */
#define DDR_ECC_1B_ERROR        (0U)
#define DDR_ECC_2B_ERROR        (1U)
#define DDR_ECC_ERR_ALL         (2U)
/** @} */

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 * \brief DDR Inline ECC region
 * The structure specifies the DDR inline ECC region start and End address
 */
typedef struct
{
    uint32_t ddrEccStart0;      /**< DDR inline ECC region-0 start address */
    uint32_t ddrEccEnd0;        /**< DDR inline ECC region-0 end address */

    uint32_t ddrEccStart1;      /**< DDR inline ECC region-1 start address */
    uint32_t ddrEccEnd1;        /**< DDR inline ECC region-1 end address */

    uint32_t ddrEccStart2;      /**< DDR inline ECC region-2 start address */
    uint32_t ddrEccEnd2;        /**< DDR inline ECC region-2 end address */
} DDR_EccRegion;

/**
 * \brief  DDR config structure
 *
 * Below arrays and values are generated using the separate DDR config tool
 *  - `ddrssCtlReg`, `ddrssPhyIndepReg`, `ddrssPhyReg`
 *  - `ddrssCtlRegNum`, `ddrssPhyIndepRegNum`, `ddrssPhyIndepRegNum`
 *  - `ddrssCtlRegCount`, `ddrssPhyIndepRegCount`, `ddrssPhyRegCount`
 */
typedef struct
{
    uint64_t clk1Freq;             /**< DDR Frequency of operation, in Hz */
    uint64_t clk2Freq;             /**< DDR Frequency of operation, in Hz */

    uint32_t *ddrssCtlReg;         /**< Array of register value to set */
    uint32_t *ddrssPhyIndepReg;    /**< Array of register value to set */
    uint32_t *ddrssPhyReg;         /**< Array of register value to set */

    uint16_t *ddrssCtlRegNum;      /**< Array of register numbers into which the values `ddrssCtlReg` will be set  */
    uint16_t *ddrssPhyIndepRegNum; /**< Array of register numbers into which the values `ddrssPhyIndepReg` will be set  */
    uint16_t *ddrssPhyRegNum;      /**< Array of register numbers into which the values `ddrssPhyReg` will be set  */

    uint16_t ddrssCtlRegCount;         /**< Number of elements in array `ddrssCtlReg` */
    uint16_t ddrssPhyIndepRegCount;    /**< Number of elements in array `ddrssCtlReg` */
    uint16_t ddrssPhyRegCount;         /**< Number of elements in array `ddrssCtlReg` */

    uint8_t fshcount;              /**< Frequency Handshake count */

    uint8_t enableEccFlag;         /**< Flag to enable Inline ECC */
    DDR_EccRegion *eccRegion;      /**< Inline ECC region address */

} DDR_Params;

/** \brief Emif ECC Error Information structure
 *
 *  This structure provides information about the ECC error
 *
 */
typedef struct
{
    uintptr_t   singlebitErrorAddress;   /**< One Bit ECC error address */
    uintptr_t   doublebitErrorAddress;   /**< Two Bit ECC error address */
    uint32_t    singlebitErrorCount;     /**< One Bit ECC error count   */
} DDR_ECCErrorInfo;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * \brief Set default values to \ref DDR_Params
 *
 * \param   prms     [out] Pointer to DDR configuration.
 *
 */
void DDR_Params_init(DDR_Params *prms);

/**
 * \brief DDR4 Initialization function
 *
 * 		   Initializes the DDR timing parameters. Sets the DDR timing parameters
 * 		   based in the DDR PLL controller configuration.
 * 		   Any changes to DDR PLL requires change to DDR timing.
 *
 * \param   prms     [in] Pointer to DDR configuration.
 *
 * \return  status 	SystemP_SUCCESS in case of success or appropriate error code
 *
 */
int32_t DDR_init(DDR_Params *prms);

/**
 * \brief Enable/Disable DDR inline ECC
 *
 * \param enableFlag [in] Flag to enable or disable DDR Inline ECC
 *
 */
void DDR_enableInlineECC (uint8_t enableFlag);

/**
 * \brief Clear ECC errors for DDR
 *
 * \param errorType [in] DDR ECC error type (single bit/two bit ECC)
 *
 * \return  status SystemP_SUCCESS in case of success or appropriate error code
 *
 */
int32_t DDR_clearECCError (uint8_t errorType);

/**
 * \brief Get ECC error status
 *
 * \param ECCErrorInfo [out] Get DDR ECC error bit status
 *
 * \return  status SystemP_SUCCESS in case of success or appropriate error code
 *
 */
int32_t DDR_getECCErrorInfo (DDR_ECCErrorInfo *ECCErrorInfo);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* DDR_DRIVER_H_ */

/** @} */

