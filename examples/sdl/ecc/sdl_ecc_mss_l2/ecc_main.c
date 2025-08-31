/*
 *   Copyright (c) Texas Instruments Incorporated 2022-2024
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
 *  \file       ecc_main.c
 *
 * \brief       This file demonstrates using the Error Correcting Code Module (ECC),
 *              utilizing the ECC and ESM Software Diagnostic Reference (SDL) functions.
 *
 *  \details    ESM Safety Example module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdio.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ecc_main.h"
#include <sdl/include/sdl_types.h>
#include <sdl/dpl/sdl_dpl.h>
#include <kernel/dpl/TimerP.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_ecc.h>
#include <dpl_interface.h>

#if defined(SOC_AM273X)
#include <sdl/include/am273x/sdlr_mss_ecc_agga.h>
#include <sdl/include/am273x/sdlr_mss_ecc_aggb.h>
#include <sdl/include/am273x/sdlr_mss_ecc_agg_mss.h>
#endif

#if defined(SOC_AWR294X)
#include <sdl/include/awr294x/sdlr_mss_ecc_agga.h>
#include <sdl/include/awr294x/sdlr_mss_ecc_aggb.h>
#include <sdl/include/awr294x/sdlr_mss_ecc_agg_mss.h>
#endif
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#if defined(SOC_AM273X) || defined(SOC_AWR294X)
#define SDL_MSS_ECC_AGGA_ECC_VECTOR_ADDR                    (0x02F7B808)
#define SDL_MSS_ECC_AGGA_ERROR_STATUS1_ADDR                 (0x02F7B820)

#define SDL_MSS_ECC_AGGA_RAM_IDS_TOTAL_ENTRIES              28U

#define SDL_MSS_ECC_AGGB_ECC_VECTOR_ADDR                    (0x02F7BC08)
#define SDL_MSS_ECC_AGGB_ERROR_STATUS1_ADDR                 (0x02F7BC20)

#define SDL_MSS_ECC_AGGB_RAM_IDS_TOTAL_ENTRIES              28U

#define SDL_MSS_ECC_AGG_MSS_ECC_VECTOR_ADDR                 (0x02F7C008)
#define SDL_MSS_ECC_AGG_MSS_ERROR_STATUS1_ADDR              (0x02F7C020)

#define SDL_MSS_ECC_AGG_RAM_IDS_TOTAL_ENTRIES               8U
#endif
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
volatile bool esmError = false;
/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            uintptr_t *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;


    printf("\r\nESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x\r\n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    printf(" \r\nTake action \r\n");
    if(esmIntrType == 1u){
        printf("\r\nHigh Priority Interrupt Executed\r\n");
    }
    else{
        printf("\r\nLow Priority Interrupt Executed\r\n");
    }
    retVal = SDL_ECC_getESMErrorInfo(esmInst, intSrc, &eccmemtype, &eccIntrSrc);

    /* Errata i2427  - Applicable for MSS-L2 and MBOX memories.*/
    /* Check if the callback API is called because of Single Bit Error. */
#if defined (SOC_AM261X)
    if(intSrc == SDL_ESM_INTR_LEVEL_SOC_ECCAGG_CORR_LEVEL)
#else
    if(intSrc == SDL_ESM0_ECC_AGGREGATOR_SOC_ECCAGG_CORR_LEVEL)
#endif
    {
        /* SEC error for MSS-L2. */
        /* Check if the error address is configured as non-cacheble. */
        /* Take additional required steps as this single bit error may cause SED and not correction because of known errata. */
        /* If the error address is configured as cacheble, SEC feature works corrctly. */
    }

    /* Any additional customer specific actions can be added here */
    retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

    printf("\r\nECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
               "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\r\n",
               eccmemtype, eccIntrSrc, eccErrorInfo.memSubType, (uint32_t)(eccErrorInfo.bitErrorOffset >> 32),
               (uint32_t)(eccErrorInfo.bitErrorOffset & 0x00000000FFFFFFFF), eccErrorInfo.bitErrorGroup);

    if (eccErrorInfo.injectBitErrCnt != 0)
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
    }
    else
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
    }

    retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);

    esmError = true;

    return retVal;
}

#endif

#if defined(SOC_AM273X) || defined(SOC_AWR294X)
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;
    DebugP_log("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    DebugP_log("\r\nTake action \r\n");

    retVal = SDL_ECC_getESMErrorInfo(esmInstType, intSrc, &eccmemtype, &eccIntrSrc);

    /* Any additional customer specific actions can be added here */
    retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

    DebugP_log("\r\nECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
               "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\r\n",
               eccmemtype, eccIntrSrc, eccErrorInfo.memSubType, (uint32_t)(eccErrorInfo.bitErrorOffset >> 32),
               (uint32_t)(eccErrorInfo.bitErrorOffset & 0x00000000FFFFFFFF), eccErrorInfo.bitErrorGroup);

    if (eccErrorInfo.injectBitErrCnt != 0)
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT, eccErrorInfo.injectBitErrCnt);
    }
    else
    {
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
    }

    retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);

    esmError = true;

    return retVal;
}
#endif

void ecc_main(void *args)
{
	int32_t testResult = 0;

    /* Open drivers to open the UART driver for console */
    Drivers_open();

#if defined(SOC_AM273X) || defined(SOC_AWR294X)
    uint8_t i;

    /* Clear all status registers of MSS AGGRA ECC AGGR.*/
    for(i=0;i<=SDL_MSS_ECC_AGGA_RAM_IDS_TOTAL_ENTRIES;i++)
    {
        /* Write the RAM ID in to Vector register*/
        SDL_REG32_FINS(SDL_MSS_ECC_AGGA_ECC_VECTOR_ADDR, MSS_ECC_AGGA_ECC_VECTOR_ECC_VECTOR, i);
        /* Clear pending interrupts.*/
        SDL_REG32_WR(SDL_MSS_ECC_AGGA_ERROR_STATUS1_ADDR, 0xF0F);
        ClockP_usleep(100);
    }

    /* Clear all status registers of MSS AGGRB ECC AGGR.*/
    for(i=0;i<=SDL_MSS_ECC_AGGB_RAM_IDS_TOTAL_ENTRIES;i++)
    {
        /* Write the RAM ID in to Vector register*/
        SDL_REG32_FINS(SDL_MSS_ECC_AGGB_ECC_VECTOR_ADDR, MSS_ECC_AGGB_ECC_VECTOR_ECC_VECTOR, i);
        /* Clear pending interrupts.*/
        SDL_REG32_WR(SDL_MSS_ECC_AGGB_ERROR_STATUS1_ADDR, 0xF0F);
        ClockP_usleep(100);
    }

    /* Clear all status registers of MSS ECC AGGR.*/
    for(i=0;i<=SDL_MSS_ECC_AGG_RAM_IDS_TOTAL_ENTRIES;i++)
    {
        /* Write the RAM ID in to Vector register*/
        SDL_REG32_FINS(SDL_MSS_ECC_AGG_MSS_ECC_VECTOR_ADDR, MSS_ECC_AGG_MSS_ECC_VECTOR_ECC_VECTOR, i);
        /* Clear pending interrupts.*/
        SDL_REG32_WR(SDL_MSS_ECC_AGG_MSS_ERROR_STATUS1_ADDR, 0xF0F);
        ClockP_usleep(100);
    }
#endif

    DebugP_log("\r\nECC Example Application\r\n");
    DebugP_log("\r\nECC UC-1 and UC-2 Test \r\n");
    testResult = ECC_funcTest();

    if (testResult == SDL_PASS)
    {
        DebugP_log("\r\nAll tests have passed. \r\n");
    }
    else
    {
        DebugP_log("\r\nSome tests have failed. \r\n");
    }

    /* Close drivers to close the UART driver for console */

    Drivers_close();
    while (true)
    {
    }
}

/* Nothing past this point */
