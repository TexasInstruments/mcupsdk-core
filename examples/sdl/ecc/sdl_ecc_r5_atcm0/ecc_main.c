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

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

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
#if defined(SOC_AM263X)
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            uintptr_t *arg)
{


    int32_t retVal = 0;
    uint32_t rd_data = 0;


    printf("\r\nESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \r\n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    printf("\r\nTake action \r\n");
    if(esmIntrType == 1u){
        printf("\r\nHigh Priority Interrupt Executed\r\n");
        /* Clear DED MSS_CTRL register*/
        SDL_REG32_WR(0x50D18094u, 0x01);
        rd_data = SDL_REG32_RD(0x50D18094u);
        printf("\r\nRead data of DED MSS_CTRL register is 0x%u\r\n",rd_data);
        /* Clear DED RAW MSS_CTRL register*/
        SDL_REG32_WR(0x50D18098u, 0x01);
        rd_data = SDL_REG32_RD(0x50D18098u);
        printf("\r\nRead data of DED RAW MSS_CTRL register is 0x%u\r\n",rd_data);
    }
    else{
        printf("\r\nLow Priority Interrupt Executed\r\n");
        /* Clear SEC MSS_CTRL register*/
        SDL_REG32_WR(0x50D18088u, 0x01);
        rd_data = SDL_REG32_RD(0x50D18088u);
        printf("\r\nRead data of SEC MSS_CTRL register is  0x%u\r\n",rd_data);
        /* Clear SEC RAW MSS_CTRL register*/
        SDL_REG32_WR(0x50D18084u, 0x01);
        rd_data = SDL_REG32_RD(0x50D18084u);
        printf("\r\nRead data of SEC RAW MSS_CTRL register is 0x%u\r\n",rd_data);
    }

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
    printf("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    printf("\r\nTake action \r\n");

    retVal = SDL_ECC_getESMErrorInfo(esmInstType, intSrc, &eccmemtype, &eccIntrSrc);

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

void ecc_main(void *args)
{
	int32_t    testResult;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

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
    Board_driversClose();
    Drivers_close();
    while (true)
    {
    }
}

/* Nothing past this point */
