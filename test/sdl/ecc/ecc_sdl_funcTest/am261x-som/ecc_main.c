/*
 *   Copyright (c) Texas Instruments Incorporated 2022-2023
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
 *  \file     ecc_main.c
 *
 * \brief This file demonstrates using the Error Correcting Code Module (ECC),
 *         utilizing the ECC and ESM Software Diagnostic Reference (SDL) functions.
 *
 *  \details  ESM Safety Example module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdio.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_ecc.h>
#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>
#include "ecc_func.h"



/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

#define SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS				(0x50D18094u)
#define SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW			(0x50D18098u)
#define SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS 				(0x50D18084u)
#define SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW			(0x50D18088u)

#define SDL_CLEAR_STATUS									(0xffu)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
volatile bool gMsmcMemParityInterrupt = false;
/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* ========================================================================== */
/*                          EXternal Function Definitions                              */
/* ========================================================================== */
extern int32_t ECC_funcTest(void);
extern int32_t ECC_ip_funcTest(void);
extern int32_t ECC_r5_funcTest(void);
extern int32_t ECC_sdl_funcTest(void);
/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */

#ifdef UNITY_INCLUDE_CONFIG_H
/*
 *  ======== Unity set up and tear down ========
 */
void setUp(void)
{
    /* Do nothing */
}

void tearDown(void)
{
    /* Do nothing */
}
#endif


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
    uint32_t rd_data = 0;


    printf("\r\nESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \r\n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    printf("\r\nTake action \r\n");
    if(esmIntrType == 1u){
        printf("\r\nHigh Priority Interrupt Executed\r\n");
    }
    else{
        printf("\r\nLow Priority Interrupt Executed\r\n");
    }
    retVal = SDL_ECC_getESMErrorInfo(esmInst, intSrc, &eccmemtype, &eccIntrSrc);

    if(((eccmemtype == SDL_R5FSS0_CORE0_ECC_AGGR) || (eccmemtype == SDL_R5FSS0_CORE0_ECC_AGGR) ||
       (eccmemtype == SDL_R5FSS0_CORE0_ECC_AGGR) || (eccmemtype== SDL_R5FSS0_CORE0_ECC_AGGR)) && ((intSrc != 0x33) && (intSrc != 0x35)))
    {
        /* Clear DED MSS_CTRL register*/
        SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS, SDL_CLEAR_STATUS);
        rd_data = SDL_REG32_RD(SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS);
        printf("\r\nRead data of DED MSS_CTRL register is 0x%u\r\n",rd_data);
        /* Clear DED RAW MSS_CTRL register*/
        SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW, SDL_CLEAR_STATUS);
        rd_data = SDL_REG32_RD(SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW);
        printf("\r\nRead data of DED RAW MSS_CTRL register is 0x%u\r\n",rd_data);

        /* Clear SEC MSS_CTRL register*/
        SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW, SDL_CLEAR_STATUS);
        rd_data = SDL_REG32_RD(SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW);
        printf("\r\nRead data of SEC MSS_CTRL register is  0x%u\r\n",rd_data);
        /* Clear SEC RAW MSS_CTRL register*/
        SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS, SDL_CLEAR_STATUS);
        rd_data = SDL_REG32_RD(SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS);
        printf("\r\nRead data of SEC RAW MSS_CTRL register is 0x%u\r\n",rd_data);
    }
    else
    {

		if(intSrc == 0x33U)
		{
			eccmemtype = 1U;
			eccIntrSrc = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
		}
		if(intSrc == 0x35U)
		{
			eccmemtype = 1U;
			eccIntrSrc = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;
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
    }

    gMsmcMemParityInterrupt = true;

    return retVal;
}

/* SDL_ECC_applicationCallbackFunction is expected to be defined by the application. It is
 * required by the SDL ECC module. It is called by the SDL ECC module to notify the
 * application of certain ECC errors that are reported as Exception events.
 * Note, however, that it is not executed in this example */
void SDL_ECC_applicationCallbackFunction(SDL_ECC_MemType eccMemType,
                                         uint32_t errorSrc,
                                         uint32_t address,
                                         uint32_t ramId,
                                         uint64_t bitErrorOffset,
                                         uint32_t bitErrorGroup)
{

    DebugP_log("\r\nECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
                "address 0x%x, ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\r\n",
                eccMemType, errorSrc, address, ramId, (uint32_t)(bitErrorOffset >> 32),
                (uint32_t)(bitErrorOffset & 0x00000000FFFFFFFF), bitErrorGroup);
    DebugP_log("\r\nTake action\r\n");

    /* Any additional customer specific actions can be added here */

}

static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("\r\nError: Init Failed\r\n");
    }

    return ret;
}

void ECC_func_app(void)
{
    int32_t    testResult;
	testResult = ECC_ip_funcTest();
	DebugP_log("\r\nECC ip func Test\r\n");
	if (testResult == SDL_PASS)
    {
        DebugP_log("\r\nAll ip tests passed. \r\n");
    }
    else
    {
        DebugP_log("\r\nSome ip tests failed. \r\n");
    }
	testResult = ECC_r5_funcTest();
	DebugP_log("\r\nECC r5 func Test\r\n");
	if (testResult == SDL_PASS)
    {
        DebugP_log("\r\nAll r5 tests passed. \r\n");
    }
    else
    {
        DebugP_log("\r\nSome r5 tests failed. \r\n");
    }
	testResult = ECC_sdl_funcTest();
	DebugP_log("\r\nECC sdl func Test\r\n");
	if (testResult == SDL_PASS)
    {
        DebugP_log("\r\nAll tests have passed. \r\n");
    }
    else
    {
        DebugP_log("\r\nSome tests have failed. \r\n");
    }
}

void ecc_app_runner(void)
{
#ifdef UNITY_INCLUDE_CONFIG_H
    UNITY_BEGIN();
    RUN_TEST(ECC_func_app,0, NULL);
    UNITY_END();
#else
    ECC_func_app();
#endif
}


int32_t test_main(void)
{
	/* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    sdlApp_dplInit();
    ecc_app_runner();

	/* Close drivers to close the UART driver for console */
    Board_driversClose();
    Drivers_close();

    return (0);
}

/* Nothing past this point */
