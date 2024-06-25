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
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/sdl_ecc.h>
#include <dpl_interface.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>
#include "ecc_test_main.h"

#if defined(R5F_INPUTS)
#include <sdl/include/awr294x/sdlr_mss_ecc_agga.h>
#include <sdl/include/awr294x/sdlr_mss_ecc_aggb.h>
#include <sdl/include/awr294x/sdlr_mss_ecc_agg_mss.h>
#elif defined(C66_INPUTS)
#include <sdl/include/awr294x/sdlr_dss_ecc_agg.h>
#endif

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define SDL_DSS_ECC_AGG_ECC_VECTOR_ADDR                     (0x060A0008)
#define SDL_DSS_ECC_AGG_ERROR_STATUS1_ADDR                  (0x060A0020)

#define SDL_DSS_ECC_AGG_RAM_IDS_TOTAL_ENTRIES               22

#define SDL_MSS_ECC_AGGA_ECC_VECTOR_ADDR                    (0x02F7B808)
#define SDL_MSS_ECC_AGGA_ERROR_STATUS1_ADDR                 (0x02F7B820)

#define SDL_MSS_ECC_AGGA_RAM_IDS_TOTAL_ENTRIES              28U

#define SDL_MSS_ECC_AGGB_ECC_VECTOR_ADDR                    (0x02F7BC08)
#define SDL_MSS_ECC_AGGB_ERROR_STATUS1_ADDR                 (0x02F7BC20)

#define SDL_MSS_ECC_AGGB_RAM_IDS_TOTAL_ENTRIES              28U

#define SDL_MSS_ECC_AGG_MSS_ECC_VECTOR_ADDR                 (0x02F7C008)
#define SDL_MSS_ECC_AGG_MSS_ERROR_STATUS1_ADDR              (0x02F7C020)

#define SDL_MSS_ECC_AGG_RAM_IDS_TOTAL_ENTRIES               8U
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
/*                          EXternal Function Definitions                              */
/* ========================================================================== */
#if defined(R5F_INPUTS)
extern int32_t ECC_ip_errTest(void);
extern int32_t ECC_r5_errTest(void);
extern int32_t ECC_errTest(void);
#elif defined(C66_INPUTS)
extern int32_t DSS_ECC_ip_errTest(void);
extern int32_t DSS_ECC_errTest(void);
#endif
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
                                            int32_t grpChannel,
                                            int32_t intSrc,
                                            uintptr_t *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;


    printf("\n  ESM Call back function called : instance = 0x%x," \
                "grpChannel = 0x%x, intSrc = 0x%x \n",
                esmInst, grpChannel, intSrc);
    printf("  Take action \n");
    retVal = SDL_ECC_getESMErrorInfo(esmInst, intSrc, &eccmemtype, &eccIntrSrc);
    if (retVal != SDL_PASS)
    {
        printf("SDL_ECC_getESMErrorInfo failed\n");
    }
    /* Any additional customer specific actions can be added here */
    retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);

    if (retVal != SDL_PASS)
    {
        printf("SDL_ECC_getErrorInfo failed\n");
    }
    printf("\n  ECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
               "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\n",
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

static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("Error: Init Failed\n");
    }

    return ret;
}

void test_sdl_ecc_unit_test_app(void)
{
    int32_t testResult = 0;
    uint8_t i;

#if defined(R5F_INPUTS)
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

    testResult = ECC_ip_errTest();
#elif defined(C66_INPUTS)

    /* Clear all status registers.*/
    for(i=0;i<=SDL_DSS_ECC_AGG_RAM_IDS_TOTAL_ENTRIES;i++)
    {
        /* Write the RAM ID in to Vector register*/
        SDL_REG32_FINS(SDL_DSS_ECC_AGG_ECC_VECTOR_ADDR, DSS_ECC_AGG_ECC_VECTOR_ECC_VECTOR, i);
        /* Clear pending interrupts.*/
        SDL_REG32_WR(SDL_DSS_ECC_AGG_ERROR_STATUS1_ADDR, 0xF0F);
        ClockP_usleep(100);
    }
    testResult = DSS_ECC_ip_errTest();
#endif
    DebugP_log("\n ECC Error ip Module Unit Test");
    if (testResult == SDL_PASS)
    {
        DebugP_log(" ip Module unit test Passed.\r\n");
    }
    else
    {
        DebugP_log(" ip Module unit test Failed.\r\n");
    }    
#if defined(R5F_INPUTS)
    testResult = ECC_errTest();
#elif defined(C66_INPUTS)
    testResult = DSS_ECC_errTest();
#endif
    DebugP_log("\n ECC Error SDL Module Unit Test");
    if (testResult == SDL_PASS)
    {
        DebugP_log(" SDL Module unit test Passed.\r\n");
    }
    else
    {
        DebugP_log(" SDL Module unit test Failed.\r\n");
    }
}

void test_sdl_ecc_test_app_runner(void)
{
#ifdef UNITY_INCLUDE_CONFIG_H
    UNITY_BEGIN();
    RUN_TEST(test_sdl_ecc_unit_test_app,0,NULL);
    UNITY_END();
#else
    test_sdl_ecc_unit_test_app();
#endif
}

void test_main(void *args)
{
    sdlApp_dplInit();
    test_sdl_ecc_test_app_runner();
}

/* Nothing past this point */
