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
#include <kernel/dpl/CpuIdP.h>
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS				(0x50D18094u)
#define SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW			(0x50D18098u)
#define SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS 				(0x50D18084u)
#define SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW			(0x50D18088u)
#define SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK    				(0x50D18080u)
#define SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK  				(0x50D18090u)

#define SDL_R5SS1_CPU0_ECC_UNCORR_ERRAGG_STATUS				(0x50D180D4u)
#define SDL_R5SS1_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW			(0x50D180D8u)
#define SDL_R5SS1_CPU0_ECC_CORR_ERRAGG_STATUS 				(0x50D180C8u)
#define SDL_R5SS1_CPU0_ECC_CORR_ERRAGG_STATUS_RAW			(0x50D180C4u)
#define SDL_R5SS1_CPU0_ECC_CORR_ERRAGG_MASK    				(0x50D180A0u)
#define SDL_R5SS1_CPU0_ECC_UNCORR_ERRAGG_MASK  				(0x50D18090u)

#define SDL_CLEAR_STATUS									(0x6u)
#define SDL_MASK_ATCM_TAG_ECC_CORR_ERRORS                   (0x79u)
#define SDL_MASK_ATCM_TAG_ECC_UNCORR_ERRORS                 (0x19u)
#define SDL_UNMASK_TCM_TAG_ERRORS                           (0x0u)

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
    #if defined (R5F0_INPUTS)
        /* Clear DED MSS_CTRL register*/
        SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS, SDL_CLEAR_STATUS);
        rd_data = SDL_REG32_RD(SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS);
        printf("\r\nRead data of DED MSS_CTRL register is 0x%u\r\n",rd_data);
        /* Clear DED RAW MSS_CTRL register*/
        SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW, SDL_CLEAR_STATUS);
        rd_data = SDL_REG32_RD(SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW);
        printf("\r\nRead data of DED RAW MSS_CTRL register is 0x%u\r\n",rd_data);
    #elif defined (R5F1_INPUTS)
        /* Clear DED MSS_CTRL register*/
        SDL_REG32_WR(SDL_R5SS1_CPU0_ECC_UNCORR_ERRAGG_STATUS, SDL_CLEAR_STATUS);
        rd_data = SDL_REG32_RD(SDL_R5SS1_CPU0_ECC_UNCORR_ERRAGG_STATUS);
        printf("\r\nRead data of DED MSS_CTRL register is 0x%u\r\n",rd_data);
        /* Clear DED RAW MSS_CTRL register*/
        SDL_REG32_WR(SDL_R5SS1_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW, SDL_CLEAR_STATUS);
        rd_data = SDL_REG32_RD(SDL_R5SS1_CPU0_ECC_UNCORR_ERRAGG_STATUS_RAW);
        printf("\r\nRead data of DED RAW MSS_CTRL register is 0x%u\r\n",rd_data);
    #endif
    }
    else{
        printf("\r\nLow Priority Interrupt Executed\r\n");
    #if defined (R5F0_INPUTS)
        /* Clear SEC MSS_CTRL register*/
        SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW, SDL_CLEAR_STATUS);
        rd_data = SDL_REG32_RD(SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS_RAW);
        printf("\r\nRead data of SEC MSS_CTRL register is  0x%u\r\n",rd_data);
        /* Clear SEC RAW MSS_CTRL register*/
        SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS, SDL_CLEAR_STATUS);
        rd_data = SDL_REG32_RD(SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_STATUS);
        printf("\r\nRead data of SEC RAW MSS_CTRL register is 0x%u\r\n",rd_data);
    #elif defined (R5F1_INPUTS)
        /* Clear SEC MSS_CTRL register*/
        SDL_REG32_WR(SDL_R5SS1_CPU0_ECC_CORR_ERRAGG_STATUS_RAW, SDL_CLEAR_STATUS);
        rd_data = SDL_REG32_RD(SDL_R5SS1_CPU0_ECC_CORR_ERRAGG_STATUS_RAW);
        printf("\r\nRead data of SEC MSS_CTRL register is  0x%u\r\n",rd_data);
        /* Clear SEC RAW MSS_CTRL register*/
        SDL_REG32_WR(SDL_R5SS1_CPU0_ECC_CORR_ERRAGG_STATUS, SDL_CLEAR_STATUS);
        rd_data = SDL_REG32_RD(SDL_R5SS1_CPU0_ECC_CORR_ERRAGG_STATUS);
        printf("\r\nRead data of SEC RAW MSS_CTRL register is 0x%u\r\n",rd_data);
    #endif
    }

    esmError = true;

    return retVal;
}

int32_t ecc_main(void)
{
	int32_t testResult = 0;
    CSL_ArmR5CPUInfo cpuInfo;

    /* Open drivers to open the UART driver for console */
    Drivers_open();

    CSL_armR5GetCpuID(&cpuInfo);

    DebugP_log("\r\nECC Example Application\r\n");
    DebugP_log("\r\nECC UC-1 and UC-2 Test \r\n");
    if(cpuInfo.grpId == CSL_ARM_R5_CLUSTER_GROUP_ID_0)
    {
        SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK, SDL_MASK_ATCM_TAG_ECC_CORR_ERRORS);
        SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK, SDL_MASK_ATCM_TAG_ECC_UNCORR_ERRORS);
    }
    else
    {
        SDL_REG32_WR(SDL_R5SS1_CPU0_ECC_CORR_ERRAGG_MASK, SDL_MASK_ATCM_TAG_ECC_CORR_ERRORS);
        SDL_REG32_WR(SDL_R5SS1_CPU0_ECC_UNCORR_ERRAGG_MASK, SDL_MASK_ATCM_TAG_ECC_UNCORR_ERRORS);
    }

    testResult = ECC_funcTest();

    if (testResult == SDL_PASS)
    {
        DebugP_log("\r\nAll tests have passed. \r\n");
    }
    else
    {
        DebugP_log("\r\nSome tests have failed. \r\n");
    }

    if(cpuInfo.grpId == CSL_ARM_R5_CLUSTER_GROUP_ID_0)
    {
        SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_CORR_ERRAGG_MASK, SDL_UNMASK_TCM_TAG_ERRORS);
        SDL_REG32_WR(SDL_R5SS0_CPU0_ECC_UNCORR_ERRAGG_MASK, SDL_UNMASK_TCM_TAG_ERRORS);
    }
    else
    {
        SDL_REG32_WR(SDL_R5SS1_CPU0_ECC_CORR_ERRAGG_MASK, SDL_UNMASK_TCM_TAG_ERRORS);
        SDL_REG32_WR(SDL_R5SS1_CPU0_ECC_UNCORR_ERRAGG_MASK, SDL_UNMASK_TCM_TAG_ERRORS);
    }
    /* Close drivers to close the UART driver for console */
    Drivers_close();
    while (true)
    {
    }
}

/* Nothing past this point */
