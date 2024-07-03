/*
 *   Copyright (c) Texas Instruments Incorporated 2024
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
 *  \file     parity_trigger.c
 *
 *  \brief    This file contains functions that provide input event triggers
 *            for the TMU Parity application.
 *
 *  \details  TMU Parity Safety Example module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_ecc.h>
#include <kernel/dpl/DebugP.h>
#include "parity_main.h"
#include <sdl/dpl/sdl_dpl.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define R0 0
#define RES_REG R0
#define SIN_OP 0
#define INSTR SIN_OP
#define TMU_SINPUF32_R0 0x40 /* SINPUF32_R0 */
#define TMU_U_BASE 0x00060000

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/*                         0.25,      1.75,      -0.75,     -0.25,     1,         0.75,      -1.75,     1.75 */
uint32_t OP1 []     = {0x3E800000,0x3FE00000,0xBF400000,0xBE800000,0x3F800000,0x3F400000,0xBFE00000,0x3FE00000};
static uint32_t arg;
SDL_ESM_config Test_esmInitConfig_MAIN =
{
    .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0x00000000u, 0x01000000u, 0x00000000u,
                    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
    .priorityBitmap = {0x00000000u, 0x000000000u, 0x01000000u, 0x00000000u,
                    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u },
    .errorpinBitmap = {0x00000000u, 0x00000000u, 0x01000000u, 0x00000000u,
                    0x00000000u, 0x00000000u, 0x00000000u, 0x00000000u},
};

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Parity_Example_init function */
int32_t Parity_Example_init (void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*operand num 0-7 , instruction[1:SINPUF32; 2:COSPUF32, 3:ATANPUF32, 4:SQRTF32, 5:IEXP2F32, 6:LOG2F32, 7:QUADF32, 8:DIVF32], input_value */
void Write_operation1(uint32_t op_num, uint32_t op_type, uint32_t value)
{
  /* Write TMU__op_type_op_num register (Example - TMU_SINPUF32_R0) */
  SDL_REG32_WR((TMU_U_BASE + TMU_SINPUF32_R0 + op_num*8 + op_type*8*8), value);
}

/*********************************************************************
* @fn      Parity_Example_init
*
* @param   None
*
* @return  0 : Success; < 0 for failures
**********************************************************************/
int32_t Parity_Example_init (void)
{
    int32_t retValue=0;
    SDL_ErrType_t result;
    void *ptr = (void *)&arg;

	if (retValue == 0)
    {
        /* Initialize ESM module */
        result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);
        if (result != SDL_PASS)
        {
            /* print error and quit */
            DebugP_log("\r\nESM_Test_init: Error initializing MSS ESM: result = %d\r\n", result);
            retValue = -1;
        }
        else
        {
            DebugP_log("\r\nESM_Test_init: Init MSS ESM complete \r\n");
        }
    }
    return retValue;
}

/*********************************************************************
 * @fn      Parity_sdlFuncTest
 *
 * @brief   Execute ECC sdl function test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/
static int32_t Parity_sdlFuncTest(void)
{
    uint32_t maxTime = 1000000000;
    uint32_t timeOutCnt = 0;
    int32_t retVal = 0;

	DebugP_log("\r\nTMU PARITY Example : Started\r\n");
	DebugP_log("\r\nTMU PARITY : R5FSS0_0\r\n");

	/* R5FSS0_0 */
    {
		DebugP_log("\r\nTMU PARITY test started\r\n");
        SDL_ECC_enableTMUROMParity();
        SDL_ECC_enableTMUROMParityForceError();
        /* Write TMU to trigger parity error */
        Write_operation1(RES_REG+1,INSTR,OP1[0]);

        /* Wait until ESM interrupt happens */
        do
        {
            timeOutCnt += 10;
            if (timeOutCnt > maxTime)
            {
                retVal = SDL_EFAIL;
                DebugP_log("\r\nESM interrupt has not occurred.... TMU PARITY test failed!!\r\n");
                break;
            }
        } while (esmError == false);

        if(retVal != SDL_EFAIL)
        {
            DebugP_log("\r\nTMU Parity Test Passed \r\n");
        }
    }

    return retVal;
}

/*********************************************************************
 * @fn      sdlApp_dplInit
 *
 * @brief   Initialization of DPL
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/
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

/* Parity Function module test */
int32_t Parity_funcTest(void)
{
    int32_t testResult = 0;

    /*Initializing the DPL*/
    sdlApp_dplInit();

    /*Initializing required modules*/
    testResult = Parity_Example_init();

    if (testResult != SDL_PASS)
    {
        DebugP_log("\r\nParity Safety Example tests: unsuccessful\r\n");
        return SDL_EFAIL;
    }

    /*Execute ECC sdl function test*/
    testResult = Parity_sdlFuncTest();

    return (testResult);
}

/* Nothing past this point */
