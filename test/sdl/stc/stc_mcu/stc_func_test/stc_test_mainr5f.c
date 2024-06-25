/* Copyright (c) 2022-2023 Texas Instruments Incorporated
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
 *  \file     main.c
 *
 *  \brief    This file contains STC example code.
 *
 *  \details  STC app
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/

#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <sdl/sdl_stc.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/* Define the application interface */


/**
 *  \brief STC configuration parameter structure.
 */






/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
void STC_test_main(int32_t coreNumber,int32_t testType);


/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
/**
 *  \brief global variable for holding data buffer.
 */
static const SDL_STC_Inst test_case[]={SDL_STC_INST_MAINR5F0, SDL_STC_INST_MAINR5F1};
static  int32_t test_Result;

/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

void STC_test_main(int32_t coreNumber,int32_t testType)
{
    int32_t sdlResult=SDL_PASS;

    /* Two type of test cases are supported\
        1.SDL_STC_TEST       (STC should pass for this testType);
        2.SDL_STC_NEG_TEST   (STC should deliberately fail for this tesType);
    */

    SDL_STC_Config configVal;
    SDL_STC_Config *pConfig=&configVal;

    /* for R5F core the default configuration */

    pConfig->intervalNum=                              STC_MSS_INTERVAL_NUM;
    pConfig->modeConfig.lpScanMode=                    STC_MSS_LP_SCAN_MODE;
    pConfig->modeConfig.codecSpreadMode=               STC_MSS_CODEC_SPREAD_MODE;
    pConfig->modeConfig.capIdleCycle=                  STC_MSS_CAP_IDLE_CYCLE;
    pConfig->modeConfig.scanEnHighCap_idleCycle=       STC_MSS_SCANEN_HIGH_CAP_IDLE_CYCLE;
    pConfig->maxRunTime=                               STC_MSS_MAX_RUN_TIME;
    pConfig->clkDiv=                                   STC_MSS_CLK_DIV;
    pConfig->romStartAddress=                          STC_ROM_START_ADDRESS;
    pConfig->pRomStartAdd=                             STC_pROM_START_ADDRESS;


    if(testType ==SDL_STC_TEST)
    {
        DebugP_log("STC Test should Pass & Completed for Core%d.\r\n",coreNumber);
    }
    else if (testType == SDL_STC_NEG_TEST)
    {
        DebugP_log("STC Test should Fail & Completed for core%d.\r\n",coreNumber);
    }

    DebugP_log("STC Test Application started for Core%d.\r\n",coreNumber);
    DebugP_log("If STC test is successfull, Core%d will go in to Reset.\r\n",coreNumber);
    sdlResult=   SDL_STC_selfTest(test_case[coreNumber], (SDL_STC_TestType)testType,pConfig);

    if (sdlResult!=SDL_PASS)
    {
        DebugP_log("STC Test Could not completed Successfully.\r\n");
    }
}




void STC_func_test_main(void *args)
{
    Drivers_open();
    Board_driversOpen();
    /* disable IRQ */
    HwiP_disable();
    static int32_t countInst=1,i;

    int32_t testTypePos = SDL_STC_TEST;
    int32_t testTypeNeg = SDL_STC_NEG_TEST;

    for (i=countInst; i>=0 ; i--)
    {
        test_Result=  SDL_STC_getStatus(test_case[i]);

        switch (test_Result)
        {
            case SDL_STC_COMPLETED_SUCCESS:
            {
                DebugP_log("Core%d is Reset.\r\n",i);
                DebugP_log("STC is done Successfully & Passed for R5F%d.\r\n",i);
                STC_test_main(i,testTypeNeg);
                break;
            }
            case SDL_STC_COMPLETED_FAILURE:
            {
                DebugP_log("Core%d is Reset.\r\n",i);
                DebugP_log("STC Test is Completed & failing for R5F%d.\r\n",i);

                break;
            }
            case SDL_STC_NOT_COMPLETED:
            {
                DebugP_log("STC Test is Active but Not yet Completed. \r\n");
                break;
            }
            case SDL_STC_NOT_RUN:
            {
                STC_test_main(i,testTypePos);
                break;
            }
            case  INVALID_RESULT:
            {
                DebugP_log("Something Invaild Input. \r\n");
                break;
            }
            default :
            break;

        }
    }
    DebugP_log("Waiting in loop in STC_Main(). \r\n");

    Board_driversClose();
    Drivers_close();

    while(1);

}


/* Nothing past this point */
