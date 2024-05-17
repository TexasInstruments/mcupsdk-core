/* Copyright (c) 2022-2024 Texas Instruments Incorporated
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
#include <dpl_interface.h>
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

#define COMMON_VARIABLE_MASK      (0x0000FFFF)
#define COMMON_VARIABLE_SHIFT      (0U)
#define SHARED_ADDRESS             (0x102E8000)
#define DSP_STC_BASEADDRESS        (0x6F79200U)

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
void STC_test_main(void );
void STC_DSP_test_main(void);


/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
/**
 *  \brief global variable for holding data buffer.
 */
/* Disable DSP instance for Selftest*/
static const SDL_STC_Inst test_case[]={SDL_STC_INST_MAINR5F0, /*SDL_STC_INST_DSP*/ SDL_STC_INVALID_INSTANCE};
volatile int32_t test_Result;

/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

void STC_DSP_test_main()
{
    int32_t sdlResult=SDL_PASS;
    volatile int32_t wait_Time = 0;
    volatile int32_t Wait_var=0;

    /* Two type of test cases are supported\
        1.SDL_STC_TEST       (STC should pass for this testType);
        2.SDL_STC_NEG_TEST   (STC should deliberately fail for this tesType);
    */

    SDL_STC_Config configVal;
    SDL_STC_Config *pConfig=&configVal;

    /* for R5F core the default configuration */

    pConfig->intervalNum=                              STC_DSS_INTERVAL_NUM;
    pConfig->modeConfig.lpScanMode=                    STC_DSS_LP_SCAN_MODE;
    pConfig->modeConfig.codecSpreadMode=               STC_DSS_CODEC_SPREAD_MODE;
    pConfig->modeConfig.capIdleCycle=                  STC_DSS_CAP_IDLE_CYCLE;
    pConfig->modeConfig.scanEnHighCap_idleCycle=       STC_DSS_SCANEN_HIGH_CAP_IDLE_CYCLE;
    pConfig->maxRunTime=                               STC_DSS_MAX_RUN_TIME;
    pConfig->clkDiv=                                   STC_DSS_CLK_DIV;
    pConfig->romStartAddress=                          STC_ROM_START_ADDRESS;
    pConfig->pRomStartAdd=                             STC_pROM_START_ADDRESS;

    DebugP_log("DSP STC Test Application started.\r\n");
    DebugP_log("If DSP STC test is successfull, DSP Core will be reset.\r\n");

    /* Reading this register for confirmation for intrrupt registration */
    Wait_var= (int32_t) HW_RD_FIELD32(SHARED_ADDRESS, COMMON_VARIABLE);
    while(1)
    {
        DebugP_log("Wait for interrupt registration in DSP Core .\r\n");
        if(Wait_var==0x1111)
        {
            break;
        }
    }
    DebugP_log("Interrupt registration is Done on DSP Core .\r\n");

    sdlResult=   SDL_STC_selfTest(test_case[1], SDL_STC_TEST, pConfig);

    /* Wait for completion of STC test from DSP side */
    do {
        wait_Time++;
        if (wait_Time > 0x0FFFFFF)
        {
            /* Timeout for the wait */
            break;
        }
    } while (1);
    while(1!= (uint32_t)HW_RD_FIELD32(DSP_STC_BASEADDRESS + SDL_STC_STCGSTAT, SDL_STC_TEST_DONE))
    {
        ;
    }

    if (sdlResult!=SDL_PASS)
    {
        DebugP_log("For DSP Core STC Test Could not completed Successfully.\r\n");
    }

    test_Result=  SDL_STC_getStatus(test_case[1]);

    switch (test_Result)
    {
        case SDL_STC_COMPLETED_SUCCESS:
        {
            DebugP_log("DSP Core is Reset.\r\n");
            DebugP_log("DSP STC is done Successfully & Passed.\r\n");
            break;
        }
        case SDL_STC_COMPLETED_FAILURE:
        {
            DebugP_log("DSP Core is Reset.\r\n");
            DebugP_log("DSP STC Test is Completed & failing.\r\n");
            break;
        }
        case SDL_STC_NOT_COMPLETED:
        {
            DebugP_log("DSP STC Test is Active but Not yet Completed. \r\n");
            break;
        }
        case SDL_STC_NOT_RUN:
        {
             DebugP_log("DSP STC Test is not run. \r\n");
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

void STC_test_main()
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

    DebugP_log("R5F STC Test Application started.\r\n");
    DebugP_log("If R5F STC test is successfull,R5F Core will go in to Reset.\r\n");
    sdlResult=   SDL_STC_selfTest(test_case[0], SDL_STC_TEST,pConfig);

    if (sdlResult!=SDL_PASS)
    {
        DebugP_log("For R5F Core STC Test Could not completed Successfully.\r\n");
    }
}

void STC_main(void *args)
{
    Drivers_open();
    Board_driversOpen();
    /* disable IRQ */
    HwiP_disable();

    int32_t core_Instance, core_IndexMax = 1;

    for(core_Instance=core_IndexMax; core_Instance>=0; core_Instance--)
    {
        int32_t curr_Instance= (int32_t)(test_case[core_Instance]);
        if(curr_Instance==(int32_t)SDL_STC_INST_DSP)
        {
            /* DO the required configuration for initialize DSP*/
            SDL_STC_dspInit();

            test_Result=  SDL_STC_getStatus(test_case[core_Instance]);
            if(test_Result==SDL_STC_NOT_RUN)
            {
                /*DSP STC main function*/
                STC_DSP_test_main();
            }
        }
        else
        {
            if(curr_Instance==(int32_t)SDL_STC_INST_MAINR5F0)
            {
                test_Result=  SDL_STC_getStatus(test_case[core_Instance]);

                switch (test_Result)
                {
                    case SDL_STC_COMPLETED_SUCCESS:
                    {
                        DebugP_log("R5F Core is Reset.\r\n");
                        DebugP_log("R5F STC is done Successfully & Passed.\r\n");
                        break;
                    }
                    case SDL_STC_COMPLETED_FAILURE:
                    {
                        DebugP_log("R5F Core is Reset.\r\n");
                        DebugP_log("R5F STC Test is Completed & failing.\r\n");
                        break;
                    }
                    case SDL_STC_NOT_COMPLETED:
                    {
                        DebugP_log("R5F STC Test is Active but Not yet Completed. \r\n");
                        break;
                    }
                    case SDL_STC_NOT_RUN:
                    {
                        STC_test_main();
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
        }
    }

    DebugP_log("Waiting in loop in STC_Main(). \r\n");

    Board_driversClose();
    Drivers_close();

    while(1);
}


/* Nothing past this point */
