/* Copyright (c) 2022-23 Texas Instruments Incorporated
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
 *  \file     hwa_main.c
 *
 *  \brief    This file contains hwa example code.
 *
 *  \details  HWA App
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include <dpl_interface.h>
#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <sdl/sdl_hwa.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/* Define the FSM,SEC and DED application interface */
typedef struct sdlHWAApp_s
{
    int32_t  (*application)(void);   /* The code that runs the application */
    char      *name;                  /* The application name */
    int32_t    status;            /* App Status */
    int64_t    test_time;            /* App Test Time */
} sdlHWAApp_t;

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
#define SDL_APP_NOT_RUN                             (-(int32_t) (2))
#define SDL_APP_FAILED                              (-(int32_t) (1))
#define SDL_APP_PASS                                ( (int32_t) (0))

#define SDL_INTR_GROUP_NUM                          (2U)
#define SDL_INTR_PRIORITY_LVL_LOW                   (0U)
#define SDL_INTR_PRIORITY_LVL_HIGH                  (1U)
#define SDL_ENABLE_ERR_PIN                          (1U)


/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
static void hwa_testExecute(void);
static int32_t hwaParityDMA0DMEM0_testExecute(void);
static int32_t hwaParityDMA0DMEM1_testExecute(void);
static int32_t hwaParityDMA0DMEM2_testExecute(void);
static int32_t hwaParityDMA0DMEM3_testExecute(void);
static int32_t hwaParityDMA0DMEM4_testExecute(void);
static int32_t hwaParityDMA0DMEM5_testExecute(void);
static int32_t hwaParityDMA0DMEM6_testExecute(void);
static int32_t hwaParityDMA0DMEM7_testExecute(void);
static int32_t hwaParityDMA0WindowRam_testExecute(void);
static int32_t hwaParityDMA1DMEM0_testExecute(void);
static int32_t hwaParityDMA1DMEM1_testExecute(void);
static int32_t hwaParityDMA1DMEM2_testExecute(void);
static int32_t hwaParityDMA1DMEM3_testExecute(void);
static int32_t hwaParityDMA1DMEM4_testExecute(void);
static int32_t hwaParityDMA1DMEM5_testExecute(void);
static int32_t hwaParityDMA1DMEM6_testExecute(void);
static int32_t hwaParityDMA1DMEM7_testExecute(void);
static int32_t hwaParityDMA1WindowRam_testExecute(void);

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
/* Event BitMap for ECC ESM callback for DSS */
SDL_ESM_NotifyParams hwaTestparamsDSS[2U] =
{
    {
        /* Event BitMap for ECC ESM callback for DSS SEC */
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG2_DSS_HWA_GRP2_ERR ,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_HWA_ESM_CallbackFunction,
    },
    {
        /* Event BitMap for ECC ESM callback for DED and RED for DSS L3 BANKA */
        .groupNumber = SDL_INTR_GROUP_NUM,
        .errorNumber = SDL_DSS_ESMG2_DSS_HWA_GRP2_ERR,
        .setIntrPriorityLvl = SDL_INTR_PRIORITY_LVL_HIGH,
        .enableInfluenceOnErrPin = SDL_ENABLE_ERR_PIN,
        .callBackFunction = &SDL_HWA_ESM_CallbackFunction,
    },
};

sdlHWAApp_t  sdlhwaAppTestList[] = {
    {hwaParityDMA0DMEM0_testExecute,      "HWA_ParityDMA0DMEM0Test",      SDL_APP_NOT_RUN },
    {hwaParityDMA0DMEM1_testExecute,      "HWA_ParityDMA0DMEM1Test",      SDL_APP_NOT_RUN },
    {hwaParityDMA0DMEM2_testExecute,      "HWA_ParityDMA0DMEM2Test",      SDL_APP_NOT_RUN },
    {hwaParityDMA0DMEM3_testExecute,      "HWA_ParityDMA0DMEM3Test",      SDL_APP_NOT_RUN },
    {hwaParityDMA0DMEM4_testExecute,      "HWA_ParityDMA0DMEM4Test",      SDL_APP_NOT_RUN },
    {hwaParityDMA0DMEM5_testExecute,      "HWA_ParityDMA0DMEM5Test",      SDL_APP_NOT_RUN },
    {hwaParityDMA0DMEM6_testExecute,      "HWA_ParityDMA0DMEM6Test",      SDL_APP_NOT_RUN },
    {hwaParityDMA0DMEM7_testExecute,      "HWA_ParityDMA0DMEM7Test",      SDL_APP_NOT_RUN },
    {hwaParityDMA0WindowRam_testExecute,  "HWA_ParityDMA0WindowRamTest",  SDL_APP_NOT_RUN },
    {hwaParityDMA1DMEM0_testExecute,      "HWA_ParityDMA1DMEM0Test",      SDL_APP_NOT_RUN },
    {hwaParityDMA1DMEM1_testExecute,      "HWA_ParityDMA1DMEM1Test",      SDL_APP_NOT_RUN },
    {hwaParityDMA1DMEM2_testExecute,      "HWA_ParityDMA1DMEM2Test",      SDL_APP_NOT_RUN },
    {hwaParityDMA1DMEM3_testExecute,      "HWA_ParityDMA1DMEM3Test",      SDL_APP_NOT_RUN },
    {hwaParityDMA1DMEM4_testExecute,      "HWA_ParityDMA1DMEM4Test",      SDL_APP_NOT_RUN },
    {hwaParityDMA1DMEM5_testExecute,      "HWA_ParityDMA1DMEM5Test",      SDL_APP_NOT_RUN },
    {hwaParityDMA1DMEM6_testExecute,      "HWA_ParityDMA1DMEM6Test",      SDL_APP_NOT_RUN },
    {hwaParityDMA1DMEM7_testExecute,      "HWA_ParityDMA1DMEM7Test",      SDL_APP_NOT_RUN },
    {hwaParityDMA1WindowRam_testExecute,  "HWA_ParityDMA1WindowRamTest",  SDL_APP_NOT_RUN },
    {SDL_HWA_fsmLockStepExecute,          "HWA_FsmLockStepTest",          SDL_APP_NOT_RUN },
    {NULL,                                "TERMINATING CONDITION",        SDL_APP_NOT_RUN }
};

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/
/********************************************************************************************************
* Parity Test for HWA DMA 0 DMEM 0
*********************************************************************************************************/
static int32_t hwaParityDMA0DMEM0_testExecute(void)
{
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM0));
}
/********************************************************************************************************
* Parity Test for HWA DMA 0 DMEM 1
*********************************************************************************************************/
static int32_t hwaParityDMA0DMEM1_testExecute(void)
{
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM1));
}
/********************************************************************************************************
* Parity Test for HWA DMA 0 DMEM 2
*********************************************************************************************************/
static int32_t hwaParityDMA0DMEM2_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM2));
}
/********************************************************************************************************
* Parity Test for HWA DMA 0 DMEM 3
*********************************************************************************************************/
static int32_t hwaParityDMA0DMEM3_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM3));
}
/********************************************************************************************************
* Parity Test for HWA DMA 0 DMEM 4
*********************************************************************************************************/
static int32_t hwaParityDMA0DMEM4_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM4));
}
/********************************************************************************************************
* Parity Test for HWA DMA 0 DMEM 5
*********************************************************************************************************/
static int32_t hwaParityDMA0DMEM5_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM5));
}
/********************************************************************************************************
* Parity Test for HWA DMA 0 DMEM 6
*********************************************************************************************************/
static int32_t hwaParityDMA0DMEM6_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM6));
}
/********************************************************************************************************
* Parity Test for HWA DMA 0 DMEM 7
*********************************************************************************************************/
static int32_t hwaParityDMA0DMEM7_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM7));
}
/********************************************************************************************************
* Parity Test for HWA DMA 0 Window RAM
*********************************************************************************************************/
static int32_t hwaParityDMA0WindowRam_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_WINDOW_RAM_MEM_ID , SDL_HWA_WINDOW_RAM));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 DMEM 0
*********************************************************************************************************/
static int32_t hwaParityDMA1DMEM0_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM0));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 DMEM 1
*********************************************************************************************************/
static int32_t hwaParityDMA1DMEM1_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM1));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 DMEM 2
*********************************************************************************************************/
static int32_t hwaParityDMA1DMEM2_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM2));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 DMEM 3
*********************************************************************************************************/
static int32_t hwaParityDMA1DMEM3_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM3));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 DMEM 4
*********************************************************************************************************/
static int32_t hwaParityDMA1DMEM4_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM4));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 DMEM 5
*********************************************************************************************************/
static int32_t hwaParityDMA1DMEM5_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM5));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 DMEM 6
*********************************************************************************************************/
static int32_t hwaParityDMA1DMEM6_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM6));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 DMEM 7
*********************************************************************************************************/
static int32_t hwaParityDMA1DMEM7_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM7));
}
/********************************************************************************************************
* Parity Test for HWA DMA 1 Window RAM
*********************************************************************************************************/
static int32_t hwaParityDMA1WindowRam_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_WINDOW_RAM_MEM_ID , SDL_HWA_WINDOW_RAM));
}

static void hwa_testExecute(void)
{
    /* Declarations of variables */
    int32_t    result = SDL_APP_PASS;
    int32_t    count;
    uint64_t testStartTime;
    uint64_t testEndTime;
    uint64_t diffTime;
    DebugP_log("\nHWA TEST START : starting\r\n");
    for(count=0U;count<2U;count++)
    {
        result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &hwaTestparamsDSS[count],NULL,NULL);
        if(result == SDL_APP_FAILED )
        {
            break;
        }
        else
        {
            result= SDL_APP_PASS;
        }
    }
    if(result == SDL_APP_PASS )
    {
        DebugP_log("\nESM Initialization for all the SDL HWA Nodes is Done\r\n");
        for ( count = 0; sdlhwaAppTestList[count].application != NULL; count++)
        {
            sdlhwaAppTestList[count].status = result;
            /* Get start time of test */
            testStartTime = ClockP_getTimeUsec();
            result = sdlhwaAppTestList[count].application();
            /* Record test end time */
            testEndTime = ClockP_getTimeUsec();
            diffTime = testEndTime-testStartTime;
            sdlhwaAppTestList[count].status = result;
            sdlhwaAppTestList[count].test_time =diffTime;
        }
        for ( count = 0; sdlhwaAppTestList[count].application != NULL; count++)
        {
            if (sdlhwaAppTestList[count].status != SDL_APP_PASS)
            {
                DebugP_log("\n Applications Name: %s  FAILED and Time taken for the Test is %d  micro secs \r\n", sdlhwaAppTestList[count].name, (uint32_t)sdlhwaAppTestList[count].test_time);
                result = SDL_APP_FAILED;
                break;
            }
            else
            {
                DebugP_log("\nApplications Name: %s  PASSED  and Time taken for the Test is %d  micro secs \r\n", sdlhwaAppTestList[count].name ,(uint32_t)sdlhwaAppTestList[count].test_time );
            }
        }

        if (result == SDL_APP_PASS)
        {
            DebugP_log("\nAll tests have passed \r\n");
        }
        else
        {
            DebugP_log("\nFew/all tests Failed \r\n");
        }
    }
    else
    {
        DebugP_log("\nESM Initialization for Few/all the SDL HWA Nodes is Failed\r\n");
    }
}

/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

void sdl_hwa_test_main(void *args)
{
    /* Declarations of variables */
    int32_t    result = SDL_APP_PASS;
    /* Init Dpl */
    result = SDL_TEST_dplInit();
    Drivers_open();
    if (result != SDL_PASS)
    {
       DebugP_log("\nError: DPL Init Failed\r\n");
    }

    DebugP_log("\nHWA Application\r\n");

    hwa_testExecute();
    Drivers_close();
}

/* Nothing past this point */


