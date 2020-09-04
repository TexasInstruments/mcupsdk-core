/* Copyright (c) 2022 Texas Instruments Incorporated
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
#include <sdl/sdl_hwa.h>

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/* Define the FSM,SEC and DED application interface */
typedef struct sdlHWAApp_s
{
    int32_t  (*application)(void);   /* The code that runs the application */
    char      *name;                  /* The application name */
    int32_t    status;            /* App Status */
} sdlHWAApp_t;

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
#define SDL_APP_NOT_RUN        (-(int32_t) (2))
#define SDL_APP_FAILED         (-(int32_t) (1))
#define SDL_APP_PASS           ( (int32_t) (0))
#define SDL_HWA_ECC_TIMEOUT    (0x10000U)
/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
static int32_t SDL_HWA_DMA0_SEC_test(void);
static int32_t SDL_HWA_DMA0_DED_test(void);
static int32_t SDL_HWA_DMA0_RED_test(void);
static int32_t SDL_HWA_DMA1_SEC_test(void);
static int32_t SDL_HWA_DMA1_DED_test(void);
static int32_t SDL_HWA_DMA1_RED_test(void);
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
    {SDL_HWA_DMA0_SEC_test,               "HWA_DMA0SECTest",              SDL_APP_NOT_RUN },
    {SDL_HWA_DMA0_DED_test,               "HWA_DMA0DEDTest",              SDL_APP_NOT_RUN },
    {SDL_HWA_DMA0_RED_test,               "HWA_DMA0REDTest",              SDL_APP_NOT_RUN },
    {SDL_HWA_DMA1_SEC_test,               "HWA_DMA0SECTest",              SDL_APP_NOT_RUN },
    {SDL_HWA_DMA1_DED_test,               "HWA_DMA0DEDTest",              SDL_APP_NOT_RUN },
    {SDL_HWA_DMA1_RED_test,               "HWA_DMA0REDTest",              SDL_APP_NOT_RUN },
    {NULL,                                "TERMINATING CONDITION",        SDL_APP_NOT_RUN }
};

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/
static int32_t SDL_HWA_DMA0_SEC_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_HWA_ECC_TIMEOUT;
    SDL_HWA_DMA0_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_HWA_DMA0_secErrorStatus()!=0U) && (timeout!=0U))
    {
        timeout--;
    }
    if(SDL_HWA_DMA0_secErrorStatus()==1U)
    {
        SDL_HWA_DMA0_secErrorClear();
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

static int32_t SDL_HWA_DMA1_SEC_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_HWA_ECC_TIMEOUT;
    SDL_HWA_DMA1_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_HWA_DMA1_secErrorStatus()!=0U) && (timeout!=0U))
    {
        timeout--;
    }
    if(SDL_HWA_DMA1_secErrorStatus()==1U)
    {
        SDL_HWA_DMA1_secErrorClear();
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

static int32_t SDL_HWA_DMA0_DED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_HWA_ECC_TIMEOUT;
    SDL_HWA_DMA0_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_HWA_DMA0_dedErrorStatus()!=0U) && (timeout!=0U))
    {
        timeout--;
    }
    if(SDL_HWA_DMA0_dedErrorStatus()==1U)
    {
        SDL_HWA_DMA0_dedErrorClear();
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

static int32_t SDL_HWA_DMA1_DED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_HWA_ECC_TIMEOUT;
    SDL_HWA_DMA1_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_HWA_DMA1_dedErrorStatus()!=0U) && (timeout!=0U))
    {
        timeout--;
    }
    if(SDL_HWA_DMA1_dedErrorStatus()==1U)
    {
        SDL_HWA_DMA1_dedErrorClear();
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

static int32_t SDL_HWA_DMA0_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_HWA_ECC_TIMEOUT;
    ret_val= SDL_HWA_DMA0_redExecute(SDL_HWA_FI_GLOBAL_SAFE, SDL_HWA_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((SDL_HWA_DMA0_redErrorStatus()!=0U) && (timeout!=0U))
        {
            timeout--;
        }
        /* Check for the failure. */
        if(SDL_HWA_DMA0_redErrorStatus()==1U)
        {
            SDL_HWA_DMA0_redErrorClear();
            ret_val = SDL_PASS;

        }
        else
        {
            ret_val = SDL_EFAIL;
        }
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

static int32_t SDL_HWA_DMA1_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_HWA_ECC_TIMEOUT;
    ret_val= SDL_HWA_DMA1_redExecute(SDL_HWA_FI_GLOBAL_SAFE, SDL_HWA_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((SDL_HWA_DMA1_redErrorStatus()!=0U) && (timeout!=0U))
        {
            timeout--;
        }
        /* Check for the failure. */
        if(SDL_HWA_DMA0_redErrorStatus()==1U)
        {
            SDL_HWA_DMA1_redErrorClear();
            ret_val = SDL_PASS;
        }
        else
        {
            ret_val = SDL_EFAIL;
        }
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

static int32_t hwaParityDMA0DMEM0_testExecute(void)
{
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM0));
}
static int32_t hwaParityDMA0DMEM1_testExecute(void)
{
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM1));
}
static int32_t hwaParityDMA0DMEM2_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM2));
}
static int32_t hwaParityDMA0DMEM3_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM3));
}
static int32_t hwaParityDMA0DMEM4_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM4));
}
static int32_t hwaParityDMA0DMEM5_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM5));
}
static int32_t hwaParityDMA0DMEM6_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM6));
}

static int32_t hwaParityDMA0DMEM7_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM7));
}

static int32_t hwaParityDMA0WindowRam_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_WINDOW_RAM_MEM_ID , SDL_HWA_WINDOW_RAM));
}

static int32_t hwaParityDMA1DMEM0_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM0));
}
static int32_t hwaParityDMA1DMEM1_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM1));
}
static int32_t hwaParityDMA1DMEM2_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM2));
}
static int32_t hwaParityDMA1DMEM3_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM3));
}
static int32_t hwaParityDMA1DMEM4_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM4));
}
static int32_t hwaParityDMA1DMEM5_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM5));
}
static int32_t hwaParityDMA1DMEM6_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM6));
}
static int32_t hwaParityDMA1DMEM7_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM7));
}
static int32_t hwaParityDMA1WindowRam_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_WINDOW_RAM_MEM_ID , SDL_HWA_WINDOW_RAM));
}

static void hwa_testExecute(void)
{
    /* Declarations of variables */
    int32_t    result = SDL_APP_PASS;
    int32_t    i;
    DebugP_log("\n HWA TEST START : starting\n");
    for ( i = 0; sdlhwaAppTestList[i].application != NULL; i++)
    {
        result = sdlhwaAppTestList[i].application();
        sdlhwaAppTestList[i].status = result;
    }

    result = SDL_APP_PASS;


    for ( i = 0; sdlhwaAppTestList[i].application != NULL; i++)
    {
        if (sdlhwaAppTestList[i].status != SDL_APP_PASS)
        {
            DebugP_log("Applications Name: %s  FAILED \n", sdlhwaAppTestList[i].name);
            result = SDL_APP_FAILED;
            break;
        }
        else
        {
            DebugP_log("Applications Name: %s  PASSED \n", sdlhwaAppTestList[i].name);
        }
    }

    if (result == SDL_APP_PASS)
    {
        DebugP_log("\n All tests have passed \n");
    }
    else
    {
        DebugP_log("\n Few/all tests Failed \n");
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
    if (result != SDL_PASS)
    {
       DebugP_log("Error: DPL Init Failed\n");
    }

    DebugP_log("\n HWA Application\r\n");

    hwa_testExecute();
}

/* Nothing past this point */


