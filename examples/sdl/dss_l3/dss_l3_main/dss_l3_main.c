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
 *  \file     dss_l3_main.c
 *
 *  \brief    This file contains adcbuf example code.
 *
 *  \details  ADCBUF app
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include <dpl_interface.h>
#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_dss_l3.h>
/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */
/* Define the FSM,SEC and DED application interface */
typedef struct sdlDSSL3App_s
{
    int32_t  (*application)(void);   /* The code that runs the application */
    char      *name;                  /* The application name */
    int32_t    status;            /* App Status */
} sdlDSSL3App_t;

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
#define SDL_APP_NOT_RUN        (-(int32_t) (2))
#define SDL_APP_FAILED         (-(int32_t) (1))
#define SDL_APP_PASS           ( (int32_t) (0))
#define SDL_DSS_L3_ECC_TIMEOUT    (0x10000U)
/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
static void dss_l3_testExecute(void);
static int32_t SDL_DSS_L3_BANKA_SEC_test(void);
static int32_t SDL_DSS_L3_BANKB_SEC_test(void);
static int32_t SDL_DSS_L3_BANKC_SEC_test(void);
static int32_t SDL_DSS_L3_BANKD_SEC_test(void);
static int32_t SDL_DSS_L3_BANKA_DED_test(void);
static int32_t SDL_DSS_L3_BANKB_DED_test(void);
static int32_t SDL_DSS_L3_BANKC_DED_test(void);
static int32_t SDL_DSS_L3_BANKD_DED_test(void);
static int32_t SDL_DSS_L3_BANKA_RED_test(void);
static int32_t SDL_DSS_L3_BANKB_RED_test(void);
static int32_t SDL_DSS_L3_BANKC_RED_test(void);
static int32_t SDL_DSS_L3_BANKD_RED_test(void);
/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
sdlDSSL3App_t  sdldssl3AppTestList[] = {
    {SDL_DSS_L3_BANKA_SEC_test,    "DSS_L3_BANKA_SecTest",       SDL_APP_NOT_RUN },
    {SDL_DSS_L3_BANKA_DED_test,    "DSS_L3_BANKA_DedTest",       SDL_APP_NOT_RUN },
    {SDL_DSS_L3_BANKA_RED_test,    "DSS_L3_BANKA_RedTest",       SDL_APP_NOT_RUN },
    {SDL_DSS_L3_BANKB_SEC_test,    "DSS_L3_BANKB_SecTest",       SDL_APP_NOT_RUN },
    {SDL_DSS_L3_BANKB_DED_test,    "DSS_L3_BANKB_DedTest",       SDL_APP_NOT_RUN },
    {SDL_DSS_L3_BANKB_RED_test,    "DSS_L3_BANKB_RedTest",       SDL_APP_NOT_RUN },
    {SDL_DSS_L3_BANKC_SEC_test,    "DSS_L3_BANKC_SecTest",       SDL_APP_NOT_RUN },
    {SDL_DSS_L3_BANKC_DED_test,    "DSS_L3_BANKC_DedTest",       SDL_APP_NOT_RUN },
    {SDL_DSS_L3_BANKC_RED_test,    "DSS_L3_BANKC_RedTest",       SDL_APP_NOT_RUN },
    {SDL_DSS_L3_BANKD_SEC_test,    "DSS_L3_BANKD_SecTest",       SDL_APP_NOT_RUN },
    {SDL_DSS_L3_BANKD_DED_test,    "DSS_L3_BANKD_DedTest",       SDL_APP_NOT_RUN },
    {SDL_DSS_L3_BANKD_RED_test,    "DSS_L3_BANKD_RedTest",       SDL_APP_NOT_RUN },
    {NULL,                         "TERMINATING CONDITION",      SDL_APP_NOT_RUN }
};

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/
static int32_t SDL_DSS_L3_BANKA_SEC_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    SDL_DSS_L3_BANKA_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_L3_BankA_secErrorStatus()!=1U) && (timeout!=0U));

    if(SDL_DSS_L3_BankA_secErrorStatus()==1U)
    {
        ret_val = SDL_PASS;
        SDL_DSS_L3_BankA_secErrorClear();
    }
    else
    {
        ret_val = SDL_EFAIL;
    }

    return ret_val;
}
static int32_t SDL_DSS_L3_BANKB_SEC_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    SDL_DSS_L3_BANKB_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_L3_BankB_secErrorStatus()!=1U) && (timeout!=0U));

    if(SDL_DSS_L3_BankB_secErrorStatus()==1U)
    {
        ret_val = SDL_PASS;
        SDL_DSS_L3_BankB_secErrorClear();
    }
    else
    {
        ret_val = SDL_EFAIL;
    }

    return ret_val;
}
static int32_t SDL_DSS_L3_BANKC_SEC_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    SDL_DSS_L3_BANKC_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_L3_BankC_secErrorStatus()!=1U) && (timeout!=0U));

    if(SDL_DSS_L3_BankC_secErrorStatus()==1U)
    {
        ret_val = SDL_PASS;
        SDL_DSS_L3_BankC_secErrorClear();
    }
    else
    {
        ret_val = SDL_EFAIL;
    }

    return ret_val;
}
static int32_t SDL_DSS_L3_BANKD_SEC_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    SDL_DSS_L3_BANKD_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_L3_BankD_secErrorStatus()!=1U) && (timeout!=0U));


    if(SDL_DSS_L3_BankD_secErrorStatus()==1U)
    {
        ret_val = SDL_PASS;
        SDL_DSS_L3_BankD_secErrorClear();
    }
    else
    {
        ret_val = SDL_EFAIL;
    }

    return ret_val;
}

static int32_t SDL_DSS_L3_BANKA_DED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    SDL_DSS_L3_BANKA_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_L3_BankA_dedErrorStatus()!=1U) && (timeout!=0U));

    if(SDL_DSS_L3_BankA_dedErrorStatus()==1U)
    {
        ret_val = SDL_PASS;
        SDL_DSS_L3_BankA_dedErrorClear();
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

static int32_t SDL_DSS_L3_BANKB_DED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    SDL_DSS_L3_BANKB_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_L3_BankB_dedErrorStatus()!=1U) && (timeout!=0U));

    if(SDL_DSS_L3_BankB_dedErrorStatus()==1U)
    {
        ret_val = SDL_PASS;
        SDL_DSS_L3_BankB_dedErrorClear();
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

static int32_t SDL_DSS_L3_BANKC_DED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    SDL_DSS_L3_BANKC_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_L3_BankC_dedErrorStatus()!=1U) && (timeout!=0U));

    if(SDL_DSS_L3_BankC_dedErrorStatus()==1U)
    {
        ret_val = SDL_PASS;
        SDL_DSS_L3_BankC_dedErrorClear();
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

static int32_t SDL_DSS_L3_BANKD_DED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    SDL_DSS_L3_BANKD_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_L3_BankD_dedErrorStatus()!=1U) && (timeout!=0U));

    if(SDL_DSS_L3_BankD_dedErrorStatus()==1U)
    {
        ret_val = SDL_PASS;
        SDL_DSS_L3_BankD_dedErrorClear();
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}


static int32_t SDL_DSS_L3_BANKA_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    ret_val= SDL_DSS_L3_BANKA_redExecute(SDL_DSS_L3_FI_GLOBAL_SAFE, SDL_DSS_L3_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
         while((SDL_DSS_L3_BankA_redErrorStatus()!=1U) && (timeout!=0U));
        /* Check for the failure. */
        if(SDL_DSS_L3_BankA_redErrorStatus()==1U)
        {
            ret_val = SDL_PASS;
            SDL_DSS_L3_BankA_redErrorClear();
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

static int32_t SDL_DSS_L3_BANKB_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    ret_val= SDL_DSS_L3_BANKB_redExecute(SDL_DSS_L3_FI_GLOBAL_SAFE, SDL_DSS_L3_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((SDL_DSS_L3_BankB_redErrorStatus()!=1U) && (timeout!=0U));
        /* Check for the failure. */
        if(SDL_DSS_L3_BankB_redErrorStatus()==1U)
        {
            ret_val = SDL_PASS;
            SDL_DSS_L3_BankB_redErrorClear();
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

static int32_t SDL_DSS_L3_BANKC_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    ret_val= SDL_DSS_L3_BANKC_redExecute(SDL_DSS_L3_FI_GLOBAL_SAFE, SDL_DSS_L3_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((SDL_DSS_L3_BankC_redErrorStatus()!=1U) && (timeout!=0U));
        /* Check for the failure. */
        if(SDL_DSS_L3_BankC_redErrorStatus()==1U)
        {
            ret_val = SDL_PASS;
            SDL_DSS_L3_BankC_redErrorClear();
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

static int32_t SDL_DSS_L3_BANKD_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    ret_val= SDL_DSS_L3_BANKD_redExecute(SDL_DSS_L3_FI_GLOBAL_SAFE, SDL_DSS_L3_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((SDL_DSS_L3_BankD_redErrorStatus()!=1U) && (timeout!=0U));
        /* Check for the failure. */
        if(SDL_DSS_L3_BankD_redErrorStatus()==1U)
        {
            ret_val = SDL_PASS;
            SDL_DSS_L3_BankD_redErrorClear();
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

static void dss_l3_testExecute(void)
{
    /* Declarations of variables */
    int32_t    result = SDL_APP_PASS;
    int32_t    i;

    DebugP_log("\n DSS L3 TEST START : starting\n");

    for ( i = 0; sdldssl3AppTestList[i].application != NULL; i++)
    {
        result = sdldssl3AppTestList[i].application();
        sdldssl3AppTestList[i].status = result;
    }

    result = SDL_APP_PASS;


    for ( i = 0; sdldssl3AppTestList[i].application != NULL; i++)
    {
        if (sdldssl3AppTestList[i].status != SDL_APP_PASS)
        {
            DebugP_log("\n Applications Name: %s  FAILED \n", sdldssl3AppTestList[i].name);
            result = SDL_APP_FAILED;
            break;
        }
        else
        {
            DebugP_log("Applications Name: %s  PASSED \n", sdldssl3AppTestList[i].name);
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

void sdl_dss_l3_test_main(void *args)
{
    /* Declarations of variables */
    int32_t    result = SDL_APP_PASS;
    /* Init Dpl */
    result = SDL_TEST_dplInit();
    if (result != SDL_PASS)
    {
       DebugP_log("Error: DPL Init Failed\n");
    }

    DebugP_log("\n DSS L3 Application\r\n");

    dss_l3_testExecute();
}



/* Nothing past this point */


