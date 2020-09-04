/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 *  \file sdl_mbox.c
 *
 *  \brief Common across test-cases using MBOX.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "mbox_main.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
volatile bool mboxSecFlag = FALSE;
#if defined SUBSYS_DSS
#if defined (SOC_AWR294X)
volatile bool rssMboxDedRedFlag = FALSE;
#endif
volatile bool dssMboxDedRedFlag = FALSE;
#endif
#if defined SUBSYS_MSS
volatile bool mssMboxDedRedFlag = FALSE;
#endif

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */
#if defined SUBSYS_MSS
SDL_ESM_NotifyParams gESM_ParamsDedRed=

{
    1U,
     /* ESM group number */
    114U,
     /* DCC error pin number connected to ESM */
    0U,
    /* Set the interrupt priority level to high or low. Applicable to Group 1 errors only. */
    TRUE,
    /* Enable failure influence on ERROR pin. Applicable to Group 1 errors only. */
    NULL,
    /* Argument passed back when the Notify function is invoked. */
    &SDL_MBOX_ESM_applicationCallbackFunctionforMSSDedRed
    /* Notify function called by the ESM driver. */

};
SDL_ESM_NotifyParams gESM_ParamsSec=

{
    1U,
     /* ESM group number */
    66U,
     /* DCC error pin number connected to ESM */
    0U,
    /* Set the interrupt priority level to high or low. Applicable to Group 1 errors only. */
    TRUE,
    /* Enable failure influence on ERROR pin. Applicable to Group 1 errors only. */
    NULL,
    /* Argument passed back when the Notify function is invoked. */
    &SDL_MBOX_ESM_applicationCallbackFunctionforSec
    /* Notify function called by the ESM driver. */

};
#endif



SDL_ESM_OpenParams esmOpenParams=
{
    TRUE
    /* boolean value to indicate if old ESM pending errors should be cleared or not
          This field will be set by SysCfg.*/
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
#if defined SUBSYS_MSS
int32_t SDL_MBOX_ESM_applicationCallbackFunctionforMSSDedRed(SDL_ESM_Inst esmInst,
                                            int grpChannel,
                                            int intSrc,
                                            void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_MSS_MBOX_dedErrorClear();
    SDL_MSS_MBOX_redErrorClear();
    mssMboxDedRedFlag = TRUE;
    return retVal;
}

int32_t SDL_MBOX_ESM_applicationCallbackFunctionforSec(SDL_ESM_Inst esmInst,
                                            int grpChannel,
                                            int intSrc,
                                            void *arg)
{
    int32_t retVal = SDL_PASS;
    SDL_MSS_MBOX_secErrorClear();
    mboxSecFlag = TRUE;
    return retVal;
}

int32_t SDL_MSS_MBOX_SEC_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_MBOX_ECC_TIMEOUT;
    ret_val = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &gESM_ParamsSec, &esmOpenParams, NULL);
    if (ret_val == SDL_PASS)
    {
        DebugP_log("\nMSS_MBOX_SEC_Test_init: Init ESM complete \n\n");
    }
    else
    {
        /* print error and quit */
        DebugP_log("nMSS_MBOX_SEC_Test_init: Error initializing ESM: result = %d\n", ret_val);

    }
    SDL_MSS_MBOX_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((mboxSecFlag!=TRUE) && (timeout!=0U))
    {
        timeout--;
    }
    if(mboxSecFlag==TRUE)
    {
        DebugP_log("ESM Call back function called and action taken \n\n");
        mboxSecFlag=FALSE;
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

int32_t SDL_MSS_MBOX_DED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_MBOX_ECC_TIMEOUT;
    ret_val = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &gESM_ParamsDedRed, &esmOpenParams, NULL);
    if (ret_val == SDL_PASS)
    {
        DebugP_log("\nMSS_MBOX_DED_Test_init: Init ESM complete \n\n");
    }
    else
    {
        /* print error and quit */
        DebugP_log("nMSS_MBOX_DED_Test_init: Error initializing ESM: result = %d\n", ret_val);

    }
    SDL_MSS_MBOX_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((mssMboxDedRedFlag!=TRUE) && (timeout!=0U))
    {
        timeout--;
    }
    if(mssMboxDedRedFlag==TRUE)
    {
        DebugP_log("ESM Call back function called and action taken \n\n");
        mssMboxDedRedFlag=FALSE;
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}


int32_t SDL_MSS_MBOX_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_MBOX_ECC_TIMEOUT;
    ret_val = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &gESM_ParamsDedRed, &esmOpenParams, NULL);
    if (ret_val == SDL_PASS)
    {
        DebugP_log("\nMSS_MBOX_RED_Test_init: Init ESM complete \n\n");
    }
    else
    {
        /* print error and quit */
        DebugP_log("nMSS_MBOX_RED_Test_init: Error initializing ESM: result = %d\n", ret_val);

    }
    ret_val= SDL_MSS_MBOX_redExecute(SDL_MBOX_FI_GLOBAL_SAFE, SDL_MBOX_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((mssMboxDedRedFlag!=TRUE) && (timeout!=0U))
        {
            timeout--;
        }
        /* Check for the failure. */
        if(mssMboxDedRedFlag==TRUE)
        {
            ret_val = SDL_PASS;
            mssMboxDedRedFlag =FALSE;
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
#endif

#if defined SUBSYS_DSS

int32_t SDL_DSS_MBOX_SEC_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_MBOX_ECC_TIMEOUT;
    SDL_DSS_MBOX_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_MBOX_secErrorStatus()!=1U) && (timeout!=0U))
    {
        timeout--;
    }
    if(SDL_DSS_MBOX_secErrorStatus()==1U)
    {
        SDL_DSS_MBOX_secErrorClear();
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}
#if defined (SOC_AWR294X)
int32_t SDL_RSS_MBOX_SEC_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_MBOX_ECC_TIMEOUT;
    SDL_RSS_MBOX_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_RSS_MBOX_secErrorStatus()!=1U) && (timeout!=0U))
    {
        timeout--;
    }
    if(SDL_RSS_MBOX_secErrorStatus()==1U)
    {
        SDL_RSS_MBOX_secErrorClear();
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}
#endif

int32_t SDL_DSS_MBOX_DED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_MBOX_ECC_TIMEOUT;
    SDL_DSS_MBOX_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_MBOX_dedErrorStatus()!=1U) && (timeout!=0U))
    {
        timeout--;
    }
    if(SDL_DSS_MBOX_dedErrorStatus()==1U)
    {
        SDL_DSS_MBOX_dedErrorClear();
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}
#if defined (SOC_AWR294X)
int32_t SDL_RSS_MBOX_DED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_MBOX_ECC_TIMEOUT;
    SDL_RSS_MBOX_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_RSS_MBOX_dedErrorStatus()!=1U) && (timeout!=0U))
    {
        timeout--;
    }
    if(SDL_RSS_MBOX_dedErrorStatus()==1U)
    {
        SDL_RSS_MBOX_dedErrorClear();
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}
#endif
int32_t SDL_DSS_MBOX_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_MBOX_ECC_TIMEOUT;
    ret_val= SDL_DSS_MBOX_redExecute(SDL_MBOX_FI_GLOBAL_SAFE, SDL_MBOX_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((SDL_DSS_MBOX_redErrorStatus()!=1U) && (timeout!=0U))
        {
            timeout--;
        }
        /* Check for the failure. */
        if(SDL_DSS_MBOX_redErrorStatus()==1U)
        {
            ret_val = SDL_PASS;
            SDL_DSS_MBOX_redErrorClear();
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
#if defined (SOC_AWR294X)
int32_t SDL_RSS_MBOX_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_MBOX_ECC_TIMEOUT;
    ret_val= SDL_RSS_MBOX_redExecute(SDL_MBOX_FI_GLOBAL_SAFE, SDL_MBOX_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((SDL_RSS_MBOX_redErrorStatus()!=1U) && (timeout!=0U))
        {
            timeout--;
        }
        /* Check for the failure. */
        if(SDL_RSS_MBOX_redErrorStatus()==1U)
        {
            ret_val = SDL_PASS;
            SDL_RSS_MBOX_redErrorClear();
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
#endif
#endif



