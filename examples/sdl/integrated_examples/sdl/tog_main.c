/*
 * TOG 
 *
 * Timeout Gasket (TOG) Example Application
 *
 *  Copyright (c) 2024-2025 Texas Instruments Incorporated
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
 */
/**
 *  \file tog_main.c
 *
 *  \brief This file triggers input for the Timeout Gasket (TOG) example
 */

#include <stdio.h>
#include <stdlib.h>
#include <dpl_interface.h>
#include <kernel/dpl/HwiP.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_exception.h>
#include <sdl/r5/v0/sdl_interrupt.h>
#include "sdlexample.h"

/* ========================================================================== */
/*                                Fuctions declarations                       */
/* ========================================================================== */

__attribute((section(".text:TOG_test"))) int32_t tog_minTimeout(uint32_t instanceIndex);
__attribute((section(".text:TOG_test"))) void TOG_injectMCUINFRATimeoutError(uint32_t instanceIndex);
__attribute((section(".text:TOG_test"))) void TOG_injectESMError(uint32_t instanceIndex);
__attribute((section(".text:TOG_test"))) void TOG_eventHandler(uint32_t instanceIndex);

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define TOG_TEST_TIMEOUTVAL             (0x10000U)
#define TOG_MAX_TEST_TIMEOUT_VALUE      (10000000u)
/* ========================================================================== */
/*                                Fuctions definitions                        */
/* ========================================================================== */

volatile bool handlerFlag __attribute__((section(".data:TOG_test"))) = false;

extern volatile uint8_t SDL_TOG_interruptDone;
uint32_t  delay;
/* This is the list of exception handle and the parameters */
const SDL_R5ExptnHandlers TOG_Test_R5ExptnHandlers =
{
    .udefExptnHandler = &SDL_EXCEPTION_undefInstructionExptnHandler,
    .swiExptnHandler = &SDL_EXCEPTION_swIntrExptnHandler,
    .pabtExptnHandler = &SDL_EXCEPTION_prefetchAbortExptnHandler,
    .dabtExptnHandler = &SDL_EXCEPTION_dataAbortExptnHandler,
    .irqExptnHandler = &SDL_EXCEPTION_irqExptnHandler,
    .fiqExptnHandler = &SDL_EXCEPTION_fiqExptnHandler,
    .udefExptnHandlerArgs = ((void *)0u),
    .swiExptnHandlerArgs = ((void *)0u),
    .pabtExptnHandlerArgs = ((void *)0u),
    .dabtExptnHandlerArgs = ((void *)0u),
    .irqExptnHandlerArgs = ((void *)0u),
};

void TOG_Test_undefInstructionExptnCallback(void)
{
    printf("\r\nUndefined Instruction exception\r\n");
}

void TOG_Test_swIntrExptnCallback(void)
{
    printf("\r\nSoftware interrupt exception\r\n");
}

void TOG_Test_prefetchAbortExptnCallback(void)
{
    printf("\r\nPrefetch Abort exception\r\n");
}
void TOG_Test_dataAbortExptnCallback(void)
{
    printf("\r\nData Abort exception\r\n");
    TOG_eventHandler((uint32_t)SDL_TOG_INSTANCE_TIMEOUT0_CFG);
}
void TOG_Test_irqExptnCallback(void)
{
    printf("\r\nIrq exception\r\n");
}

void TOG_Test_fiqExptnCallback(void)
{
    printf("\r\nFiq exception\r\n");
}

void TOG_exceptionInit(void)
{
    SDL_EXCEPTION_CallbackFunctions_t exceptionCallbackFunctions =
            {
             .udefExptnCallback = TOG_Test_undefInstructionExptnCallback,
             .swiExptnCallback = TOG_Test_swIntrExptnCallback,
             .pabtExptnCallback = TOG_Test_prefetchAbortExptnCallback,
             .dabtExptnCallback = TOG_Test_dataAbortExptnCallback,
             .irqExptnCallback = TOG_Test_irqExptnCallback,
             .fiqExptnCallback = TOG_Test_fiqExptnCallback,
            };

    /* Initialize SDL exception handler */
    SDL_EXCEPTION_init(&exceptionCallbackFunctions);
    /* Register SDL exception handler */
    Intc_RegisterExptnHandlers(&TOG_Test_R5ExptnHandlers);

    return;
}

void TOG_eventHandler( uint32_t instanceIndex )
{
    int32_t status = SDL_PASS;
    uint32_t pendInts;
    uint32_t intCount;
    SDL_TOG_errInfo errInfo;
    SDL_TOG_config cfg;
    SDL_TOG_Inst instance;
    SDL_TOG_IntrSrc intSrc;
    intSrc = (SDL_TOG_INTRSRC_UNEXPECTED_RESPONSE |
              SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT);

    instance = instanceIndex;
    cfg.cfgCtrl = SDL_TOG_CFG_TIMEOUT;

    if (intSrc != 0U)
    {
        /* Read error info */
        status = SDL_TOG_getErrInfo(instance, &errInfo);
    }

    if (intSrc & SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT)
    {
        /* Get Transaction timeout interrupt count */
        if (status == SDL_PASS)
        {
            status = SDL_TOG_getIntrCount(instance, SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT, &intCount);
        }

        /* Clear Transaction timeout interrupt events */
        if ((status == SDL_PASS) && (intCount != 0))
        {
            status = SDL_TOG_ackIntr(instance, SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT, intCount);
        }
    }

    if (intSrc & SDL_TOG_INTRSRC_UNEXPECTED_RESPONSE)
    {
        /* Get Unexpected Response interrupt count */
        if (status == SDL_PASS)
        {
            status = SDL_TOG_getIntrCount(instance, SDL_TOG_INTRSRC_UNEXPECTED_RESPONSE, &intCount);
        }

        /* Clear Unexpected response interrupt events */
        if ((status == SDL_PASS) && (intCount != 0))
        {
            status = SDL_TOG_ackIntr(instance, SDL_TOG_INTRSRC_UNEXPECTED_RESPONSE, intCount);
        }
    }

    /* Get Pending interrupt count */
    if (status == SDL_PASS)
    {
        status = SDL_TOG_getIntrPending(instance, &pendInts );
    }

    /* Clear Pending interrupt */
    if (status == SDL_PASS)
    {
        status = SDL_TOG_clrIntrPending(instance, SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT);
    }

    if (status == SDL_PASS)
    {
        status = SDL_TOG_clrIntrPending(instance, SDL_TOG_INTRSRC_UNEXPECTED_RESPONSE);
    }

    if (status == SDL_PASS)
    {
        handlerFlag = true;
        /* Call SDL API to configure back Timeout Gasket */
        cfg.timeoutVal = TOG_TEST_TIMEOUTVAL;
        status = SDL_TOG_init(instance, &cfg);

        /* Stop the Timeout Gasket */
        SDL_TOG_stop( instance );

        /* Reset the Timeout gasket */
        SDL_TOG_reset( instance );
    }
    return;
}

/* DDR Baseaddress 0x60000000 translated to system address 0x080000000 for instance 0 */
/* For AM263px, this is the address of OSPI memory. */
#define END_POINT_ACCESS 0x60000000
/* According to instance, This END_POINT_ACCESS have to be changed */

void TOG_injectESMError(uint32_t instanceIndex)
{
    SDL_TOG_Inst instance;
    SDL_TOG_config cfg;
    instance = instanceIndex;
    cfg.cfgCtrl = SDL_TOG_CFG_TIMEOUT;
#if defined (SOC_AM261X)
    volatile uint32_t i = 0;
#endif

    /* Call SDL API to set smaller timeout to trigger error */
    cfg.timeoutVal = 1u;
    SDL_TOG_init(instance, &cfg);
    /* According to instance, need to access this Address*/
#if defined (SOC_AM263X)
    SDL_REG32_RD(END_POINT_ACCESS);
#else
    /* Read from OSPI memory. */
    (*(volatile uint32_t *)(END_POINT_ACCESS));
#endif

    /* Call SDL API to set configure back to original timeout value */
   cfg.timeoutVal = TOG_TEST_TIMEOUTVAL;
#if defined (SOC_AM261X)
    /* Delay before set original timeout */
   for(i=0;i<0xFFFF;i++);
#endif
   SDL_TOG_init(instance, &cfg);
}

int32_t tog_minTimeout(uint32_t instanceIndex)
{
    SDL_TOG_Inst instance;
    SDL_TOG_config cfg;
    int32_t status = SDL_PASS;
    int32_t result = 0;
    volatile uint32_t timeoutCount = 0;
    instance = instanceIndex;
    cfg.cfgCtrl = SDL_TOG_CFG_TIMEOUT;

    if (result == 0)
    {
        /* Enable interrupts */
        status = SDL_TOG_setIntrEnable(instance, SDL_TOG_INTRSRC_ALL, true);
        if (status != SDL_PASS)
        {
            result = -1;
        }
    }

    /** Step 2: Configure and enable Timeout Gasket */
    if (result == 0)
    {
        /* Call SDL API to configure Timeout Gasket */
        cfg.timeoutVal = TOG_TEST_TIMEOUTVAL;
        status = SDL_TOG_init(instance, &cfg);
        if (status != SDL_PASS)
        {
            result = -1;
        }
    }

    if (result == 0)
    {
        /* Call SDL API to enable Timeout Gasket */
        status = SDL_TOG_start(instance);
        if (status != SDL_PASS)
        {
            result = -1;
        }
    }

    SDL_TOG_registerInterrupt();
    /* Initialise exception handler */
    TOG_exceptionInit();

    /* Step 3: Inject timeout error */
    if (result == 0)
    {
        TOG_injectESMError(instance);
    }

    /**--- Step 3: Wait for TOG Interrupt ---*/
    if (result == 0)
    {
        /* Timeout if exceeds time */
#if defined (SOC_AM263PX) || defined (SOC_AM261X)
        while ((!SDL_TOG_interruptDone)
#else
        while ((!handlerFlag)
#endif
               && (timeoutCount < TOG_MAX_TEST_TIMEOUT_VALUE))
        {
            timeoutCount++;
        }

#if defined (SOC_AM263PX) || defined (SOC_AM261X)
        if (!(SDL_TOG_interruptDone))
#else
        if (!(handlerFlag))
#endif
        {
            SDL_TOG_stop( instance );
            result = -1;
        }
        /* reset Done flag so we can run again */
        handlerFlag = false;
        SDL_TOG_interruptDone = false;
    }

    return (result);
}


