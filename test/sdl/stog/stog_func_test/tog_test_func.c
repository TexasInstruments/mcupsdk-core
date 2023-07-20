/*
 * TOG Usecases
 *
 * Timeout Gasket (TOG) Example Application
 *
 *  Copyright (c) 2023 Texas Instruments Incorporated
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
 *  \file tog_test_func.c
 *
 *  \brief This file triggers input for the Timeout Gasket (TOG) example
 */

#include "tog_test_main.h"
#include <dpl_interface.h>
#include<kernel/dpl/HwiP.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

__attribute((section(".text:TOG_test"))) int32_t tog_minTimeout(uint32_t instanceIndex);
__attribute((section(".text:TOG_test"))) void TOG_injectMCUINFRATimeoutError(uint32_t instanceIndex);
__attribute((section(".text:TOG_test"))) void TOG_injectESMError(uint32_t instanceIndex);

__attribute((section(".text:ESMApp_Handlers"))) int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                                            SDL_ESM_IntType esmIntrType, uint32_t grpChannel,
                                                            uint32_t index, uint32_t intSrc, void *arg);

__attribute((section(".text:TOG_test"))) void TOG_eventHandler(uint32_t instanceIndex);


#if  defined (SOC_AM64X) || defined (SOC_AM243X)
#if defined (M4F_CORE)
#define HWI_SECTION __attribute__((section(".text.hwi")))
#define HwiP_MAX_INTERRUPTS     (16u+64u)
#define ICSR            (volatile uint32_t *)(0xE000ED04u)

typedef struct HwiP_Ctrl_s {

    HwiP_FxnCallback isr[HwiP_MAX_INTERRUPTS];
    void *isrArgs[HwiP_MAX_INTERRUPTS];

} HwiP_Ctrl;
extern HwiP_Ctrl gHwiCtrl;

void HWI_SECTION HwiP_interrupt_handler()
{
    volatile uint32_t *addr;
    uint32_t activeIntNum;

    addr = ICSR;
    activeIntNum = (*addr & 0xFF);

    if(    (activeIntNum < HwiP_MAX_INTERRUPTS)
        && (activeIntNum >= 15) /* sys tick or external NIVC interrupt */
        && (gHwiCtrl.isr[activeIntNum] != NULL)
        )
    {
        gHwiCtrl.isr[activeIntNum](gHwiCtrl.isrArgs[activeIntNum]);
    }
}

void HWI_SECTION HwiP_nmi_handler()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void HWI_SECTION HwiP_hardFault_handler()
{
    volatile uint32_t loop = 0;

    while(loop)
    {
        /*it have to exit for timeout gasket*/
    }
}

void HWI_SECTION HwiP_memFault_handler()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void HWI_SECTION HwiP_busFault_handler()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void HWI_SECTION HwiP_usageFault_handler()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void HWI_SECTION HwiP_reserved_handler()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void HWI_SECTION HwiP_svc_handler()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void HWI_SECTION HwiP_debugMon_handler()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void HWI_SECTION HwiP_pendSV_handler()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

extern uint32_t __STACK_END;
extern void _c_int00();


uint32_t __attribute__((section(".vectors"), aligned(32))) gHwiP_vectorTable[HwiP_MAX_INTERRUPTS]  = {
    (uint32_t)&__STACK_END,             /* 0 */
    (uint32_t)&_c_int00,                /* 1 */
    (uint32_t)&HwiP_nmi_handler,        /* 2 */
    (uint32_t)&HwiP_hardFault_handler,  /* 3 */
    (uint32_t)&HwiP_memFault_handler,   /* 4 */
    (uint32_t)&HwiP_busFault_handler,   /* 5 */
    (uint32_t)&HwiP_usageFault_handler, /* 6 */
    (uint32_t)&HwiP_reserved_handler,   /* 7 */
    (uint32_t)&HwiP_reserved_handler,   /* 8 */
    (uint32_t)&HwiP_reserved_handler,   /* 9 */
    (uint32_t)&HwiP_reserved_handler,   /* 10 */
    (uint32_t)&HwiP_svc_handler,        /* 11 */
    (uint32_t)&HwiP_debugMon_handler,   /* 12 */
    (uint32_t)&HwiP_reserved_handler,   /* 13 */
    (uint32_t)&HwiP_pendSV_handler,     /* 14 */
    (uint32_t)&HwiP_interrupt_handler,  /* 15 */ /* SysTick */
    /* rest of the handlers are setup in HwiP_init and would be for
     * the 'external' NVIC interrupts
     */
};
#endif
#endif

#define TOG_TEST_TIMEOUTVAL 0x10000U
static uint32_t arg;

volatile bool handlerFlag __attribute__((section(".data:TOG_test"))) = false;

SDL_ESM_config TOG_Test_esmInitConfig_MCU =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config */
    .enableBitmap = {0x08000007u, 0x00000000u, 0x00000000u, 0x00000000u,
                },
     /**< All events enable: except timer and self test  events, and Main ESM output */
    /* Temporarily disabling vim compare error as well*/
    .priorityBitmap = {0x08000007u, 0x00000000u, 0x00000000u, 0x00000000u,
                        },
    /**< All events high priority: except timer, selftest error events, and Main ESM output */
    .errorpinBitmap = {0x08000007u, 0x00000000u, 0x00000000u, 0x00000000u,
                      },
    /**< All events high priority: except timer, selftest error events, and Main ESM output */
};

int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            void *arg)
{
    int32_t retVal = SDL_PASS;

    TOG_eventHandler((uint32_t)SDL_TOG_INSTANCE_TIMEOUT0_CFG);

    DebugP_log("\r\n  ESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \r\n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    DebugP_log("  Take action \r\n");

    /* Any additional customer specific actions can be added here */

    return retVal;
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

    DebugP_log(" \r\n  TOG_eventHandler  \r\n");
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

/* DDR Baseaddress 0x60000000 translated to system address 0x080000000 for instance 1 */
#define END_POINT_ACCESS1 0x60000000
/* Mailbox BaseAddress 0xA9000000 translated to system address 0x29000000 for instance 0 */
#define END_POINT_ACCESS0 0xA9000000

void TOG_injectESMError(uint32_t instanceIndex)
{
    SDL_TOG_Inst instance;
    SDL_TOG_config cfg;
    instance = instanceIndex;
    cfg.cfgCtrl = SDL_TOG_CFG_TIMEOUT;
    int32_t status;

    /* Call SDL API to set smaller timeout to trigger error */
    cfg.timeoutVal = 1u;
    status = SDL_TOG_init(instance, &cfg);
    if (status != SDL_PASS)
    {
        DebugP_log("   Inject SDL_TOG_init TimeoutVal Failed \r\n");
        /* Assert */
    }
	if(instanceIndex==0)
	{
	  SDL_REG32_RD(END_POINT_ACCESS0);
	}
	else{

      SDL_REG32_RD(END_POINT_ACCESS1);
	}

    /* Call SDL API to set configure back to original timeout value */
    cfg.timeoutVal = TOG_TEST_TIMEOUTVAL;
    status = SDL_TOG_init(instance, &cfg);
    if (status != SDL_PASS)
    {
        DebugP_log("   Configure back SDL_TOG_init TimeoutVal Failed \r\n");
        /* Assert */
    }
}


int32_t tog_minTimeout(uint32_t instanceIndex)
{
    SDL_TOG_Inst instance;
    SDL_TOG_config cfg;
    void *ptr = (void *)&arg;
    int32_t status = SDL_PASS;
    int32_t result = 0;
    volatile uint32_t timeoutCount = 0;
    instance = instanceIndex;
    cfg.cfgCtrl = SDL_TOG_CFG_TIMEOUT;

    DebugP_log("\r\n Demo for minimum value timeout for instance %d\r\n",instanceIndex);

    /* Initialize MCU ESM module */
#if defined (M4F_CORE)
#if defined (SOC_AM64X) || defined (SOC_AM243X)
    /* Initialize MCU ESM module */
    status = SDL_ESM_init(SDL_ESM_INST_MCU_ESM0, &TOG_Test_esmInitConfig_MCU, SDL_ESM_applicationCallbackFunction, ptr);
#endif
#endif
    if (status != SDL_PASS) {
        /* print error and quit */
        DebugP_log("TOG_App_init: Error initializing MCU ESM: result = %d\r\n", result);

        result = -1;
    } else {
        DebugP_log("\r\n Init MCU ESM complete \r\n");
    }

    if (result == 0)
    {
        /* Enable interrupts */
        status = SDL_TOG_setIntrEnable(instance, SDL_TOG_INTRSRC_ALL, true);
        if (status != SDL_PASS)
        {
            DebugP_log("   SDL_TOG_setIntrEnable Failed \r\n");
            result = -1;
        } else {
            DebugP_log("\r\nSDL_TOG_setIntrEnable complete \r\n");
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
            DebugP_log("   SDL_TOG_init timeout Failed \r\n");
            result = -1;
        } else {
            DebugP_log("\r\nSDL_TOG_init.timeout complete \r\n");
        }
    }

    if (result == 0)
    {
        /* Call SDL API to enable Timeout Gasket */
        status = SDL_TOG_start(instance);
        if (status != SDL_PASS)
        {
            DebugP_log("   SDL_TOG_start Failed \r\n");
            result = -1;
        } else {
            DebugP_log("\r\nSDL_TOG_start complete \r\n");
        }
    }

    /* Step 3: Inject timeout error */
    if (result == 0)
    {
        TOG_injectESMError(instance);
    }

    /**--- Step 3: Wait for TOG Interrupt ---*/
    if (result == 0)
    {
        /* Timeout if exceeds time */
        while ((!handlerFlag)
               && (timeoutCount < TOG_MAX_TEST_TIMEOUT_VALUE))
        {
            timeoutCount++;
        }

        if (!(handlerFlag))
        {
            SDL_TOG_stop( instance );
            DebugP_log("   TOG test timed out \r\n");
            result = -1;
        } else {
            DebugP_log("\r\nSDL_TOG_stop complete \r\n");
        }
        /* reset Done flag so we can run again */
        handlerFlag = false;
    }

    return (result);
}


