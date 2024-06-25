/**
 * @file  dpl_interface.c
 *
 * @brief
 *  Example implementation of SDL DPL API.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2021, Texas Instruments, Inc.
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

#include <stdio.h>
#include <sdl/dpl/sdl_dpl.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/AddrTranslateP.h>

#define NUM_HWI_OBJS  10
HwiP_Object gTestHwiObjects[NUM_HWI_OBJS];
uint32_t gTestHwiAvail[NUM_HWI_OBJS];

int32_t PBIST_TEST_findAvailObj()
{
    int i = 0;

    for (i = 0; i < NUM_HWI_OBJS; i++)
    {
        if (gTestHwiAvail[i] == true)
        {
            break;
        }
    }
    if (i == NUM_HWI_OBJS)
    {
        i = -1;
    }
    return i;
}

int32_t PBIST_TEST_findObj(HwiP_Object *obj)
{
    int i = 0;

    for (i = 0; i < NUM_HWI_OBJS; i++)
    {
        if (&gTestHwiObjects[i] == obj)
        {
            break;
        }
    }
    if (i == NUM_HWI_OBJS)
    {
        i = -1;
    }
    return i;
}

pSDL_DPL_HwipHandle PBIST_TEST_registerInterrupt(SDL_DPL_HwipParams *pParams)
{
    HwiP_Params hwipParams;
    HwiP_Params_init(&hwipParams);
    int32_t objNum;

    hwipParams.args = (void *)pParams->callbackArg;
    #ifndef SDL_SOC_MCU_R5F
    /*
     * For M4F, external interrupt #10 at NVIC is
     * 16 internal interrupts + external interrupt number at NVIC
     */
    hwipParams.intNum = pParams->intNum + 16;
    #else
    hwipParams.intNum = pParams->intNum;
    #endif
    hwipParams.callback = pParams->callback;

    objNum = PBIST_TEST_findAvailObj();

    if (objNum != -1)
    {
        HwiP_construct(&gTestHwiObjects[objNum], &hwipParams);
    }

    return &gTestHwiObjects[objNum];
}

pSDL_DPL_HwipHandle PBIST_TEST_registerInterrupt_error(SDL_DPL_HwipParams *pParams)
{
    return NULL;
}

int32_t PBIST_TEST_deregisterInterrupt(pSDL_DPL_HwipHandle handle)
{
    int32_t objNum;
    objNum = PBIST_TEST_findObj(handle);
    if (objNum != -1)
    {
        gTestHwiAvail[objNum] = true;
    }
    HwiP_destruct(handle);
    return SDL_PASS;
}

void PBIST_TEST_eventHandler( uint32_t instanceId)
{
    return;
}

pSDL_DPL_HwipHandle PBIST_TEST_registerInterrupt_timeout(SDL_DPL_HwipParams *pParams)
{
    HwiP_Params hwipParams;
    HwiP_Params_init(&hwipParams);
    int32_t objNum;

    hwipParams.args = (void *)pParams->callbackArg;
    #ifndef SDL_SOC_MCU_R5F
    /*
     * For M4F, external interrupt #10 at NVIC is
     * 16 internal interrupts + external interrupt number at NVIC
     */
    hwipParams.intNum = pParams->intNum + 16;
    #else
    hwipParams.intNum = pParams->intNum;
    #endif
    hwipParams.callback = (pSDL_DPL_InterruptCallbackFunction)PBIST_TEST_eventHandler;

    objNum = PBIST_TEST_findAvailObj();

    if (objNum != -1)
    {
        HwiP_construct(&gTestHwiObjects[objNum], &hwipParams);
    }

    return &gTestHwiObjects[objNum];
}

int32_t PBIST_TEST_enableInterrupt(uint32_t intNum)
{
    #ifndef SDL_SOC_MCU_R5F
    /*
     * For M4F, external interrupt #10 at NVIC is
     * 16 internal interrupts + external interrupt number at NVIC
     */
    HwiP_enableInt(intNum + 16);
    #else
    HwiP_enableInt(intNum);
    #endif
    return SDL_PASS;
}

int32_t PBIST_TEST_disableInterrupt(uint32_t intNum)
{
    #ifndef SDL_SOC_MCU_R5F
    /*
     * For M4F, external interrupt #10 at NVIC is
     * 16 internal interrupts + external interrupt number at NVIC
     */
    HwiP_disableInt(intNum + 16);
    #else
    HwiP_disableInt(intNum);
    #endif
    return SDL_PASS;
}

int32_t PBIST_TEST_disableInterrupt_error(uint32_t intNum)
{
    return SDL_EFAIL;
}

void* PBIST_TEST_addrTranslate(uint64_t addr, uint32_t size)
{
    uint32_t transAddr = (uint32_t)(-1);

    transAddr = (uint32_t)AddrTranslateP_getLocalAddr(addr);

    return (void *)transAddr;
}

void* PBIST_TEST_addrTranslate_error(uint64_t addr, uint32_t size)
{
    return (void*)(-1);
}

SDL_DPL_Interface test_dpl_interface1 =
{
    .enableInterrupt = (pSDL_DPL_InterruptFunction) PBIST_TEST_enableInterrupt,
    .disableInterrupt = (pSDL_DPL_InterruptFunction) PBIST_TEST_disableInterrupt_error,
    .registerInterrupt = (pSDL_DPL_RegisterFunction) PBIST_TEST_registerInterrupt,
    .deregisterInterrupt = (pSDL_DPL_DeregisterFunction) PBIST_TEST_deregisterInterrupt,
    .delay = (pSDL_DPL_DelayFunction) ClockP_sleep,
    .addrTranslate = (pSDL_DPL_AddrTranslateFunction) PBIST_TEST_addrTranslate
};

int32_t PBIST_TEST1_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_DPL_init(&test_dpl_interface1);

    return ret;
}

SDL_DPL_Interface test_dpl_interface2 =
{
    .enableInterrupt = (pSDL_DPL_InterruptFunction) PBIST_TEST_enableInterrupt,
    .disableInterrupt = (pSDL_DPL_InterruptFunction) PBIST_TEST_disableInterrupt,
    .registerInterrupt = (pSDL_DPL_RegisterFunction) PBIST_TEST_registerInterrupt_error,
    .deregisterInterrupt = (pSDL_DPL_DeregisterFunction) PBIST_TEST_deregisterInterrupt,
    .delay = (pSDL_DPL_DelayFunction) ClockP_sleep,
    .addrTranslate = (pSDL_DPL_AddrTranslateFunction) PBIST_TEST_addrTranslate
};

int32_t PBIST_TEST2_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_DPL_init(&test_dpl_interface2);

    return ret;
}

SDL_DPL_Interface test_dpl_interface3 =
{
    .enableInterrupt = (pSDL_DPL_InterruptFunction) PBIST_TEST_enableInterrupt,
    .disableInterrupt = (pSDL_DPL_InterruptFunction) PBIST_TEST_disableInterrupt,
    .registerInterrupt = (pSDL_DPL_RegisterFunction) PBIST_TEST_registerInterrupt,
    .deregisterInterrupt = (pSDL_DPL_DeregisterFunction) PBIST_TEST_deregisterInterrupt,
    .delay = (pSDL_DPL_DelayFunction) ClockP_sleep,
    .addrTranslate = (pSDL_DPL_AddrTranslateFunction) PBIST_TEST_addrTranslate_error
};

int32_t PBIST_TEST3_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_DPL_init(&test_dpl_interface3);

    return ret;
}

SDL_DPL_Interface test_dpl_interface4 =
{
    .enableInterrupt = (pSDL_DPL_InterruptFunction) PBIST_TEST_enableInterrupt,
    .disableInterrupt = (pSDL_DPL_InterruptFunction) PBIST_TEST_disableInterrupt,
    .registerInterrupt = (pSDL_DPL_RegisterFunction) PBIST_TEST_registerInterrupt_timeout,
    .deregisterInterrupt = (pSDL_DPL_DeregisterFunction) PBIST_TEST_deregisterInterrupt,
    .delay = (pSDL_DPL_DelayFunction) ClockP_sleep,
    .addrTranslate = (pSDL_DPL_AddrTranslateFunction) PBIST_TEST_addrTranslate
};

int32_t PBIST_TEST4_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_DPL_init(&test_dpl_interface4);

    return ret;
}
