/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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

#include <kernel/nortos/dpl/m4/HwiP_armv7m.h>
#include <drivers/hw_include/csl_types.h>
#include <ti_compatibility.h>

typedef struct HwiP_Struct_s {

    uint32_t intNum;

} HwiP_Struct;

HwiP_Ctrl gHwiCtrl;

void HWI_SECTION HwiP_enableInt(uint32_t intNum)
{
    volatile uint32_t *addr;

    if (intNum >= 16U)
    {
        /* interrupts external to M4F */
        uint32_t index, mask;

        index = (intNum - 16U) >> 5;
        mask = (uint32_t)1U << ((intNum - 16U) & 0x1fU);

        addr = NVIC_ISER(index);
        *addr = mask;
    }
    else
    if (intNum == 15U) /* SysTick interrupt */
    {
        uint32_t oldIntState;

        addr = SYSTICK_CSR;
        oldIntState = HwiP_disable();
        *addr |= 0x00000002U;   /* enable SysTick interrupt */
        HwiP_restore(oldIntState);
    }
}

uint32_t HWI_SECTION HwiP_disableInt(uint32_t intNum)
{
    volatile uint32_t *addr;
    uint32_t oldEnableState;

    if (intNum >= 16U)
    {
        /* interrupts external to M4F */
        uint32_t index, mask;

        index = (intNum - 16U) >> 5;
        mask = (uint32_t)1U << ((intNum - 16U) & 0x1fU);

        addr = NVIC_ISER(index);
        oldEnableState = *addr & mask;

        addr = NVIC_ICER(index);
        *addr = mask;
    }
    else
    if (intNum == 15U)
    {
        uint32_t oldIntState;

        addr = SYSTICK_CSR;
        oldIntState = HwiP_disable();
        oldEnableState = *addr & 0x00000002U;
        *addr &= ~0x00000002U;  /* disable SysTick interrupt */
        HwiP_restore(oldIntState);
    }
    else
    {
        oldEnableState = 0;
    }

    return oldEnableState;
}

void HWI_SECTION HwiP_restoreInt(uint32_t intNum, uint32_t oldEnableState)
{
    volatile uint32_t *addr;

    if (intNum >= 16U)
    {
        /* interrupts external to M4F */
        uint32_t index, mask;

        index = (intNum - 16U) >> 5;
        mask = (uint32_t)1U << ((intNum - 16U) & 0x1fU);

        if (oldEnableState != 0U)
        {
            addr = NVIC_ISER(index);
        }
        else
        {
            addr = NVIC_ICER(index);
        }
        *addr = mask;
    }
    else
    if (intNum == 15U)
    {
        uint32_t oldIntState;

        addr = SYSTICK_CSR;
        oldIntState = HwiP_disable();
        if (oldEnableState != 0U) {
            *addr |= 0x00000002U;       /* enable SysTick Int */
        }
        else {
            *addr &= ~0x00000002U;      /* disable SysTick Int */
        }
        HwiP_restore(oldIntState);
    }

}

void HWI_SECTION HwiP_clearInt(uint32_t intNum)
{
    volatile uint32_t *addr;

    /* only valid for interrupts external to M4F */
    if (intNum >= 16U)
    {
        uint32_t index, mask;

        index = (intNum - 16U) >> 5;
        mask = (uint32_t)1U << ((intNum - 16U) & 0x1fU);

        addr = NVIC_ICPR(index);
        *addr = mask;
    }
}

void HWI_SECTION HwiP_setPri(uint32_t intNum, uint32_t priority)
{
    if(priority < HwiP_MAX_PRIORITY)
    {
        volatile uint32_t *addr;

        priority = (priority << HwiP_NVIC_PRI_SHIFT);
        /* User interrupt (id >= 16) priorities are set in the IPR registers */
        if (intNum >= 16U) {
            uint32_t index, mask, shift;

            shift = (((intNum - 16U) & 0x3U) * 8);
            mask  = 0xFF << shift;
            index = (intNum - 16U) / 4;

            addr = NVIC_IPRI(index);
            *addr &= ~(mask); /* clear priority */
            *addr |= (priority << shift); /* set priority */
        }
        else
        if (intNum >= 4U)
        {
            /* System interrupt (id >= 4) priorities are set in the SHPR registers */
            uint32_t index, mask, shift;

            shift = (((intNum - 4U) & 0x3U) * 8);
            mask  = 0xFF << shift;
            index = (intNum - 4U) / 4;

            addr = SHPR(index);
            *addr &= ~(mask); /* clear priority */
            *addr |= (priority << shift); /* set priority */
        }
        else
        {
            /* System interrupts (id < 4) priorities  are fixed in hardware */
        }
    }
}

void HWI_SECTION HwiP_Params_init(HwiP_Params *params)
{
    params->intNum = 0;
    params->callback = NULL;
    params->args = NULL;
    params->eventId = 0; /* NOT USED */
    params->priority = HwiP_MAX_PRIORITY-1; /* set default as lowest priority */
    params->isFIQ = 0; /* NOT USED */
    params->isPulse = 0; /* NOT USED */
}

int32_t HWI_SECTION HwiP_construct(HwiP_Object *handle, HwiP_Params *params)
{
    HwiP_Struct *obj = (HwiP_Struct *)handle;

    DebugP_assertNoLog( sizeof(HwiP_Struct) <= sizeof(HwiP_Object) );
    DebugP_assertNoLog( params->callback != NULL );
    DebugP_assertNoLog( params->intNum < HwiP_MAX_INTERRUPTS );
    DebugP_assertNoLog( params->priority < HwiP_MAX_PRIORITY );
    /* can register user handlers only for systick and external NVIC interrupts */
    DebugP_assertNoLog( params->intNum >= 15 );

    HwiP_disableInt(params->intNum);
    HwiP_clearInt(params->intNum);
    HwiP_setPri(params->intNum, params->priority);

    gHwiCtrl.isr[params->intNum] = params->callback;
    gHwiCtrl.isrArgs[params->intNum] = params->args;

    obj->intNum = params->intNum;

    HwiP_enableInt(params->intNum);

    return SystemP_SUCCESS;
}

int32_t HwiP_setArgs(HwiP_Object *handle, void *args)
{
    HwiP_Struct *obj = (HwiP_Struct *)handle;

    DebugP_assertNoLog( obj->intNum < HwiP_MAX_INTERRUPTS );

    gHwiCtrl.isrArgs[obj->intNum] = args;

    return SystemP_SUCCESS;
}

void HWI_SECTION HwiP_destruct(HwiP_Object *handle)
{
    HwiP_Struct *obj = (HwiP_Struct *)handle;

    /* disable interrupt, clear pending if any, make as lowest priority
     */
    HwiP_disableInt(obj->intNum);
    HwiP_clearInt(obj->intNum);
    HwiP_setPri(obj->intNum, HwiP_MAX_PRIORITY-1);

    /* clear interrupt data structure */
    gHwiCtrl.isr[obj->intNum] = NULL;
    gHwiCtrl.isrArgs[obj->intNum] = NULL;
}

void HWI_SECTION HwiP_post(uint32_t intNum)
{
    volatile uint32_t *addr;

    /* only valid for interrupts external to M4F */
    if (intNum >= 16U)
    {
        addr = STIR;
        *addr = (intNum - 16U);
    }
}

uintptr_t HWI_SECTION HwiP_disable()
{
    return _set_interrupt_priority(HwiP_NVIC_PRI_DISABLE);
}

void HWI_SECTION HwiP_enable()
{
    _set_interrupt_priority(0u);
}

void HWI_SECTION HwiP_restore(uintptr_t oldIntState)
{
    _set_interrupt_priority(oldIntState);
}

void HWI_SECTION HwiP_init()
{
    uint32_t i;
    volatile uint32_t *addr;

    /* disable interrupts */
    HwiP_disable();

    addr = VTOR;
    *addr = (uint32_t)gHwiP_vectorTable;

    /* initalize local data structure, and set all interrupts to lowest priority
     * and set ISR address as IRQ handler
     */
    for(i=0; i<HwiP_MAX_INTERRUPTS;i++)
    {
        gHwiCtrl.isr[i] = NULL;
        gHwiCtrl.isrArgs[i] = NULL;

        if(i >= 15)
        {
            /* handler setup, interrupt disable/clear is allowed only for systick and external MVIC interrupts
             */
            void HwiP_interrupt_handler();

            gHwiP_vectorTable[i] = (uint32_t)&HwiP_interrupt_handler;

            HwiP_disableInt(i);
            HwiP_clearInt(i);
        }

        /* keep all interrupt as low priority by default */
        HwiP_setPri(i, HwiP_MAX_PRIORITY-1);
    }

    /* keep interrupt disabled, they should be enabled during system init
     * after all driver is done
     */
}

uint32_t HwiP_inISR(void)
{
    uint32_t stat;

    if (( *ICSR & 0x000001ff) == 0)
    {
        stat = 0;
    }
    else
    {
        stat = 1;
    }

    return (stat);
}
