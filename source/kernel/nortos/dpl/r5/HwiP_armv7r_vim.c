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

#include <kernel/dpl/HwiP.h>
#include <kernel/nortos/dpl/r5/HwiP_armv7r_vim.h>
#include <drivers/hw_include/csl_types.h>

static volatile uint32_t gdummy;

typedef struct HwiP_Struct_s {

    uint32_t intNum;

} HwiP_Struct;

HwiP_Ctrl gHwiCtrl;

void HWI_SECTION HwiP_enableInt(uint32_t intNum)
{
    volatile uint32_t *addr;
    uint32_t bitPos;

    __asm__ __volatile__ (" isb"   "\n\t": : : "memory");
    __asm__ __volatile__ (" dsb"   "\n\t": : : "memory");

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_INT_EN(intNum));
    bitPos = VIM_BIT_POS(intNum);

    *addr = (0x1u << bitPos);
}

uint32_t HWI_SECTION HwiP_disableInt(uint32_t intNum)
{
    volatile uint32_t *addr;
    uint32_t bitPos;
    uint32_t isEnable = 0;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_INT_DIS(intNum));
    bitPos = VIM_BIT_POS(intNum);

    if( (*addr & (0x1u << bitPos)))
    {
        isEnable = 1;
    }
    *addr = (0x1u << bitPos);

    __asm__ __volatile__ (" isb"  "\n\t": : : "memory");
    __asm__ __volatile__ (" dsb"  "\n\t": : : "memory");

    return isEnable;
}

void HWI_SECTION HwiP_restoreInt(uint32_t intNum, uint32_t oldIntState)
{
    if(oldIntState)
    {
        HwiP_enableInt(intNum);
    }
    else
    {
        HwiP_disableInt(intNum);
    }
}

void HWI_SECTION HwiP_clearInt(uint32_t intNum)
{
    volatile uint32_t *addr;
    uint32_t bitPos;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_STS(intNum));
    bitPos = VIM_BIT_POS(intNum);

    *addr = ( 0x1u << bitPos);
}

void HWI_SECTION HwiP_post(uint32_t intNum)
{
    volatile uint32_t *addr;
    uint32_t bitPos;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_RAW(intNum));
    bitPos = VIM_BIT_POS(intNum);

    *addr = ( 0x1u << bitPos);

    /*
     * Add delay to insure posted interrupt are triggered before function
     * returns.
     */
    __asm__  __volatile__ (" dsb" "\n\t": : : "memory");
    __asm__  __volatile__ (" isb" "\n\t": : : "memory");

}

void HWI_SECTION HwiP_Params_init(HwiP_Params *params)
{
    params->intNum = 0;
    params->callback = NULL;
    params->args = NULL;
    params->eventId = 0; /* NOT USED */
    params->priority = (HwiP_MAX_PRIORITY-1);
    params->isFIQ = 0;
    params->isPulse = 0;
}

int32_t HWI_SECTION HwiP_construct(HwiP_Object *handle, HwiP_Params *params)
{
    HwiP_Struct *obj = (HwiP_Struct *)handle;

    DebugP_assertNoLog( sizeof(HwiP_Struct) <= sizeof(HwiP_Object) );
    DebugP_assertNoLog( params->callback != NULL );
    DebugP_assertNoLog( params->intNum < HwiP_MAX_INTERRUPTS );
    DebugP_assertNoLog( params->priority < HwiP_MAX_PRIORITY );

    HwiP_disableInt(params->intNum);
    HwiP_clearInt(params->intNum);

    HwiP_setAsFIQ(params->intNum, params->isFIQ);
    HwiP_setPri(params->intNum, params->priority);
    HwiP_setAsPulse(params->intNum, params->isPulse);
    if(params->isFIQ)
    {
        HwiP_setVecAddr(params->intNum, (uintptr_t)HwiP_fiq_handler);
    }
    else
    {
        HwiP_setVecAddr(params->intNum, (uintptr_t)HwiP_irq_handler);
    }

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

    /* disable interrupt, clear pending if any, make as pulse, ISR, lowest priority
     * set valid default vector address
     */
    HwiP_disableInt(obj->intNum);
    HwiP_clearInt(obj->intNum);
    HwiP_setAsFIQ(obj->intNum, 0);
    HwiP_setPri(obj->intNum, HwiP_MAX_PRIORITY-1);
    HwiP_setAsPulse(obj->intNum, 0);
    HwiP_setVecAddr(obj->intNum, (uintptr_t)HwiP_irq_handler);

    /* clear interrupt data structure */
    gHwiCtrl.isr[obj->intNum] = NULL;
    gHwiCtrl.isrArgs[obj->intNum] = NULL;
}

void HWI_SECTION HwiP_init()
{
    uint32_t i;

    /* disable IRQ */
    HwiP_disable();
    /* disable FIQ */
    HwiP_disableFIQ();

    DebugP_assertNoLog(gHwiConfig.intcBaseAddr != 0);

    gHwiCtrl.spuriousIRQCount = 0;
    gHwiCtrl.spuriousFIQCount = 0;

    /* initalize local data structure, and set all interrupts to lowest priority
     * and set ISR address as IRQ handler
     */
    for(i=0; i<HwiP_MAX_INTERRUPTS; i++)
    {
        gHwiCtrl.isr[i] = NULL;
        gHwiCtrl.isrArgs[i] = NULL;

        HwiP_setPri(i, 0xF);
        HwiP_setVecAddr(i, (uintptr_t)HwiP_irq_handler);
    }

    /* disable, clear, set as IRQ and level, all interrupts */
    for(i=0; i<HwiP_MAX_INTERRUPTS/32; i++)
    {
        volatile uint32_t *addr;

        /* disable all interrupts */
        addr = (uint32_t*)(gHwiConfig.intcBaseAddr + VIM_INT_DIS(i*32));
        *addr = 0xFFFFFFFFu;

        /* clear all pending interrupts */
        addr = (uint32_t*)(gHwiConfig.intcBaseAddr + VIM_STS(i*32));
        *addr = 0xFFFFFFFFu;

        /* make all as level */
        addr = (uint32_t*)(gHwiConfig.intcBaseAddr + VIM_INT_TYPE(i*32));
        *addr = 0x0u;

        /* make all as IRQ */
        addr = (uint32_t*)(gHwiConfig.intcBaseAddr + VIM_INT_MAP(i*32));
        *addr = 0x0u;
    }

    /* ACK and clear any pending request */
    {
        gdummy = HwiP_getIRQVecAddr();
        gdummy = HwiP_getFIQVecAddr();
        HwiP_ackIRQ(0);
        HwiP_ackFIQ(0);
    }

    #ifdef HWIP_VIM_VIC_ENABLE
    HwiP_enableVIC();
    #endif
    HwiP_enableFIQ();
    /* dont enable IRQ, enable it after
     * all init is to make sure any pending peripheral interrupt status bits from old runs are cleared
     * before interrupts are eanbled.
     *
     * This allows to reload the binary without power cycling the whole SOC
     */
    /* HwiP_enable(); */
}

uint32_t HwiP_getCPSR();

uint32_t HWI_SECTION HwiP_inISR(void)
{
    uint32_t mode = (HwiP_getCPSR() & 0x1F);

    return (uint32_t)(mode != ARMV7R_SYSTEM_MODE);
}