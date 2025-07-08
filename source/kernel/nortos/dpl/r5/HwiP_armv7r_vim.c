/*
 *  Copyright (C) 2018-2023 Texas Instruments Incorporated
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
#include <drivers/hw_include/soc_config.h>

/** Each bit in the IRQPRIMSK corresponds to the priority level. 
 *  - bit = 1 -> Interrupts of this priority are enabled.
 *  - bit = 0 -> Interrupts of this priority are disabled.
 *
 * This macro computes the priority mask such that all interrupts of higher priority are only enabled.
 * i.e, all interrupts of priority value < `pri` are enabled (since lower the value higher the priority)
 * and all interrupts of priority value >= `pri` are disabled(masked).
 */
#define GET_PRIMSK_FROM_PRIORITY(pri)   ((uint32_t)((0x1U << (pri)) - 1U))

#define NOP1()  do { __asm__ __volatile__("NOP");            } while(0)
#define NOP5()  do { NOP1(); NOP1(); NOP1(); NOP1(); NOP1(); } while(0)
#define NOP10() do { NOP5(); NOP5();                         } while(0)

static volatile uint32_t gdummy;

static void Hwip_dataAndInstructionBarrier(void)
{
    __asm__ __volatile__ (" isb"   "\n\t": : : "memory");
    __asm__ __volatile__ (" dsb"   "\n\t": : : "memory");
}



HwiP_Ctrl gHwiCtrl;
#ifdef INTR_PROF
HwiP_Prof_Ctrl gHwiCtrlProf;
#endif

void HWI_SECTION HwiP_enableInt(uint32_t intNum)
{
    volatile uint32_t *addr;
    uint32_t bitPos;

    Hwip_dataAndInstructionBarrier();

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_INT_EN(intNum));
    bitPos = VIM_BIT_POS(intNum);

    *addr = ((uint32_t)0x1 << bitPos);
}

uint32_t HWI_SECTION HwiP_disableInt(uint32_t intNum)
{
    volatile uint32_t *addr;
    uint32_t bitPos;
    uint32_t isEnable = 0;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_INT_DIS(intNum));
    bitPos = VIM_BIT_POS(intNum);

    if( (*addr & ((uint32_t)0x1 << bitPos))!=0U)
    {
        isEnable = 1;
    }
    *addr = ((uint32_t)0x1 << bitPos);


    Hwip_dataAndInstructionBarrier();


    return isEnable;
}

void HWI_SECTION HwiP_restoreInt(uint32_t intNum, uint32_t oldIntState)
{
    if(oldIntState!=0U)
    {
        HwiP_enableInt(intNum);
    }
    else
    {
       (void) HwiP_disableInt(intNum);
    }
}

void HWI_SECTION HwiP_clearInt(uint32_t intNum)
{
    volatile uint32_t *addr;
    uint32_t bitPos;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_STS(intNum));
    bitPos = VIM_BIT_POS(intNum);

    *addr = ((uint32_t)0x1 << bitPos);
}

void HWI_SECTION HwiP_post(uint32_t intNum)
{
    volatile uint32_t *addr;
    uint32_t bitPos;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_RAW(intNum));
    bitPos = VIM_BIT_POS(intNum);

    *addr = ((uint32_t)0x1 << bitPos);

    /*
     * Add delay to insure posted interrupt are triggered before function
     * returns.
     */

    Hwip_dataAndInstructionBarrier();


}

void HWI_SECTION HwiP_Params_init(HwiP_Params *params)
{
    params->intNum = 0;
    params->callback = NULL;
    params->args = NULL;
    params->eventId = 0; /* NOT USED */
    params->priority = (HwiP_MAX_PRIORITY-1U);
    params->isFIQ = 0;
    params->isPulse = 0;
}

int32_t HWI_SECTION HwiP_construct(HwiP_Object *handle, HwiP_Params *params)
{
    HwiP_Object *obj = handle;

    DebugP_assertNoLog( params->callback != NULL );
    DebugP_assertNoLog( params->intNum < HwiP_MAX_INTERRUPTS );
    DebugP_assertNoLog( params->priority < HwiP_MAX_PRIORITY );

    (void) HwiP_disableInt(params->intNum);
    HwiP_clearInt(params->intNum);

    HwiP_setAsFIQ(params->intNum, params->isFIQ);
    HwiP_setPri(params->intNum, params->priority);
    HwiP_setAsPulse(params->intNum, params->isPulse);
    if(params->isFIQ != 0U)
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
    HwiP_Object *obj = handle;

    DebugP_assertNoLog( obj->intNum < HwiP_MAX_INTERRUPTS );

    gHwiCtrl.isrArgs[obj->intNum] = args;

    return SystemP_SUCCESS;
}

void HWI_SECTION HwiP_destruct(HwiP_Object *handle)
{
    HwiP_Object *obj = handle;

    /* disable interrupt, clear pending if any, make as pulse, ISR, lowest priority
     * set valid default vector address
     */
   (void) HwiP_disableInt(obj->intNum);
    HwiP_clearInt(obj->intNum);
    HwiP_setAsFIQ(obj->intNum, 0);
    HwiP_setPri(obj->intNum, HwiP_MAX_PRIORITY-1U);
    HwiP_setAsPulse(obj->intNum, 0);
    HwiP_setVecAddr(obj->intNum, (uintptr_t)HwiP_irq_handler);

    /* clear interrupt data structure */
    gHwiCtrl.isr[obj->intNum] = NULL;
    gHwiCtrl.isrArgs[obj->intNum] = NULL;
}

void HWI_SECTION HwiP_init(void)
{
    uint32_t i;

    /* disable IRQ */
    (void) HwiP_disable();
    /* disable FIQ */
    (void) HwiP_disableFIQ();

    DebugP_assertNoLog(gHwiConfig.intcBaseAddr != 0U);

    gHwiCtrl.spuriousIRQCount  = 0U;
    gHwiCtrl.spuriousFIQCount  = 0U;
    gHwiCtrl.activeIntPriority = HwiP_MAX_PRIORITY;

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
    for(i=0; i<(HwiP_MAX_INTERRUPTS/INTERRUPT_VALUE); i++)
    {
        volatile uint32_t *addr;

        /* disable all interrupts */
        addr = (uint32_t*)(gHwiConfig.intcBaseAddr + VIM_INT_DIS(i*INTERRUPT_VALUE));
        *addr = 0xFFFFFFFFu;

        /* clear all pending interrupts */
        addr = (uint32_t*)(gHwiConfig.intcBaseAddr + VIM_STS(i*INTERRUPT_VALUE));
        *addr = 0xFFFFFFFFu;

        /* make all as level */
        addr = (uint32_t*)(gHwiConfig.intcBaseAddr + VIM_INT_TYPE(i*INTERRUPT_VALUE));
        *addr = 0x0u;

        /* make all as IRQ */
        addr = (uint32_t*)(gHwiConfig.intcBaseAddr + VIM_INT_MAP(i*INTERRUPT_VALUE));
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
    #else
    #if defined (SOC_AM65X)
        HwiP_disableVIC();
    #endif
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

#ifdef __cplusplus
extern "C" {
#endif
uint32_t HwiP_getCPSR(void);
#ifdef __cplusplus
}
#endif

uint32_t HWI_SECTION HwiP_inISR(void)
{
    uint32_t mode = (HwiP_getCPSR() & 0x1FU);
    uint32_t result =0;
    if((mode != ARMV7R_SYSTEM_MODE) && (mode != ARMV7R_USER_MODE))
    {
         result= 1;
    }
    return result;
}

static inline uint32_t HWI_SECTION HwiP_setVimIrqPriMaskRawNonAtomic(uint32_t priMask)
{
    volatile uint32_t *addr;
    volatile uint32_t oldPriMask;

    /* Store the current priority mask and set to new value */
    addr       = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_IRQPRIMASK);
    oldPriMask = *addr;
    *addr      = priMask;

    return oldPriMask;
}

static uint32_t HWI_SECTION HwiP_setVimIrqPriMaskRawAtomic(uint32_t priMask)
{
    volatile uint32_t *addr;
    volatile uint32_t oldPriMask;
    uintptr_t oldIntrState;

    /* Disable the interrupts globally before modifying priority mask */
    oldIntrState = HwiP_disable();

    oldPriMask = HwiP_setVimIrqPriMaskRawNonAtomic(priMask);

    /* Wait for priority mask update to take effect */
    NOP10();

    /* Read & Write IRQVEC to force re-evaluation */
    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_IRQVEC);
    __asm__ __volatile__("LDR r2,[%0]"::"r"(addr):"r2");
    __asm__ __volatile__("STR r2,[%0]"::"r"(addr):"r2");

    /* Restore the interrupts globally */
    HwiP_restore(oldIntrState);

    return oldPriMask;
}

uint32_t HWI_SECTION HwiP_setVimIrqPriMaskNonAtomic(uint32_t priority)
{
    return HwiP_setVimIrqPriMaskRawNonAtomic(GET_PRIMSK_FROM_PRIORITY(priority));
}

uint32_t HWI_SECTION HwiP_setVimIrqPriMaskAtomic(uint32_t priority)
{
    return HwiP_setVimIrqPriMaskRawAtomic(GET_PRIMSK_FROM_PRIORITY(priority));
}

void HWI_SECTION HwiP_restoreVimIrqPriMask(uint32_t key)
{
    (void)HwiP_setVimIrqPriMaskRawNonAtomic(key);
}