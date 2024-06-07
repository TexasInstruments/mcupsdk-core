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
#include <drivers/pmu.h>

static volatile uint32_t gdummy;


/* compile flag to enable or disable interrupt nesting */
#define HWIP_NESTED_INTERRUPTS_IRQ_ENABLE

/* Save FPU context, used in FIQ Handler */
static inline  void Hwip_save_fpu_context(void)
{
    __asm__ __volatile__ ( "FMRX  R0, FPSCR"  "\n\t": : : "memory");
    __asm__ __volatile__ ( "VPUSH {D0-D15}"  "\n\t": : : "memory");
    __asm__ __volatile__ ( "PUSH  {R0}"  "\n\t": : : "memory");
}

/* Restore FPU context, used in FIQ Handler */
static inline  void Hwip_restore_fpu_context(void)
{
    __asm__ __volatile__ ( "POP   {R0}"  "\n\t": : : "memory");
    __asm__ __volatile__ ( "VPOP  {D0-D15}"  "\n\t": : : "memory");
    __asm__ __volatile__ ( "VMSR  FPSCR, R0"  "\n\t": : : "memory");
}

void __attribute__((section(".text.hwi"))) HwiP_irq_handler_c(void);
void __attribute__((interrupt("UNDEF"), section(".text.hwi"))) HwiP_reserved_handler(void);
void __attribute__((interrupt("UNDEF"), section(".text.hwi"))) HwiP_undefined_handler(void);
void __attribute__((interrupt("ABORT"), section(".text.hwi"))) HwiP_prefetch_abort_handler(void);
#ifdef __cplusplus
extern "C" {
#endif
void __attribute__((interrupt("ABORT"), section(".text.hwi"))) HwiP_data_abort_handler(void);
#ifdef __cplusplus
}
#endif

#ifdef INTR_PROF
void __attribute__((section(".text.hwi"))) HwiP_irq_profile_c(uint32_t intNum)
{
    uint32_t idx = 0, exists = 1;

    for(idx = 0; idx <= gHwiCtrlProf.traceInterruptedISRIndex; idx++)
    {
        if(gHwiCtrlProf.traceInterruptedISR[idx] == intNum)
        {
            gHwiCtrlProf.traceInterruptedISR[idx] = 0;
            gHwiCtrlProf.traceInterruptedISRIndex --;
            if(gHwiCtrlProf.traceInterruptedISRIndex == 0)
            {
                gHwiCtrlProf.readCounterStop = CSL_armR5PmuReadCntr(CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM);
                if(gHwiCtrlProf.readCounterStop > gHwiCtrlProf.readCounterStart)
                {
                    gHwiCtrlProf.pmuCountVal += gHwiCtrlProf.readCounterStop - gHwiCtrlProf.readCounterStart - gHwiCtrlProf.pmuCalibration;
                }
                else
                {
                    gHwiCtrlProf.pmuCountVal += (0xFFFFFFFFU - gHwiCtrlProf.readCounterStart) + gHwiCtrlProf.readCounterStop - gHwiCtrlProf.pmuCalibration;
                }
            }
            exists = 0;
            break;
        }
    }
    if(exists)
    {
        gHwiCtrlProf.traceInterruptedISR[(gHwiCtrlProf.traceInterruptedISRIndex)] = intNum;
        gHwiCtrlProf.traceInterruptedISRIndex ++;
        gHwiCtrlProf.readCounterStart = CSL_armR5PmuReadCntr(CSL_ARM_R5_PMU_CYCLE_COUNTER_NUM);
    }
}
#endif

/* IRQ handler starts execution in HwiP_irq_handler, defined in portASM.S
 * After some initial assembly logic it then branches to this function.
 * After exiting this function it does some more assembly including
 * doing task switch if requested.
 */
void __attribute__((section(".text.hwi"))) HwiP_irq_handler_c(void)
{
    int32_t status;
    uint32_t intNum;

    #ifndef HWIP_VIM_VIC_ENABLE

    /* Read to force prioritization logic to take effect */
    HwiP_getIRQVecAddr();
    #endif

    status = HwiP_getIRQ(&intNum);
    if(status==SystemP_SUCCESS)
    {
        #ifdef INTR_PROF
        if(gHwiCtrlProf.profileIntr == 1)
        {
            HwiP_irq_profile_c(intNum);
        }
        #endif

        uint32_t isPulse = HwiP_isPulse(intNum);
        HwiP_FxnCallback isr;
        void *args;

        if(isPulse != 0U)
        {
            HwiP_clearInt(intNum);
        }

        isr = gHwiCtrl.isr[intNum];
        args = gHwiCtrl.isrArgs[intNum];

        #ifdef HWIP_NESTED_INTERRUPTS_IRQ_ENABLE
        /* allow nesting of interrupts */
        HwiP_enable();
        #endif

        if(isr!=NULL)
        {
            isr(args);
        }

        /* disallow nesting of interrupts */
        (void)HwiP_disable();

        if(isPulse == 0U)
        {
            HwiP_clearInt(intNum);
        }
        HwiP_ackIRQ(intNum);

        #ifdef INTR_PROF
        if(gHwiCtrlProf.profileIntr == 1)
        {
            HwiP_irq_profile_c(intNum);
        }
        #endif
    }
    else
    {
        /* spurious interrupt */
        gHwiCtrl.spuriousIRQCount++;
        HwiP_ackIRQ(0);
    }
}

void __attribute__((interrupt("FIQ"), section(".text.hwi"))) HwiP_fiq_handler(void)
{
    int32_t status;
    uint32_t intNum;

    #ifdef EN_SAVE_RESTORE_FPU_CONTEXT
    Hwip_save_fpu_context();
    #endif

    /* Read to force prioritization logic to take effect */
    gdummy = HwiP_getFIQVecAddr();

    status = HwiP_getFIQ(&intNum);
    if(status==SystemP_SUCCESS)
    {
        uint32_t isPulse = HwiP_isPulse(intNum);
        HwiP_FxnCallback isr;
        void *args;

        if(isPulse != 0U)
        {
            HwiP_clearInt(intNum);
        }

        isr = gHwiCtrl.isr[intNum];
        args = gHwiCtrl.isrArgs[intNum];

        #if 0   /* FIQ interrupt nesting not supported */
        /* allow nesting of interrupts */
        HwiP_enableFIQ();
        #endif

        if(isr!=NULL)
        {
            isr(args);
        }

        /* disallow nesting of interrupts */
        (void)HwiP_disableFIQ();

        if(isPulse == 0U)
        {
            HwiP_clearInt(intNum);
        }
        HwiP_ackFIQ(intNum);
    }
    else
    {
        /* spurious interrupt */
        gHwiCtrl.spuriousFIQCount++;
        HwiP_ackFIQ(0);
    }

    #ifdef EN_SAVE_RESTORE_FPU_CONTEXT
    Hwip_restore_fpu_context();
    #endif
}

void __attribute__((interrupt("UNDEF"), section(".text.hwi"))) HwiP_reserved_handler(void)
{
    volatile uint32_t loop = 1;
    while(loop != 0U) { ; }

}

void __attribute__((interrupt("UNDEF"), section(".text.hwi"))) HwiP_undefined_handler(void)
{
    volatile uint32_t loop = 1;
    while(loop != 0U){  ; }

}

void __attribute__((interrupt("ABORT"), section(".text.hwi"))) HwiP_prefetch_abort_handler(void)
{
    volatile uint32_t loop = 1;
    while(loop != 0U)  { ;}
}

void __attribute__((interrupt("ABORT"), section(".text.hwi"), weak)) HwiP_data_abort_handler_c(void)
{
    volatile uint32_t loop = 1;
    while(loop != 0U){ ; }

}
