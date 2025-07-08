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
#include <kernel/dpl/DebugP.h>

#define TEXT_HWI    __attribute__((section(".text.hwi")))
#define WEAK        __attribute__((weak))

#ifdef __cplusplus
extern "C" {
#endif

/* Following handlers are set directly in vector table, hence use interrupt attribute */
void TEXT_HWI __attribute__((interrupt("UNDEF"))) HwiP_reserved_handler(void);
void TEXT_HWI __attribute__((interrupt("FIQ"))) HwiP_fiq_handler(void);
void TEXT_HWI __attribute__((interrupt("SWI"))) HwiP_svc_handler(void);

/** Following handlers starts in assembly and switch to C implementation. 
 * Hence DON'T use interrupt attribute. Else it will result in re-entry */ 
void TEXT_HWI HwiP_irq_handler_c(void);
void TEXT_HWI HwiP_data_abort_handler_c(volatile uint32_t LR);
void TEXT_HWI HwiP_prefetch_abort_handler_c(volatile uint32_t LR);
void TEXT_HWI HwiP_undefined_handler_c(volatile uint32_t LR);
/* Use weak attribute with user handler so that applications can override with its own implementation */
void TEXT_HWI WEAK HwiP_user_data_abort_handler_c(DFSR dfsr, ADFSR adfsr, volatile uint32_t DFAR, 
                                                  volatile uint32_t LR,volatile uint32_t SPSR);
void TEXT_HWI WEAK HwiP_user_prefetch_abort_handler_c(IFSR ifsr, AIFSR aifsr, volatile uint32_t IFAR, 
                                                      volatile uint32_t LR,volatile uint32_t SPSR);
void TEXT_HWI WEAK HwiP_user_undefined_handler_c(volatile uint32_t LR, volatile uint32_t SPSR);

volatile uint32_t GET_DFSR(void);
volatile uint32_t GET_ADFSR(void);
volatile uint32_t GET_DFAR(void);
volatile uint32_t GET_IFSR(void);
volatile uint32_t GET_AIFSR(void);
volatile uint32_t GET_IFAR(void);
volatile uint32_t GET_SPSR(void);
volatile uint32_t GET_LR(void);

#ifdef __cplusplus
}
#endif

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


/* IRQ handler starts execution in HwiP_irq_handler, defined in
 * HwiP_armv7r_handlers_nortos_asm.S
 * After some initial assembly logic it then branches to this function.
 * After exiting this function it does some more assembly before exiting
 */
void TEXT_HWI HwiP_irq_handler_c(void)
{
    int32_t status;
    uint32_t intNum, priority;

    #ifndef HWIP_VIM_VIC_ENABLE

    /* Read to force prioritization logic to take effect, in non-VIC mode */
    HwiP_getIRQVecAddr();
    #endif

    status = HwiP_getIRQ(&intNum, &priority);
    if(status==SystemP_SUCCESS)
    {
        /* Store the current active interrupt priority in software for any later use.
         * `oldPriority` will be used to restore upon exiting from this handler. */
        uint32_t oldPriority = HwiP_updateActivePrioritySW(priority);

        uint32_t isPulse = HwiP_isPulse(intNum);
        HwiP_FxnCallback isr;
        void *args;

        #ifdef HWIP_USE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS
        uint32_t priMaskKey;
        #endif

        if(isPulse!=0U)
        {
            HwiP_clearInt(intNum);
        }

        isr = gHwiCtrl.isr[intNum];
        args = gHwiCtrl.isrArgs[intNum];

        #ifdef HWIP_USE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS
        priMaskKey = HwiP_setVimIrqPriMaskAtomic(priority);
        #endif

        #ifdef HWIP_NESTED_INTERRUPTS_IRQ_ENABLE
        /* allow nesting of interrupts */
        HwiP_enable();
        #endif

        if(isr!=NULL)
        {
            isr(args);
        }

        /* disallow nesting of interrupts */
        (void) HwiP_disable();

        if(isPulse==0U)
        {
            HwiP_clearInt(intNum);
        }
        
        #ifdef HWIP_USE_INTERRUPT_PRIORITY_BASED_CRITICAL_SECTIONS
        /* Note: Restore the priority mask only after clearing the interrupt. 
         *       Else VIM would fire this again. */
        HwiP_restoreVimIrqPriMask(priMaskKey);
        #endif

        HwiP_ackIRQ(intNum);

        HwiP_restoreActivePrioritySW(oldPriority);
    }
    else
    {
        /* spurious interrupt */
        gHwiCtrl.spuriousIRQCount++;
        HwiP_ackIRQ(0);
    }
}

void TEXT_HWI __attribute__((interrupt("FIQ"))) HwiP_fiq_handler(void)
{
    int32_t status;
    uint32_t intNum;
    volatile uint32_t dummy;

    #ifdef HWIP_FPU_CONTEXT_SAVE_RESTORE_ENABLE
    Hwip_save_fpu_context();
    #endif

    /* Read to force prioritization logic to take effect */
    dummy = HwiP_getFIQVecAddr();
    (void)dummy;

    status = HwiP_getFIQ(&intNum);
    if(status==SystemP_SUCCESS)
    {
        uint32_t isPulse = HwiP_isPulse(intNum);
        HwiP_FxnCallback isr;
        void *args;

        if(isPulse!=0U)
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
        (void) HwiP_disableFIQ();

        if(isPulse==0U)
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

    #ifdef HWIP_FPU_CONTEXT_SAVE_RESTORE_ENABLE
    Hwip_restore_fpu_context();
    #endif
}

void TEXT_HWI __attribute__((interrupt("UNDEF"))) HwiP_reserved_handler(void)
{
    volatile uint32_t loop = 1;
    while(loop!=0U)
    {
        ;
    }
}

/* Undefined handler starts execution in HwiP_undefined_handler, defined in
 * HwiP_armv7r_handlers_nortos_asm.S
 * After some initial assembly logic it then branches to this function.
 * After exiting this function it does some more assembly before exiting
 */
void TEXT_HWI HwiP_undefined_handler_c(volatile uint32_t LR)
{

    typedef struct {
        volatile uint32_t SPSR;
        /* DFSR register */
        volatile uint32_t LR;
        /* Instruction causing the exception*/
    }UNDEF_REG;

    UNDEF_REG abort_regs;
    abort_regs.SPSR = GET_SPSR();
    abort_regs.LR   = LR;

    HwiP_user_undefined_handler_c(abort_regs.LR, abort_regs.SPSR);

}

void __attribute__((interrupt("SWI"), section(".text.hwi"))) HwiP_svc_handler(void)
{
    volatile uint32_t loop = 1;
    while(loop!=0U)
    {
        ;
    }

}

/* Prefetch abort handler starts execution in HwiP_prefetch_abort_handler, defined in
 * HwiP_armv7r_handlers_nortos_asm.S
 * After some initial assembly logic it then branches to this function.
 * After exiting this function it does some more assembly before exiting
 */
void TEXT_HWI HwiP_prefetch_abort_handler_c(volatile uint32_t LR)
{

    typedef struct {
        volatile uint32_t IFSR;
        /* IFSR register */
        volatile uint32_t AIFSR;
        /* AIFSR register */
        volatile uint32_t IFAR;
        /* IFAR register */
        volatile uint32_t LR;
        /* Instruction causing the exception*/
        volatile uint32_t SPSR;
        /* SPSR register*/
    }PREFETCH_ABORT_REG;

    PREFETCH_ABORT_REG abort_regs;

    /*Extract register values through functions coded in ASM*/
    abort_regs.AIFSR = GET_AIFSR();
    abort_regs.IFAR  = GET_IFAR();
    abort_regs.IFSR  = GET_IFSR();
    abort_regs.LR    = LR;
    abort_regs.SPSR  = GET_SPSR();

    /*Extract contents of IFSR register
    1. status: indicates the type of fault generated
    2. sd: distinguishes between an AXI Decode or Slave error on an external abort.
    This bit is only valid for external aborts. For all other aborts types of abort,
    this bit is set to zero*/
    IFSR     ifsr;
    uint32_t ifsr_value = abort_regs.IFSR;

    ifsr.status = ((ifsr_value & 0xF) | (((ifsr_value >> 10) & 0x1) << 4));
    ifsr.sd     = ((ifsr_value >> 12) & 0x1);

    /*Extract contents of AIFSR register
    1. index: returns the index value for the access giving the error
    2. side_ext: value returned in this field indicates the source of the error
    3. recoverable_error:  value returned in this field indicates if the error is recoverable
        (0=Unrecoverable error, 1=Recoverable Error)
    4. cacheway: value returned in this field indicates the cache way or ways in which the error occurred*/
    AIFSR    aifsr;
    uint32_t aifsr_value = abort_regs.AIFSR;

    aifsr.index             = ((aifsr_value >> 5) & 0x1FF);
    aifsr.side_ext          = (((aifsr_value >> 22) & 0x3) | (((aifsr_value >> 20) & 0x1) << 2));
    aifsr.recoverable_error = ((aifsr_value >> 21) & 0x1);
    aifsr.cacheway          = ((aifsr_value >> 24) & 0xF);

    HwiP_user_prefetch_abort_handler_c(ifsr, aifsr, abort_regs.IFAR, abort_regs.LR, abort_regs.SPSR);

}

/* Data abort handler starts execution in HwiP_data_abort_handler, defined in
 * HwiP_armv7r_handlers_nortos_asm.S
 * After some initial assembly logic it then branches to this function.
 * After exiting this function it does some more assembly before exiting
 */
void TEXT_HWI HwiP_data_abort_handler_c(volatile uint32_t LR)
{

    typedef struct {
        volatile uint32_t DFSR;
        /* DFSR register */
        volatile uint32_t ADFSR;
        /* ADFSR register */
        volatile uint32_t DFAR;
        /* DFAR register */
        volatile uint32_t LR;
        /* Instruction causing the exception*/
        volatile uint32_t SPSR;
        /* SPSR register*/
    }DATA_ABORT_REG;

    /*Extract register values through functions coded in ASM*/
    DATA_ABORT_REG abort_regs;
    abort_regs.ADFSR = GET_ADFSR();
    abort_regs.DFAR  = GET_DFAR();
    abort_regs.DFSR  = GET_DFSR();
    abort_regs.LR    = LR;
    abort_regs.SPSR  = GET_SPSR();

    /*Extract contents of DFSR register
    1. status: indicates the type of fault generated
    2. sd: distinguishes between an AXI Decode or Slave error on an external abort.
    This bit is only valid for external aborts. For all other aborts types of abort,
    this bit is set to zero
    3. rw:  Indicates whether a read or write access caused an abort
        (0=read abort; 1=write abort)*/
    DFSR dfsr;
    uint32_t dfsr_value = abort_regs.DFSR;

    dfsr.status = ((dfsr_value & 0xF) | (((dfsr_value >> 10) & 0x1) << 4));
    dfsr.rw     = ((dfsr_value >> 11) & 0x1);
    dfsr.sd     = ((dfsr_value >> 12) & 0x1);

    /*Extract contents of ADFSR register
    1. index: returns the index value for the access giving the error
    2. side_ext: value returned in this field indicates the source of the error
    3. recoverable_error:  value returned in this field indicates if the error is recoverable
        (0=Unrecoverable error, 1=Recoverable Error)
    4. cacheway: value returned in this field indicates the cache way or ways in which the error occurred*/
    ADFSR adfsr;
    uint32_t adfsr_value = abort_regs.ADFSR; 

    adfsr.index             = ((adfsr_value >> 5) & 0x1FF);
    adfsr.side_ext          = (((adfsr_value >> 22) & 0x3) | (((adfsr_value >> 20) & 0x1) << 2));
    adfsr.recoverable_error = ((adfsr_value >> 21) & 0x1);
    adfsr.cacheway          = ((adfsr_value >> 24) & 0xF);

    HwiP_user_data_abort_handler_c(dfsr, adfsr, abort_regs.DFAR, abort_regs.LR, abort_regs.SPSR);

}

void TEXT_HWI WEAK HwiP_user_undefined_handler_c(volatile uint32_t LR, volatile uint32_t SPSR)
{
    volatile uint32_t loop = 1U;
    while(loop != 0U){ ; }
}

void TEXT_HWI WEAK HwiP_user_prefetch_abort_handler_c(IFSR ifsr, AIFSR aifsr, volatile uint32_t IFAR, 
                                                      volatile uint32_t LR,volatile uint32_t SPSR)
{
    volatile uint32_t loop = 1U;
    while(loop != 0U){ ; }
}

void TEXT_HWI WEAK HwiP_user_data_abort_handler_c(DFSR dfsr, ADFSR adfsr, volatile uint32_t DFAR, 
                                                  volatile uint32_t LR,volatile uint32_t SPSR)
{
    volatile uint32_t loop = 1U;
    while(loop != 0U){ ; }
}
