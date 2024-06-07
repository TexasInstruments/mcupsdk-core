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

#ifndef HWIP_ARMV7R_VIM_H
#define HWIP_ARMV7R_VIM_H

#ifdef __cplusplus
extern "C"
{
#endif


#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/soc_config.h>

/* compile flag to enable VIC mode of operation, undef this to use non-VIC mode */
#if !defined (SOC_AM65X)
#define HWIP_VIM_VIC_ENABLE
#endif


#define HWI_SECTION __attribute__((section(".text.hwi")))

#define HwiP_MAX_INTERRUPTS     (512u)
#define HwiP_MAX_PRIORITY       (16u)

#define VIM_BIT_POS(j)   ( (j) & 0x1Fu )
#define VIM_IRQVEC       (0x18u)
#define VIM_FIQVEC       (0x1Cu)
#define VIM_ACTIRQ       (0x20u)
#define VIM_ACTFIQ       (0x24u)
#define VIM_IRQPRIMASK   (0x28u)
#define VIM_RAW(j)       (0x400U + ((((j) >> 5) & 0xFU) * 0x20U))
#define VIM_STS(j)       (0x404U + ((((j) >> 5) & 0xFU) * 0x20U))
#define VIM_INT_EN(j)    (0x408U + ((((j) >> 5) & 0xFU) * 0x20U))
#define VIM_INT_DIS(j)   (0x40CU + ((((j) >> 5) & 0xFU) * 0x20U))
#define VIM_INT_MAP(j)   (0x418U + ((((j) >> 5) & 0xFU) * 0x20U))
#define VIM_INT_TYPE(j)  (0x41CU + ((((j) >> 5) & 0xFU) * 0x20U))
#define VIM_INT_PRI(j)   (0x1000u + ((j) * 0x4u))
#define VIM_INT_VEC(j)   (0x2000u + ((j) * 0x4u))

#define ARMV7R_FIQ_MODE       (0x11u)
#define ARMV7R_IRQ_MODE       (0x12u)
#define ARMV7R_SVC_MODE       (0x13u)
#define ARMV7R_SYSTEM_MODE    (0x1Fu)

#define INTERRUPT_VALUE        (32U)

typedef struct HwiP_Ctrl_s {

    HwiP_FxnCallback isr[HwiP_MAX_INTERRUPTS];
    void *isrArgs[HwiP_MAX_INTERRUPTS];

    uint32_t spuriousIRQCount;
    uint32_t spuriousFIQCount;
} HwiP_Ctrl;

#ifdef INTR_PROF
typedef struct HwiP_Prof_Ctrl_s {

    uint32_t traceInterruptedISR[HwiP_MAX_INTERRUPTS];
    uint32_t traceInterruptedISRIndex;
    uint32_t readCounterStart;
    uint32_t readCounterStop;
    uint32_t profileIntr;
    uint64_t pmuCountVal;
    uint64_t pmuCalibration;

} HwiP_Prof_Ctrl;
extern HwiP_Prof_Ctrl gHwiCtrlProf;
#endif

extern HwiP_Ctrl gHwiCtrl;
extern HwiP_Config gHwiConfig;
/* APIs defined in HwiP_armv7r_asm.S */
uint32_t HwiP_disableFIQ(void);
void HwiP_enableFIQ(void);
void HwiP_enableVIC(void);

void HwiP_disableVIC(void);
void HwiP_fiq_handler(void);
void HwiP_irq_handler(void);
void HwiP_reserved_handler(void);
void HwiP_undefined_handler(void);
void HwiP_prefetch_abort_handler(void);
void HwiP_data_abort_handler(void);
void HwiP_irq_handler_c(void);

static inline void  HWI_SECTION HwiP_setAsFIQ(uint32_t intNum, uint32_t isFIQ)
{
    volatile uint32_t *addr;
    uint32_t bitPos;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_INT_MAP(intNum));
    bitPos = VIM_BIT_POS(intNum);

    if(isFIQ != 0U)
    {
        *addr |= ((uint32_t)0x1u << bitPos);
    }
    else
    {
        *addr &= ~(0x1u << bitPos);
    }
}

static inline uint32_t  HWI_SECTION HwiP_isPulse(uint32_t intNum)
{
    volatile uint32_t *addr;
    uint32_t bitPos;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_INT_TYPE(intNum));
    bitPos = VIM_BIT_POS(intNum);

    return ((*addr >> bitPos) & 0x1u );
}


static inline void  HWI_SECTION HwiP_setAsPulse(uint32_t intNum, uint32_t isPulse)
{
    volatile uint32_t *addr;
    uint32_t bitPos;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_INT_TYPE(intNum));
    bitPos = VIM_BIT_POS(intNum);

    if(isPulse != 0U)
    {
        *addr |= ((uint32_t)0x1u << bitPos);
    }
    else
    {
        *addr &= ~(0x1u << bitPos);
    }
}

static inline void  HWI_SECTION HwiP_setPri(uint32_t intNum, uint32_t priority)
{
    volatile uint32_t *addr;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_INT_PRI(intNum));

    *addr = (priority & 0xFu);
}

static inline void HWI_SECTION HwiP_setVecAddr(uint32_t intNum, uintptr_t vecAddr)
{
    volatile uint32_t *addr;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_INT_VEC(intNum));

    *addr = ((uint32_t)vecAddr & 0xFFFFFFFCU);
}

static inline uint32_t HWI_SECTION HwiP_getIRQVecAddr(void)
{
    volatile uint32_t *addr;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_IRQVEC);

    return *addr;
}

static inline uint32_t HWI_SECTION HwiP_getFIQVecAddr(void)
{
    volatile uint32_t *addr;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_FIQVEC);

    return *addr;
}

static inline int32_t HWI_SECTION HwiP_getIRQ(uint32_t *intNum)
{
    volatile uint32_t *addr;
    int32_t status = SystemP_FAILURE;
    uint32_t value;

    *intNum = 0;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_ACTIRQ);
    value = *addr;

    if((value & 0x80000000U) != 0U)
    {
        *intNum = (value & (HwiP_MAX_INTERRUPTS-1U));
        status = SystemP_SUCCESS;
    }
    return status;
}

static inline int32_t HWI_SECTION HwiP_getFIQ(uint32_t *intNum)
{
    volatile uint32_t *addr;
    int32_t status = SystemP_FAILURE;
    uint32_t value;

    *intNum = 0;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_ACTFIQ);
    value = *addr;

    if((value & 0x80000000U) != 0U)
    {
        *intNum = (value & 0x3FFU);
        status = SystemP_SUCCESS;
    }
    return status;
}

static inline void HWI_SECTION HwiP_ackIRQ(uint32_t intNum)
{
    volatile uint32_t *addr;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_IRQVEC);
    *addr= intNum;
}

static inline void HWI_SECTION HwiP_ackFIQ(uint32_t intNum)
{
    volatile uint32_t *addr;

    addr = (volatile uint32_t *)(gHwiConfig.intcBaseAddr + VIM_FIQVEC);
    *addr= intNum;
}


#define ISR_CALL_LEVEL_NONFLOAT_NONREENTRANT(fn, arg, intNum, vim_sts_addr, vim_sts_clr_mask, vim_addr)                 \
    __asm__ volatile(                  \
    "   SUB     lr, lr, #4                  \n"    \
    "   PUSH    {lr}                        \n"    \
    "   MRS     lr, SPSR                    \n"    \
    "   PUSH    {lr}                        \n"    \
    "   PUSH    {r0-r4, r12}                \n"    \
    "   LDR     r0, ="#arg"                 \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#fn"                  \n"    \
    "   BLX     r1                          \n"    \
    "   LDR     r0, ="#vim_sts_addr"        \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#vim_sts_clr_mask"    \n"    \
    "   LDR     r1, [r1]                    \n"    \
    "   STR     r1, [r0]                    \n"    \
    "   LDR     r0, ="#intNum"              \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#vim_addr"            \n"    \
    "   LDR     r1, [r1]                    \n"    \
    "   STR     r0, [r1, 0x18]              \n"    \
    "   POP     {r0-r4, r12}                \n"    \
    "   POP     {LR}                        \n"    \
    "   MSR     SPSR_cxsf, LR               \n"    \
    "   POP     {LR}                        \n"    \
    "   MOVS    PC, LR                      \n"    \
    )


#define ISR_CALL_PULSE_NONFLOAT_NONREENTRANT(fn, arg, intNum, vim_sts_addr, vim_sts_clr_mask, vim_addr)                  \
    __asm__ volatile(                  \
    "   SUB     lr, lr, #4                  \n"    \
    "   PUSH    {lr}                        \n"    \
    "   MRS     lr, SPSR                    \n"    \
    "   PUSH    {lr}                        \n"    \
    "   PUSH    {r0-r4, r12}                \n"    \
    "   LDR     r0, ="#vim_sts_addr"        \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#vim_sts_clr_mask"    \n"    \
    "   LDR     r1, [r1]                    \n"    \
    "   STR     r1, [r0]                    \n"    \
    "   LDR     r0, ="#arg"                 \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#fn"                  \n"    \
    "   BLX     r1                          \n"    \
    "   LDR     r0, ="#intNum"              \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#vim_addr"            \n"    \
    "   LDR     r1, [r1]                    \n"    \
    "   STR     r0, [r1, 0x18]              \n"    \
    "   POP     {r0-r4, r12}                \n"    \
    "   POP     {LR}                        \n"    \
    "   MSR     SPSR_cxsf, LR               \n"    \
    "   POP     {LR}                        \n"    \
    "   MOVS    PC, LR                      \n"    \
    )


#define ISR_CALL_LEVEL_FLOAT_NONREENTRANT(fn, arg, intNum, vim_sts_addr, vim_sts_clr_mask, vim_addr)                  \
    __asm__ volatile(                  \
    "   SUB     lr, lr, #4                  \n"    \
    "   PUSH    {lr}                        \n"    \
    "   MRS     lr, SPSR                    \n"    \
    "   PUSH    {lr}                        \n"    \
    "   PUSH    {r0-r4, r12}                \n"    \
    "   FMRX    R0, FPSCR                   \n"    \
    "   VPUSH   {D0-D15}                    \n"    \
    "   PUSH    {R0}                        \n"    \
    "   LDR     r0, ="#arg"                 \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#fn"                  \n"    \
    "   BLX     r1                          \n"    \
    "   LDR     r0, ="#vim_sts_addr"        \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#vim_sts_clr_mask"    \n"    \
    "   LDR     r1, [r1]                    \n"    \
    "   STR     r1, [r0]                    \n"    \
    "   LDR     r0, ="#intNum"              \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#vim_addr"            \n"    \
    "   LDR     r1, [r1]                    \n"    \
    "   STR     r0, [r1, 0x18]              \n"    \
    "   POP     {R0}                        \n"    \
    "   VPOP    {D0-D15}                    \n"    \
    "   VMSR    FPSCR, R0                   \n"    \
    "   POP     {r0-r4, r12}                \n"    \
    "   POP     {LR}                        \n"    \
    "   MSR     SPSR_cxsf, LR               \n"    \
    "   POP     {LR}                        \n"    \
    "   MOVS    PC, LR                      \n"    \
    )


#define ISR_CALL_PULSE_FLOAT_NONREENTRANT(fn, arg, intNum, vim_sts_addr, vim_sts_clr_mask, vim_addr)                  \
    __asm__ volatile(                  \
    "   SUB     lr, lr, #4                  \n"    \
    "   PUSH    {lr}                        \n"    \
    "   MRS     lr, SPSR                    \n"    \
    "   PUSH    {lr}                        \n"    \
    "   PUSH    {r0-r4, r12}                \n"    \
    "   FMRX    R0, FPSCR                   \n"    \
    "   VPUSH   {D0-D15}                    \n"    \
    "   PUSH    {R0}                        \n"    \
    "   LDR     r0, ="#vim_sts_addr"        \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#vim_sts_clr_mask"    \n"    \
    "   LDR     r1, [r1]                    \n"    \
    "   STR     r1, [r0]                    \n"    \
    "   LDR     r0, ="#arg"                 \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#fn"                  \n"    \
    "   BLX     r1                          \n"    \
    "   LDR     r0, ="#intNum"              \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#vim_addr"            \n"    \
    "   LDR     r1, [r1]                    \n"    \
    "   STR     r0, [r1, 0x18]              \n"    \
    "   POP     {R0}                        \n"    \
    "   VPOP    {D0-D15}                    \n"    \
    "   VMSR    FPSCR, R0                   \n"    \
    "   POP     {r0-r4, r12}                \n"    \
    "   POP     {LR}                        \n"    \
    "   MSR     SPSR_cxsf, LR               \n"    \
    "   POP     {LR}                        \n"    \
    "   MOVS    PC, LR                      \n"    \
    )


#define ISR_CALL_LEVEL_NONFLOAT_REENTRANT(fn, arg, intNum, vim_sts_addr, vim_sts_clr_mask, vim_addr)                  \
    __asm__ volatile(                  \
    "   SUB     lr, lr, #4                  \n"    \
    "   PUSH    {lr}                        \n"    \
    "   MRS     lr, SPSR                    \n"    \
    "   PUSH    {lr}                        \n"    \
    "   CPS     #0x13                       \n"    \
    "   PUSH    {r0-r4, r12}                \n"    \
    "   AND     r2, sp, #4                  \n"    \
    "   SUB     sp, sp, r2                  \n"    \
    "   PUSH    {r2, lr}                    \n"    \
    "   CPSIE   i                           \n"    \
    "   LDR     r0, ="#arg"                 \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#fn"                  \n"    \
    "   BLX     r1                          \n"    \
    "   CPSID   i                           \n"    \
    "   LDR     r0, ="#vim_sts_addr"        \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#vim_sts_clr_mask"    \n"    \
    "   LDR     r1, [r1]                    \n"    \
    "   STR     r1, [r0]                    \n"    \
    "   LDR     r0, ="#intNum"              \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#vim_addr"            \n"    \
    "   LDR     r1, [r1]                    \n"    \
    "   STR     r0, [r1, 0x18]              \n"    \
    "   POP     {r2, lr}                    \n"    \
    "   ADD     sp, sp, r2                  \n"    \
    "   CPSID   i                           \n"    \
    "   DSB                                 \n"    \
    "   ISB                                 \n"    \
    "   POP     {r0-r4, r12}                \n"    \
    "   CPS     #0x12                       \n"    \
    "   POP     {LR}                        \n"    \
    "   MSR     SPSR_cxsf, LR               \n"    \
    "   POP     {LR}                        \n"    \
    "   MOVS    PC, LR                      \n"    \
    )


#define ISR_CALL_PULSE_NONFLOAT_REENTRANT(fn, arg, intNum, vim_sts_addr, vim_sts_clr_mask, vim_addr)                  \
    __asm__ volatile(                  \
    "   SUB     lr, lr, #4                  \n"    \
    "   PUSH    {lr}                        \n"    \
    "   MRS     lr, SPSR                    \n"    \
    "   PUSH    {lr}                        \n"    \
    "   CPS     #0x13                       \n"    \
    "   PUSH    {r0-r4, r12}                \n"    \
    "   AND     r2, sp, #4                  \n"    \
    "   SUB     sp, sp, r2                  \n"    \
    "   PUSH    {r2, lr}                    \n"    \
    "   LDR     r0, ="#vim_sts_addr"        \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#vim_sts_clr_mask"    \n"    \
    "   LDR     r1, [r1]                    \n"    \
    "   STR     r1, [r0]                    \n"    \
    "   CPSIE   i                           \n"    \
    "   LDR     r0, ="#arg"                 \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#fn"                  \n"    \
    "   BLX     r1                          \n"    \
    "   CPSID   i                           \n"    \
    "   LDR     r0, ="#intNum"              \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#vim_addr"            \n"    \
    "   LDR     r1, [r1]                    \n"    \
    "   STR     r0, [r1, 0x18]              \n"    \
    "   POP     {r2, lr}                    \n"    \
    "   ADD     sp, sp, r2                  \n"    \
    "   CPSID   i                           \n"    \
    "   DSB                                 \n"    \
    "   ISB                                 \n"    \
    "   POP     {r0-r4, r12}                \n"    \
    "   CPS     #0x12                       \n"    \
    "   POP     {LR}                        \n"    \
    "   MSR     SPSR_cxsf, LR               \n"    \
    "   POP     {LR}                        \n"    \
    "   MOVS    PC, LR                      \n"    \
    )


#define ISR_CALL_LEVEL_FLOAT_REENTRANT(fn, arg, intNum, vim_sts_addr, vim_sts_clr_mask, vim_addr)                  \
    __asm__ volatile(                  \
    "   SUB     lr, lr, #4                  \n"    \
    "   PUSH    {lr}                        \n"    \
    "   MRS     lr, SPSR                    \n"    \
    "   PUSH    {lr}                        \n"    \
    "   CPS     #0x13                       \n"    \
    "   PUSH    {r0-r4, r12}                \n"    \
    "   FMRX    R0, FPSCR                   \n"    \
    "   VPUSH   {D0-D15}                    \n"    \
    "   PUSH    {R0}                        \n"    \
    "   AND     r2, sp, #4                  \n"    \
    "   SUB     sp, sp, r2                  \n"    \
    "   PUSH    {r2, lr}                    \n"    \
    "   CPSIE   i                           \n"    \
    "   LDR     r0, ="#arg"                 \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#fn"                  \n"    \
    "   BLX     r1                          \n"    \
    "   CPSID   i                           \n"    \
    "   LDR     r0, ="#vim_sts_addr"        \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#vim_sts_clr_mask"    \n"    \
    "   LDR     r1, [r1]                    \n"    \
    "   STR     r1, [r0]                    \n"    \
    "   LDR     r0, ="#intNum"              \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#vim_addr"            \n"    \
    "   LDR     r1, [r1]                    \n"    \
    "   STR     r0, [r1, 0x18]              \n"    \
    "   POP     {r2, lr}                    \n"    \
    "   ADD     sp, sp, r2                  \n"    \
    "   CPSID   i                           \n"    \
    "   DSB                                 \n"    \
    "   ISB                                 \n"    \
    "   POP     {R0}                        \n"    \
    "   VPOP    {D0-D15}                    \n"    \
    "   VMSR    FPSCR, R0                   \n"    \
    "   POP     {r0-r4, r12}                \n"    \
    "   CPS     #0x12                       \n"    \
    "   POP     {LR}                        \n"    \
    "   MSR     SPSR_cxsf, LR               \n"    \
    "   POP     {LR}                        \n"    \
    "   MOVS    PC, LR                      \n"    \
    )


#define ISR_CALL_PULSE_FLOAT_REENTRANT(fn, arg, intNum, vim_sts_addr, vim_sts_clr_mask, vim_addr)                  \
    __asm__ volatile(                  \
    "   SUB     lr, lr, #4                  \n"    \
    "   PUSH    {lr}                        \n"    \
    "   MRS     lr, SPSR                    \n"    \
    "   PUSH    {lr}                        \n"    \
    "   CPS     #0x13                       \n"    \
    "   PUSH    {r0-r4, r12}                \n"    \
    "   FMRX    R0, FPSCR                   \n"    \
    "   VPUSH   {D0-D15}                    \n"    \
    "   PUSH    {R0}                        \n"    \
    "   AND     r2, sp, #4                  \n"    \
    "   SUB     sp, sp, r2                  \n"    \
    "   PUSH    {r2, lr}                    \n"    \
    "   LDR     r0, ="#vim_sts_addr"        \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#vim_sts_clr_mask"    \n"    \
    "   LDR     r1, [r1]                    \n"    \
    "   STR     r1, [r0]                    \n"    \
    "   CPSIE   i                           \n"    \
    "   LDR     r0, ="#arg"                 \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#fn"                  \n"    \
    "   BLX     r1                          \n"    \
    "   CPSID   i                           \n"    \
    "   LDR     r0, ="#intNum"              \n"    \
    "   LDR     r0, [r0]                    \n"    \
    "   LDR     r1, ="#vim_addr"            \n"    \
    "   LDR     r1, [r1]                    \n"    \
    "   STR     r0, [r1, 0x18]              \n"    \
    "   POP     {r2, lr}                    \n"    \
    "   ADD     sp, sp, r2                  \n"    \
    "   CPSID   i                           \n"    \
    "   DSB                                 \n"    \
    "   ISB                                 \n"    \
    "   POP     {R0}                        \n"    \
    "   VPOP    {D0-D15}                    \n"    \
    "   VMSR    FPSCR, R0                   \n"    \
    "   POP     {r0-r4, r12}                \n"    \
    "   CPS     #0x12                       \n"    \
    "   POP     {LR}                        \n"    \
    "   MSR     SPSR_cxsf, LR               \n"    \
    "   POP     {LR}                        \n"    \
    "   MOVS    PC, LR                      \n"    \
    )


#ifdef __cplusplus
}
#endif

#endif /* HWIP_ARMV7R_VIM_H */
