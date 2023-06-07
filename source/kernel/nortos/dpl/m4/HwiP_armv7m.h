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

#ifndef HWIP_ARMV7M_H
#define HWIP_ARMV7M_H

#ifdef __cplusplus
extern "C"
{
#endif


#include <kernel/dpl/HwiP.h>

#define HWI_SECTION __attribute__((section(".text.hwi")))

#define HwiP_MAX_INTERRUPTS     (16u+64u)
#define HwiP_MAX_PRIORITY       (8u)

#define HwiP_NVIC_PRI_BITS      (3u)
#define HwiP_NVIC_PRI_SHIFT     (8u-(HwiP_NVIC_PRI_BITS))
/* this is the value to use to disable all interrupts above this priority */
#define HwiP_NVIC_PRI_DISABLE   (((HwiP_MAX_PRIORITY)-1U) << (HwiP_NVIC_PRI_SHIFT))

#define NVIC_BASE       (0xE000E000u)
#define NVIC_ISER(x)    (volatile uint32_t *)((NVIC_BASE)+0x100u+(4u*(x)))
#define NVIC_ICER(x)    (volatile uint32_t *)((NVIC_BASE)+0x180u+(4u*(x)))
#define NVIC_ISPR(x)    (volatile uint32_t *)((NVIC_BASE)+0x200u+4u*(x))
#define NVIC_ICPR(x)    (volatile uint32_t *)((NVIC_BASE)+0x280u+(4u*(x)))
#define NVIC_IPRI(x)    (volatile uint32_t *)((NVIC_BASE)+0x400u+(4u*(x)))

#define SHPR(x)         (volatile uint32_t *)((0xE000ED18U)+(4u*(x)))

#define SYSTICK_CSR     (volatile uint32_t *)(0xE000E010u)
#define STIR            (volatile uint32_t *)(0xE000EF00u)

#define VTOR            (volatile uint32_t *)(0xE000ED08u)

#define ICSR            (volatile uint32_t *)(0xE000ED04u)

typedef struct HwiP_Ctrl_s {

    HwiP_FxnCallback isr[HwiP_MAX_INTERRUPTS];
    void *isrArgs[HwiP_MAX_INTERRUPTS];

} HwiP_Ctrl;

extern HwiP_Ctrl gHwiCtrl;
extern uint32_t gHwiP_vectorTable[HwiP_MAX_INTERRUPTS];

/**
 * \brief Set Priority to Interrupt
 *
 * \param intNum [in]   Interrupt number
 * \param priority [in] Priority
 */
void HwiP_setPri(uint32_t intNum, uint32_t priority);

#ifdef __cplusplus
}
#endif

#endif /* HWIP_ARMV7M_H */
