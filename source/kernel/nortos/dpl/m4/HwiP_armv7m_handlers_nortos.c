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
    volatile uint32_t loop = 1;
    while(loop)
        ;
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