/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <FreeRTOS.h>
#include <task.h>
#include "port_hwi.h"
#include "portmacro.h"
#include <kernel/nortos/dpl/c66/HwiP_c66.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Size of ISR Stack in c66x */
#define configHWI_TASK_STACK_DEPTH              (4096U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

extern void Hwi_switchAndDispatch__I(int32_t intNum);
extern void vPortYieldAsyncFromISR(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* --> __TI_STATIC_BASE */
extern void *__TI_STATIC_BASE;
extern void *_vectors;

extern uint32_t ulPortInterruptNesting;
extern uint32_t ulPortYieldRequired;

Hwi_Module_State__ Hwi_Module__state__V = {
    (uint16_t) 0x4003,                          /* ierMask */
    (int32_t) 0x0,                              /* intNum */
    ((char*) NULL),                             /* taskSP */
    ((char*) NULL),                             /* isrStack */
    ((void *) ((void*)&_vectors)),              /* vectorTableBase */
    ((void *) ((void*)&__TI_STATIC_BASE)),      /* bss */
    (int32_t) 0x0,                              /* scw */
};

StackType_t  uxHwiStack[configHWI_TASK_STACK_DEPTH];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void Hwi_dispatchC__I(int32_t intNum)
{
    ulPortInterruptNesting++;
    Hwi_switchAndDispatch__I(intNum);

    ulPortInterruptNesting--;
    if(ulPortInterruptNesting == 0)
    {
        if(ulPortYieldRequired != pdFALSE)
        {
            ulPortYieldRequired = pdFALSE;
            vPortYieldAsyncFromISR();
        }
    }
}

void Hwi_dispatchCore__I(int32_t intNum)
{
    gHwiIntcIntrTable.isr[intNum]();
}

/*
 *  ======== Hwi_getIsrStackAddress ========
 */
char *Hwi_getIsrStackAddress()
{
    uint32_t            isrStack;

    isrStack = (uint32_t) uxHwiStack;
    isrStack += (uint32_t) sizeof(uxHwiStack);
    isrStack &= ~0xF;       /* align to long word */

    return ((char *) isrStack);
}

/*
 *  ======== Hwi_Module_startup ========
 */
void Hwi_Module_startup(void)
{
    /*
     * Initialize the pointer to the isrStack. These symbols are part of the
     * Hwi_module (instead of floating) in order to support ROM.
     * Leave room for one 32-bit value pushed by xdc_runtime_Startup_reset()
     * (for cases where intentionally reset as resume from power down),
     * and maintain double word alignment.
     */
    Hwi_Module__state__V.isrStack = Hwi_getIsrStackAddress() - 8;
    Hwi_Module__state__V.taskSP = (char *)-1;/* signal that we're executing on the */
}

/*
 *  ======== switchFromBootStack ========
 *  Indicate that we are leaving the boot stack and
 *  are about to switch to a task stack.
 */
void Hwi_switchFromBootStack(void)
{
    Hwi_Module__state__V.taskSP = 0;
}

uint32_t HwiP_inISR(void)
{
    uint32_t stat = 0U;

    if((Hwi_Module__state__V.taskSP != (char *)0) &&
       (Hwi_Module__state__V.taskSP != (char *)-1))
    {
        stat =  1U;
    }

    return stat;
}
