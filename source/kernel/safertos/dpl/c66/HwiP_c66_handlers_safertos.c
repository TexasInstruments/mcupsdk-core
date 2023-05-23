/*
 *  Copyright (C) 2021-23 Texas Instruments Incorporated
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

#include <kernel/dpl/HwiP.h>
#include "SafeRTOS_API.h"
#include <drivers/hw_include/csl_types.h>
#include <kernel/nortos/dpl/c66/HwiP_c66.h>
#include <c6x.h>
#include <kernel/safertos/dpl/c66/HwiP_safertos.h>
#include "portmacro.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

HwiP_appInterruptHandlerHookFxnPtr gHwiIntHook = NULL;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void HwiP_intcDispatcher(void);
static void HwiP_intcEcm0Dispatcher(void);
static void HwiP_intcEcm1Dispatcher(void);
static void HwiP_intcEcm2Dispatcher(void);
static void HwiP_intcEcm3Dispatcher(void);
static void HwiP_intcReservedDispatcher(void);
interrupt void HwiP_intcNmiDispatcher(void);
extern void vPortSetInterruptVectors(void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void HwiP_assignIntrHandlers(void)
{
    uint32_t            i;
    HwiP_IntcIsr        ecmIsr[] =
    {
        &HwiP_intcEcm0Dispatcher,
        &HwiP_intcEcm1Dispatcher,
        &HwiP_intcEcm2Dispatcher,
        &HwiP_intcEcm3Dispatcher
    };

    /* Set all vector ISR to default dispatcher */
    for(i = 0U; i < HwiP_INTC_NUM_INTR; i++)
    {
        gHwiIntcIntrTable.isr[i]  = &HwiP_intcDispatcher;
    }
    /* Override for NMI and reserved */
    gHwiIntcIntrTable.isr[0] = &HwiP_intcReservedDispatcher;
    gHwiIntcIntrTable.isr[1] = &HwiP_intcNmiDispatcher;
    gHwiIntcIntrTable.isr[2] = &HwiP_intcReservedDispatcher;
    gHwiIntcIntrTable.isr[3] = &HwiP_intcReservedDispatcher;
    /* Override for ECM */
    for(i = 0U; i < HwiP_NUM_ECM; i++)
    {
        gHwiIntcIntrTable.isr[HwiP_VECTID_ECM_START + i] = ecmIsr[i];
    }

    return;
}

static void HwiP_intcDispatcher(void)
{
    //TODO
}

static void HwiP_intcEcm0Dispatcher(void)
{
    HwiP_intcEcmDispatcher(0U);
}

static void HwiP_intcEcm1Dispatcher(void)
{
    HwiP_intcEcmDispatcher(1U);
}

static void HwiP_intcEcm2Dispatcher(void)
{
    HwiP_intcEcmDispatcher(2U);
}

static void HwiP_intcEcm3Dispatcher(void)
{
    HwiP_intcEcmDispatcher(3U);
}

static void HwiP_intcReservedDispatcher(void)
{
    while(1);
}

uint32_t HwiP_inISR(void)
{
    uint32_t stat = 0U, tsr;

    tsr = TSR;
    if(tsr & 0x200U)    /* TSR.INT - bit 9 */
    {
        stat = 1U;
    }

    return (stat);
}

void HwiP_intcIvpSet(void)
{
    vPortSetInterruptVectors();
}

/* Register interrupt handler callback. */
void HwiP_registerInterruptHandlerHook(HwiP_appInterruptHandlerHookFxnPtr hookFxnPtr)
{
    gHwiIntHook = hookFxnPtr;
}

/* Dispatch handler for TI MCU+ style interrupts. */
void vApplicationInterruptHandlerHook( portUInt32Type ulInterruptVectorNum )
{
    if( ulInterruptVectorNum < HwiP_INTC_NUM_RESV_INTR )
    {
        DebugP_assert(FALSE); /* NMI or reserved */
    }
    else
    {
        if( ulInterruptVectorNum < HwiP_VECTID_ECM_START )
        {
            gHwiIntcIntrTable.isr[ ulInterruptVectorNum ]();
        }
        else
        {
            HwiP_intcEcmDispatcher( ulInterruptVectorNum - HwiP_VECTID_ECM_START );
        }

        if(gHwiIntHook != NULL)
        {
            gHwiIntHook(ulInterruptVectorNum);
        }
    }
}

int32_t HwiP_portRaisePrivilege(void)
{
    return (int32_t) portRAISE_PRIVILEGE();
}

void HwiP_portRestorePrivilege(int32_t state)
{
    portRESTORE_PRIVILEGE(state);
}
