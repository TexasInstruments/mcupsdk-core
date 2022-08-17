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

#include <kernel/nortos/dpl/c66/HwiP_c66.h>
#include <drivers/hw_include/csl_types.h>
#include <c6x.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct HwiP_Struct_s
{
    uint32_t intNum;
} HwiP_Struct;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static inline void HwiP_intcMapEventVector(HwiP_IntcRegsOvly pIntcRegs,
                                           uint32_t eventId,
                                           uint32_t vectId);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

HwiP_Ctrl       gHwiCtrl;
HwiP_IntcVect   gHwiIntcIntrTable;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void HwiP_init(void)
{
    uint32_t            i, key;
    uint32_t            vectId;
    HwiP_IntcRegsOvly   pIntcRegs;

    key = _disable_interrupts();

    /* initalize data structure */
    gHwiCtrl.pIntcRegs = (HwiP_IntcRegsOvly) HwiP_INTC_BASE_ADDR;
    for(i = 0U; i < HwiP_MAX_EVENTS; i++)
    {
        gHwiCtrl.isr[i] = NULL;
        gHwiCtrl.isrArgs[i] = NULL;
    }
    gHwiCtrl.nmiHandler = NULL;
    gHwiCtrl.nmiArgs = NULL;

    /* Assign dispatcher - gets set based on No-RTOS/FreeRTOS */
    HwiP_assignIntrHandlers();
    HwiP_intcIvpSet();

    /*
     * Disable and clear all ECM events
     */
    pIntcRegs = gHwiCtrl.pIntcRegs;
    for(i = 0U; i < HwiP_NUM_ECM; i++)
    {
        pIntcRegs->EVTMASK[i] = 0xFFFFFFFFU;
        pIntcRegs->EVTCLR[i]  = 0xFFFFFFFFU;
    }

    /*
     * Route ECM events to CPU interrupts and enable the CPU interrupts
     */
    for(i = 0U; i < HwiP_NUM_ECM; i++)
    {
        vectId = HwiP_VECTID_ECM_START + i;
        /* Note: Event ID 0-3 are reserved for ECM!! */
        HwiP_intcMapEventVector(pIntcRegs, i, vectId);
        HwiP_intcInterruptClear(vectId);
        (void) HwiP_intcInterruptEnable(vectId);
    }

    /*
     * Enable Global interrupts - NMIE and GIE to enable maskable interrupts
     */
    /* Set NMIE bit in the interrupt enable register (IER) */
    (void) HwiP_intcGlobalNmiEnable();
    /* Set global interrupt enable bit (GIE) bit in the control status register (CSR) */
    (void) HwiP_intcGlobalEnable(NULL);

    (void) _restore_interrupts(key);

    return;
}

int32_t HwiP_construct(HwiP_Object *handle, HwiP_Params *params)
{
    uint32_t            key;
    uint32_t            ecmId, eventId;
    HwiP_Struct        *obj;
    HwiP_IntcRegsOvly   pIntcRegs;

    obj = (HwiP_Struct *)handle;
    pIntcRegs = gHwiCtrl.pIntcRegs;
    DebugP_assertNoLog(sizeof(HwiP_Struct) <= sizeof(HwiP_Object));
    DebugP_assertNoLog(params->callback != NULL);
    DebugP_assertNoLog(params->intNum < HwiP_MAX_EVENTS);
    /* Check for reserved event used by ECM - 0, 2, 3 */
    DebugP_assertNoLog(params->intNum >= HwiP_INTC_NUM_RESV_INTR);

    key = _disable_interrupts();

    gHwiCtrl.isr[params->intNum] = params->callback;
    gHwiCtrl.isrArgs[params->intNum] = params->args;
    obj->intNum = params->intNum;

        /* TODO: Handle direct interrupts. */
        /* Enable the event through ECM */
        ecmId = params->intNum >> 5U;
        eventId = params->intNum & 0x1FU;
        pIntcRegs->EVTMASK[ecmId] &= ~((uint32_t) 1U << eventId);

   (void) _restore_interrupts(key);

    return SystemP_SUCCESS;
}

void HwiP_destruct(HwiP_Object *handle)
{
    uint32_t            key;
    uint32_t            ecmId, eventId;
    HwiP_Struct        *obj;
    HwiP_IntcRegsOvly   pIntcRegs;

    obj = (HwiP_Struct *) handle;
    pIntcRegs = gHwiCtrl.pIntcRegs;
    DebugP_assertNoLog(obj->intNum < HwiP_MAX_EVENTS);
    /* Check for reserved event used by ECM - 0 to 3 */
    DebugP_assertNoLog(obj->intNum >= HwiP_INTC_NUM_RESV_INTR);

    key = _disable_interrupts();

        /* TODO: Handle direct interrupts */
        /* Disable and clear event through ECM */
        ecmId = obj->intNum >> 5U;
        eventId = obj->intNum & 0x1FU;
        pIntcRegs->EVTMASK[ecmId] |= ((uint32_t) 1U << eventId);
        pIntcRegs->EVTCLR[ecmId]   = ((uint32_t) 1U << eventId);

    /* clear interrupt data structure */
    gHwiCtrl.isr[obj->intNum] = NULL;
    gHwiCtrl.isrArgs[obj->intNum] = NULL;

    (void) _restore_interrupts(key);

    return;
}

int32_t HwiP_registerNmiHandler(HwiP_FxnCallback nmiHandler, void *args)
{
    int32_t retVal = SystemP_SUCCESS;
    if (gHwiCtrl.nmiHandler != NULL)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        gHwiCtrl.nmiHandler = nmiHandler;
        gHwiCtrl.nmiArgs = args;
    }
    return retVal;
}

int32_t HwiP_unregisterNmiHandler(void)
{
    int32_t retVal = SystemP_SUCCESS;
    if (gHwiCtrl.nmiHandler == NULL)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        gHwiCtrl.nmiHandler = NULL;
        gHwiCtrl.nmiArgs = NULL;
    }
    return retVal;
}

int32_t HwiP_setArgs(HwiP_Object *handle, void *args)
{
    HwiP_Struct *obj = (HwiP_Struct *)handle;

    DebugP_assertNoLog(obj->intNum < HwiP_MAX_EVENTS);

    gHwiCtrl.isrArgs[obj->intNum] = args;

    return SystemP_SUCCESS;
}

void HwiP_enableInt(uint32_t intNum)
{
    uint32_t            key;
    uint32_t            ecmId, eventId;
    HwiP_IntcRegsOvly   pIntcRegs;

    pIntcRegs = gHwiCtrl.pIntcRegs;
    DebugP_assertNoLog(intNum < HwiP_MAX_EVENTS);
    /* Check for reserved event used by ECM - 0 to 3 */
    DebugP_assertNoLog(intNum >= HwiP_INTC_NUM_RESV_INTR);

    //TODO: Handle direct interrupts
    //if(1)
    {
        /* Enable event through ECM */
        ecmId = intNum >> 5U;
        eventId = intNum & 0x1FU;
        key = _disable_interrupts();
        pIntcRegs->EVTMASK[ecmId] &= ~((uint32_t) 1U << eventId);
        (void) _restore_interrupts(key);
    }
    // else block here

    return;
}

uint32_t HwiP_disableInt(uint32_t intNum)
{
    uint32_t            key;
    uint32_t            ecmId, eventId;
    HwiP_IntcRegsOvly   pIntcRegs;
    uint32_t            isEnable = 0;

    pIntcRegs = gHwiCtrl.pIntcRegs;
    DebugP_assertNoLog(intNum < HwiP_MAX_EVENTS);
    /* Check for reserved event used by ECM - 0 to 3 */
    DebugP_assertNoLog(intNum >= HwiP_INTC_NUM_RESV_INTR);

    //TODO: Handle direct interrupts
    //if(1)
    {
        /* Enable event through ECM if already enabled */
        ecmId = intNum >> 5U;
        eventId = intNum & 0x1FU;
        key = _disable_interrupts();
        if ((pIntcRegs->EVTMASK[ecmId] & ((uint32_t) 1U << eventId)) == 0U)
        {
            isEnable = 1U;
            pIntcRegs->EVTMASK[ecmId] |= ((uint32_t) 1U << eventId);
        }
        (void) _restore_interrupts(key);
    }
    // else block here

    return (isEnable);
}

void HwiP_restoreInt(uint32_t intNum, uint32_t oldIntState)
{
    if(oldIntState!=0U)
    {
        HwiP_enableInt(intNum);
    }
    else
    {
       (void) HwiP_disableInt(intNum);
    }

    return;
}

void HwiP_clearInt(uint32_t intNum)
{
    uint32_t            ecmId, eventId;
    HwiP_IntcRegsOvly   pIntcRegs;

    pIntcRegs = gHwiCtrl.pIntcRegs;
    DebugP_assertNoLog(intNum < HwiP_MAX_EVENTS);
    /* Check for reserved event used by ECM - 0 to 3 */
    DebugP_assertNoLog(intNum >= HwiP_INTC_NUM_RESV_INTR);

    //TODO: Handle direct interrupts
    //if(1)
    {
        /* Clear event through ECM */
        ecmId = intNum >> 5U;
        eventId = intNum & 0x1FU;
        pIntcRegs->EVTCLR[ecmId] = ((uint32_t) 1U << eventId);
    }
    // else block here

    return;
}

void HwiP_setPri(uint32_t intNum, uint32_t priority)
{
    /* C66x doesn't support programmable priority - it is implicit based on vect ID */
    return;
}

void HwiP_post(uint32_t intNum)
{
    uint32_t            ecmId, eventId;
    HwiP_IntcRegsOvly   pIntcRegs;

    pIntcRegs = gHwiCtrl.pIntcRegs;
    DebugP_assertNoLog(intNum < HwiP_MAX_EVENTS);
    /* Check for reserved event used by ECM - 0 to 3 */
    DebugP_assertNoLog(intNum >= HwiP_INTC_NUM_RESV_INTR);

    //TODO: Handle direct interrupts
    //if(1)
    {
        /* Disable and clear event through ECM */
        ecmId = intNum >> 5U;
        eventId = intNum & 0x1FU;
        pIntcRegs->EVTSET[ecmId] = ((uint32_t) 1U << eventId);
    }
    // else block here

    return;
}

uintptr_t HwiP_disable(void)
{
    return (uintptr_t)(_disable_interrupts());
}

void HwiP_enable(void)
{
	(void) _enable_interrupts();
    return;
}

void HwiP_restore(uintptr_t oldIntState)
{
    (void)_restore_interrupts(oldIntState);
    return;
}

void HwiP_Params_init(HwiP_Params *params)
{
    params->intNum = 0;
    params->callback = NULL;
    params->args = NULL;
    params->eventId = 0;    /* NOT USED */
    params->priority = 0;   /* NOT USED */
    params->isFIQ = 0;      /* NOT USED */
    params->isPulse = 0;    /* NOT USED */

    return;
}

void HwiP_intcEcmDispatcher(uint32_t ecmId)
{
    uint32_t            isrIdx, isrStartIdx;
    HwiP_IntcRegsOvly   pIntcRegs = gHwiCtrl.pIntcRegs;
    uint32_t            i, evtMask;
    volatile uint32_t   mevtFlag;
	uint32_t flag = 0U;
    uint32_t loop = 1U;
	
    isrStartIdx = HwiP_EVENTS_PER_ECM * ecmId;
    while(loop != 0U)
    {
        /* Get current pending ECM interrupts */
        mevtFlag = pIntcRegs->MEVTFLAG[ecmId];
        if(mevtFlag == 0U)
        {
            /* No pending interrupts */
            break;
        }
        /* Clear current pending ECM interrupts */
        pIntcRegs->EVTCLR[ecmId] = mevtFlag;

        /* Check and process pending interrupts */
        for(i = 0U; i < HwiP_EVENTS_PER_ECM; i++)
        {
            evtMask = ((uint32_t) 1U << i);
            flag = (mevtFlag & evtMask);
            isrIdx = isrStartIdx + i;
            if((gHwiCtrl.isr[isrIdx] != NULL) &&
               (flag != 0U))
            {
                /* Call user callback */
                gHwiCtrl.isr[isrIdx](gHwiCtrl.isrArgs[isrIdx]);
            }
            mevtFlag &= ~evtMask;
            if(mevtFlag == 0U)
            {
                break;
            }
        }
    }

    return;
}

static inline void HwiP_intcMapEventVector(HwiP_IntcRegsOvly pIntcRegs,
                                           uint32_t eventId,
                                           uint32_t vectId)
{
    uint32_t    bitLow;

    if(vectId < 8U)
    {
        bitLow = (vectId - 4U) * 8U;
        CSL_FINSR(pIntcRegs->INTMUX1, bitLow + 6U, bitLow, eventId);
    }
    else if(vectId < 12U)
    {
        bitLow = (vectId - 8U) * 8U;
        CSL_FINSR(pIntcRegs->INTMUX2, bitLow + 6U, bitLow, eventId);
    }
    else
    {
        bitLow = (vectId - 12U) * 8U;
        CSL_FINSR(pIntcRegs->INTMUX3, bitLow + 6U, bitLow, eventId);
    }

    return;
}

