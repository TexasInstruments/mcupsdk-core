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

/**
 *  \file udma_event.c
 *
 *  \brief File containing the UDMA driver event management functions.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/udma/udma_priv.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void Udma_eventIsrFxn(void *args);
static int32_t Udma_eventCheckParams(Udma_DrvHandleInt drvHandle,
                                     const Udma_EventPrms *eventPrms);
static int32_t Udma_eventCheckUnRegister(Udma_DrvHandleInt drvHandle,
                                         Udma_EventHandleInt eventHandle);
static int32_t Udma_eventAllocResource(Udma_DrvHandleInt drvHandle,
                                       Udma_EventHandleInt eventHandle);
static void Udma_eventFreeResource(Udma_DrvHandleInt drvHandle,
                                   Udma_EventHandleInt eventHandle);
static int32_t Udma_eventConfig(Udma_DrvHandleInt drvHandle,
                                Udma_EventHandleInt eventHandle);
static int32_t Udma_eventReset(Udma_DrvHandleInt drvHandle,
                               Udma_EventHandleInt eventHandle);
static void Udma_eventProgramSteering(Udma_DrvHandleInt drvHandle,
                                      Udma_EventHandleInt eventHandle);
static void Udma_eventResetSteering(Udma_DrvHandleInt drvHandle,
                                    Udma_EventHandleInt eventHandle);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Udma_eventRegister(Udma_DrvHandle drvHandle,
                           Udma_EventHandle eventHandle,
                           Udma_EventPrms *eventPrms)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            allocDone = (uint32_t) FALSE;
    Udma_DrvHandleInt   drvHandleInt;
    Udma_EventHandleInt eventHandleInt;

    /* Error check */
    if((NULL_PTR == drvHandle) || (NULL_PTR == eventHandle) || (NULL_PTR == eventPrms))
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandleInt = (Udma_DrvHandleInt) drvHandle;
        if(drvHandleInt->drvInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        retVal = Udma_eventCheckParams(drvHandleInt, eventPrms);
    }

    if(UDMA_SOK == retVal)
    {
        /* Copy and init parameters */
        eventHandleInt = (Udma_EventHandleInt) eventHandle;
        (void) memcpy(
            &eventHandleInt->eventPrms, eventPrms, sizeof(eventHandleInt->eventPrms));
        eventHandleInt->drvHandle       = drvHandleInt;
        eventHandleInt->globalEvent     = UDMA_EVENT_INVALID;
        eventHandleInt->vintrNum        = UDMA_EVENT_INVALID;
        eventHandleInt->vintrBitNum     = UDMA_EVENT_INVALID;
        eventHandleInt->irIntrNum       = UDMA_INTR_INVALID;
        eventHandleInt->coreIntrNum     = UDMA_INTR_INVALID;
        eventHandleInt->nextEvent       = (Udma_EventHandleInt) NULL_PTR;
        eventHandleInt->prevEvent       = (Udma_EventHandleInt) NULL_PTR;
        eventHandleInt->hwiHandle       = NULL_PTR;
        eventHandleInt->vintrBitAllocFlag = 0U;
        eventHandleInt->pIaGeviRegs     = (volatile CSL_intaggr_imapRegs_gevi *) NULL_PTR;
        eventHandleInt->pIaVintrRegs    = (volatile CSL_intaggr_intrRegs_vint *) NULL_PTR;
    }

    if(UDMA_SOK == retVal)
    {
        if ((UDMA_INST_TYPE_NORMAL           != drvHandleInt->instType) &&
            (UDMA_EVENT_TYPE_TEARDOWN_PACKET == eventPrms->eventType))
        {
            /* In case of devices like AM64x, Teardown is not supported.
            Therefore no need to allocate resource and configure teardown event.

            eventHandle is already populated with drvHandle and eventPrms,
            becase during Unregistering this event,
            the instType in DrvHandle and evenType in eventPrms
            are required to bypass the eventReset
            (Since only evenHandle is passed to eventUnRegister) */
        }
        else
        {
            /* Alloc event resources */
            retVal = Udma_eventAllocResource(drvHandleInt, eventHandleInt);
            if(UDMA_SOK == retVal)
            {
                allocDone = (uint32_t) TRUE;
            }
            else
            {
                DebugP_logError("[UDMA] Event resource allocation failed!!\r\n");
            }

            if(UDMA_SOK == retVal)
            {
                /* Set init flag as events are allocated and event config expects
                * this flag to be set */
                eventHandleInt->eventInitDone = UDMA_INIT_DONE;

                /* Configure Event */
                retVal = Udma_eventConfig(drvHandleInt, eventHandleInt);
                if(UDMA_SOK == retVal)
                {
                    allocDone = (uint32_t) TRUE;
                }
                else
                {
                    DebugP_logError("[UDMA] Event config failed!!\r\n");
                }
            }

            if(UDMA_SOK != retVal)
            {
                /* Error. Free-up resource if allocated */
                if(((uint32_t) TRUE) == allocDone)
                {
                    Udma_eventFreeResource(drvHandleInt, eventHandleInt);
                    eventHandleInt->eventInitDone = UDMA_DEINIT_DONE;
                }
            }
            else
            {
                /* Copy the allocated resource info */
                DebugP_assert(eventHandleInt->pIaVintrRegs != NULL_PTR);
                eventPrms->intrStatusReg    = &eventHandleInt->pIaVintrRegs->STATUSM;
                eventPrms->intrClearReg     = &eventHandleInt->pIaVintrRegs->STATUS_CLEAR;
                if(eventHandleInt->vintrBitNum != UDMA_EVENT_INVALID)
                {
                    eventPrms->intrMask     = ((uint64_t)1U << eventHandleInt->vintrBitNum);
                }
                else
                {
                    /* No VINT bit for global master event */
                    eventPrms->intrMask     = 0U;
                }
                if(NULL_PTR == eventHandleInt->eventPrms.masterEventHandle)
                {
                    /* This is master handle - copy directly from here itself */
                    eventPrms->vintrNum     = eventHandleInt->vintrNum;
                    eventPrms->coreIntrNum  = eventHandleInt->coreIntrNum;
                }
                else
                {
                    /* Copy core number from master handle */
                    /* Copy from master handle */
                    eventPrms->vintrNum       =
                       ((Udma_EventHandleInt) (eventHandleInt->eventPrms.masterEventHandle))->vintrNum;
                    eventPrms->coreIntrNum    =
                        ((Udma_EventHandleInt) (eventHandleInt->eventPrms.masterEventHandle))->coreIntrNum;
                }
                /* Copy the same info to eventHandleInt->eventPrms*/
                eventHandleInt->eventPrms.intrStatusReg   = eventPrms->intrStatusReg;
                eventHandleInt->eventPrms.intrClearReg    = eventPrms->intrClearReg;
                eventHandleInt->eventPrms.intrMask        = eventPrms->intrMask;
                eventHandleInt->eventPrms.vintrNum        = eventPrms->vintrNum;
                eventHandleInt->eventPrms.coreIntrNum     = eventPrms->coreIntrNum;
            }
        }
    }
    return (retVal);
}

int32_t Udma_eventUnRegister(Udma_EventHandle eventHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandle;
    Udma_EventHandleInt eventHandleInt;

    /* Error check */
    if(NULL_PTR == eventHandle)
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        eventHandleInt = (Udma_EventHandleInt) eventHandle;
        drvHandle = eventHandleInt->drvHandle;
        if((NULL_PTR == drvHandle) || (drvHandle->drvInitDone != UDMA_INIT_DONE))
        {
            retVal = UDMA_EFAIL;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if ((UDMA_INST_TYPE_NORMAL           != drvHandle->instType) &&
            (UDMA_EVENT_TYPE_TEARDOWN_PACKET == eventHandleInt->eventPrms.eventType))
        {
            /* In case of devices like AM64x, Teardown is not supported.
            Therefore no need to unregister teardown event. */
        }
        else
        {
            retVal = Udma_eventCheckUnRegister(drvHandle, eventHandleInt);

            if(UDMA_SOK == retVal)
            {
                if(NULL_PTR != eventHandleInt->hwiHandle)
                {
                    /* Disable able core interrupt to avoid having insane
                    * state/variables when an interrupt occurs while processing
                    * event free */
                    DebugP_assert(eventHandleInt->coreIntrNum != UDMA_INTR_INVALID);
                    HwiP_disableInt(eventHandleInt->coreIntrNum);
                }
                /* Reset and Free-up event resources */
                retVal = Udma_eventReset(drvHandle, eventHandleInt);
                if(UDMA_SOK != retVal)
                {
                    DebugP_logError("[UDMA] Free Event resource failed!!!\r\n");
                }
                Udma_eventFreeResource(drvHandle, eventHandleInt);

                eventHandleInt->eventInitDone  = UDMA_DEINIT_DONE;
                eventHandleInt->pIaGeviRegs    = (volatile CSL_intaggr_imapRegs_gevi *) NULL_PTR;
                eventHandleInt->pIaVintrRegs   = (volatile CSL_intaggr_intrRegs_vint *) NULL_PTR;
                eventHandleInt->drvHandle      = (Udma_DrvHandleInt) NULL_PTR;
            }
        }
    }
    return (retVal);
}

uint32_t Udma_eventGetId(Udma_EventHandle eventHandle)
{
    uint32_t            evtNum = UDMA_EVENT_INVALID;
    Udma_DrvHandleInt   drvHandle;
    Udma_EventHandleInt eventHandleInt = (Udma_EventHandleInt) eventHandle;

    if((NULL_PTR != eventHandleInt) &&
       (UDMA_INIT_DONE == eventHandleInt->eventInitDone))
    {
        drvHandle = eventHandleInt->drvHandle;
        if(NULL_PTR != drvHandle)
        {
            evtNum = drvHandle->iaGemOffset + eventHandleInt->globalEvent;
        }
    }

    return (evtNum);
}

int32_t Udma_eventDisable(Udma_EventHandle eventHandle)
{
    int32_t             retVal = UDMA_EFAIL;
    Udma_DrvHandleInt   drvHandle;
    uint32_t            vintrBitNum;
    uint32_t            vintrNum;
    Udma_EventHandleInt eventHandleInt = (Udma_EventHandleInt) eventHandle;

    if((NULL_PTR != eventHandleInt) &&
       (UDMA_INIT_DONE == eventHandleInt->eventInitDone))
    {
        drvHandle = eventHandleInt->drvHandle;
        if(NULL_PTR != drvHandle)
        {
            /* In case of shared events "eventHandle->vintrNum" will be invalid,
             * since it relies on the master event.
             * Hence, use "eventHandle->eventPrms.vintrNum"
             * which will be populated with,
             * master events vintrNum for shared events and
             * its own vintrNum for exlcusive events. */
            vintrNum = eventHandleInt->eventPrms.vintrNum;
            vintrBitNum = vintrNum * UDMA_MAX_EVENTS_PER_VINTR;
            vintrBitNum += eventHandleInt->vintrBitNum;

            retVal = CSL_intaggrSetIntrEnable(
                         &drvHandle->iaRegs, vintrBitNum, (bool)false);
        }
    }

    return (retVal);
}

int32_t Udma_eventEnable(Udma_EventHandle eventHandle)
{
    int32_t             retVal = UDMA_EFAIL;
    Udma_DrvHandleInt   drvHandle;
    uint32_t            vintrBitNum;
    uint32_t            vintrNum;
    Udma_EventHandleInt eventHandleInt = (Udma_EventHandleInt) eventHandle;

    if((NULL_PTR != eventHandleInt) &&
       (UDMA_INIT_DONE == eventHandleInt->eventInitDone))
    {
        drvHandle = eventHandleInt->drvHandle;
        if(NULL_PTR != drvHandle)
        {
            /* In case of shared events "eventHandle->vintrNum" will be invalid,
             * since it relies on the master event.
             * Hence, use "eventHandle->eventPrms.vintrNum"
             * which will be populated with,
             * master events vintrNum for shared events and
             * its own vintrNum exlcusive events. */
            vintrNum = eventHandleInt->eventPrms.vintrNum;
            vintrBitNum = vintrNum * UDMA_MAX_EVENTS_PER_VINTR;
            vintrBitNum += eventHandleInt->vintrBitNum;

            retVal = CSL_intaggrSetIntrEnable(
                         &drvHandle->iaRegs, vintrBitNum, (bool)true);
        }
    }

    return (retVal);
}

Udma_EventHandle Udma_eventGetGlobalHandle(Udma_DrvHandle drvHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandleInt;
    Udma_EventHandle    eventHandle = (Udma_EventHandle) NULL_PTR;

    /* Error check */
    if(NULL_PTR == drvHandle)
    {
        retVal = UDMA_EBADARGS;
    }
    if(UDMA_SOK == retVal)
    {
        drvHandleInt = (Udma_DrvHandleInt) drvHandle;
        if(drvHandleInt->drvInitDone != UDMA_INIT_DONE)
        {
            retVal = UDMA_EFAIL;
        }
    }
    if(UDMA_SOK == retVal)
    {
        eventHandle = (Udma_EventHandle) drvHandleInt->globalEventHandle;
    }

    return (eventHandle);
}

void UdmaEventPrms_init(Udma_EventPrms *eventPrms)
{
    if(NULL_PTR != eventPrms)
    {
        eventPrms->eventType            = UDMA_EVENT_TYPE_DMA_COMPLETION;
        eventPrms->eventMode            = UDMA_EVENT_MODE_SHARED;
        eventPrms->chHandle             = (Udma_ChHandle) NULL_PTR;
        eventPrms->ringHandle           = (Udma_RingHandle) NULL_PTR;
        eventPrms->masterEventHandle    = (Udma_EventHandle) NULL_PTR;
        eventPrms->eventCb              = (Udma_EventCallback) NULL_PTR;
        eventPrms->intrPriority         = 1U;
        eventPrms->appData              = NULL_PTR;
        eventPrms->preferredCoreIntrNum = UDMA_CORE_INTR_ANY;
        eventPrms->intrStatusReg        = (volatile uint64_t *) NULL_PTR;
        eventPrms->intrClearReg         = (volatile uint64_t *) NULL_PTR;
        eventPrms->intrMask             = 0U;
        eventPrms->vintrNum             = UDMA_EVENT_INVALID;
        eventPrms->coreIntrNum          = UDMA_INTR_INVALID;
    }

    return;
}

static void Udma_eventIsrFxn(void *args)
{
    uint32_t            vintrBitNum;
    uint32_t            vintrNum;
    uint32_t            teardownStatus;
    Udma_EventHandleInt eventHandle = (Udma_EventHandleInt) args;
    Udma_DrvHandleInt   drvHandle;
    Udma_EventPrms     *eventPrms;
    Udma_RingHandleInt  ringHandle;

    teardownStatus = UDMA_EVENT_CH_TEARDOWN_STATUS_NA;
    ringHandle = NULL;
    drvHandle = eventHandle->drvHandle;
    vintrNum = eventHandle->vintrNum;
    DebugP_assert(vintrNum != UDMA_EVENT_INVALID);
    /* Loop through all the shared events. In case of exclusive events,
     * the next event is NULL_PTR and the logic remains same and the while breaks */
    while(eventHandle != NULL_PTR)
    {
        /* There is no valid VINT bit for global master event */
        if(UDMA_EVENT_TYPE_MASTER != eventHandle->eventPrms.eventType)
        {
            DebugP_assert(eventHandle->vintrBitNum <= UDMA_MAX_EVENTS_PER_VINTR);
            vintrBitNum = vintrNum * UDMA_MAX_EVENTS_PER_VINTR;
            vintrBitNum += eventHandle->vintrBitNum;

            /* Check IA status */
            if((bool)true == CSL_intaggrIsIntrPending(&drvHandle->iaRegs, vintrBitNum, (bool)true))
            {
                /* Clear the interrupt */
                (void) CSL_intaggrClrIntr(&drvHandle->iaRegs, vintrBitNum);

                /* Notify through callback if registered */
                eventPrms = &eventHandle->eventPrms;

                if((UDMA_EVENT_TYPE_DMA_COMPLETION == eventPrms->eventType) ||
                   (UDMA_EVENT_TYPE_TEARDOWN_COMPLETION == eventPrms->eventType))
                {
                    DebugP_assert(eventPrms->chHandle != NULL_PTR);
                    ringHandle = ((Udma_ChHandleInt) (eventPrms->chHandle))->cqRing;

                    /* Read the teardown status bit in the Reverse Ring Occupancy register */
                    if( CSL_lcdma_ringaccIsTeardownComplete(&ringHandle->drvHandle->lcdmaRaRegs, ringHandle->ringNum) == TRUE )
                    {
                        teardownStatus = UDMA_EVENT_CH_TEARDOWN_STATUS_COMPLETE;
                    }
                    else
                    {
                        teardownStatus = UDMA_EVENT_CH_TEARDOWN_STATUS_INCOMPLETE;
                    }
                }

                /* Since DMA completion and teardown completion are registered on the same event, teardown status bit in the
                   Reverse Ring Occupancy register must be used to differentiate both events. If it is a DMA Completion event,
                   the teardown status bit should not be set and if it is a teardown completion event, the teardown status bit
                   must be set */
                if(!(((eventHandle->eventPrms.eventType == UDMA_EVENT_TYPE_DMA_COMPLETION) && (teardownStatus == UDMA_EVENT_CH_TEARDOWN_STATUS_COMPLETE)) ||
                 ((eventHandle->eventPrms.eventType == UDMA_EVENT_TYPE_TEARDOWN_COMPLETION) && (teardownStatus == UDMA_EVENT_CH_TEARDOWN_STATUS_INCOMPLETE))))
                {
                    if((Udma_EventCallback) NULL_PTR != eventPrms->eventCb)
                    {
                        eventPrms->eventCb(
                            eventHandle, eventPrms->eventType, eventPrms->appData);
                    }
                }
            }
        }

        /* Move to next shared event */
        eventHandle = eventHandle->nextEvent;
    }

    return;
}

static int32_t Udma_eventCheckParams(Udma_DrvHandleInt drvHandle,
                                     const Udma_EventPrms *eventPrms)
{
    int32_t             retVal = UDMA_SOK;
    Udma_EventHandleInt masterEventHandle;

    DebugP_assert(eventPrms != NULL_PTR);

    /* Exclusive event checks */
    if(UDMA_EVENT_MODE_EXCLUSIVE == eventPrms->eventMode)
    {
        if(NULL_PTR != eventPrms->masterEventHandle)
        {
            retVal = UDMA_EINVALID_PARAMS;
            DebugP_logError("[UDMA] Master event handle should be NULL_PTR for exclusive event!!!\r\n");
        }
    }

    /* Shared event checks */
    if(UDMA_EVENT_MODE_SHARED == eventPrms->eventMode)
    {
        /* Shared event slave checks */
        if(NULL_PTR != eventPrms->masterEventHandle)
        {
            /* Check if callback is non-null for slave shared events when
             * master has callback set - This is becasuse once the master has
             * interrupt registered, all slaves should have a callback as IA
             * is same and there is no individual control to disable
             * interrupt */
            masterEventHandle = (Udma_EventHandleInt) eventPrms->masterEventHandle;
            if(((Udma_EventCallback) NULL_PTR != masterEventHandle->eventPrms.eventCb) &&
               ((Udma_EventCallback) NULL_PTR == eventPrms->eventCb))
            {
                retVal = UDMA_EINVALID_PARAMS;
                DebugP_logError("[UDMA] No callback set for slave shared events!!!\r\n");
            }
            /* Check if master has not registered a callback, the slave should not
             * expect a callback either!! */
            if(((Udma_EventCallback) NULL_PTR == masterEventHandle->eventPrms.eventCb) &&
               ((Udma_EventCallback) NULL_PTR != eventPrms->eventCb) &&
               (UDMA_EVENT_TYPE_MASTER != masterEventHandle->eventPrms.eventType))
            {
                retVal = UDMA_EINVALID_PARAMS;
                DebugP_logError("[UDMA] Callback set for slave shared events when master event didnot set a callback!!!\r\n");
            }
        }
    }

    /* Channel handle should be provided to reconfigure channel related event */
    if((UDMA_EVENT_TYPE_DMA_COMPLETION == eventPrms->eventType) ||
       (UDMA_EVENT_TYPE_TEARDOWN_COMPLETION == eventPrms->eventType) ||
       (UDMA_EVENT_TYPE_TEARDOWN_PACKET == eventPrms->eventType) ||
       (UDMA_EVENT_TYPE_TR == eventPrms->eventType))
    {
        if(NULL_PTR == eventPrms->chHandle)
        {
            retVal = UDMA_EINVALID_PARAMS;
            DebugP_logError("[UDMA] Channel handle should be provided for ring/ch OES programming!!!\r\n");
        }
    }

    /* Ring handle should be provided to configure ring event */
    if(UDMA_EVENT_TYPE_RING == eventPrms->eventType)
    {
        if(NULL_PTR == eventPrms->ringHandle)
        {
            retVal = UDMA_EINVALID_PARAMS;
            DebugP_logError("[UDMA] Ring handle should be provided for ring OES programming!!!\r\n");
        }
    }

    if(UDMA_EVENT_TYPE_MASTER == eventPrms->eventType)
    {
        if(UDMA_EVENT_MODE_SHARED != eventPrms->eventMode)
        {
            retVal = UDMA_EINVALID_PARAMS;
            DebugP_logError("[UDMA] Event should be shareable for global master event type!!!\r\n");
        }

        if(NULL_PTR != eventPrms->masterEventHandle)
        {
            retVal = UDMA_EINVALID_PARAMS;
            DebugP_logError("[UDMA] Master handle should be NULL_PTR for master event type!!!\r\n");
        }
    }

    return (retVal);
}

static int32_t Udma_eventCheckUnRegister(Udma_DrvHandleInt drvHandle,
                                         Udma_EventHandleInt eventHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_EventPrms     *eventPrms;
    Udma_RingHandle     ringHandle;
    uint32_t            fOcc;
    uint32_t            rOcc;

    DebugP_assert(eventHandle != NULL_PTR);
    eventPrms = &eventHandle->eventPrms;

    if(eventHandle->eventInitDone != UDMA_INIT_DONE)
    {
        retVal = UDMA_EFAIL;
    }
    if(UDMA_SOK == retVal)
    {
        /* Can't free-up master event when shared events are still not yet
        * unregistered */
        if((NULL_PTR == eventPrms->masterEventHandle) &&
           (NULL_PTR != eventHandle->nextEvent))
        {
            retVal = UDMA_EFAIL;
            DebugP_logError("[UDMA] Can't free master event when shared events are still registered!!!\r\n");
        }
    }

     if(UDMA_SOK == retVal)
    {
        /* Ring occupancies must be zero before unregistering Ring / DMA completion events.
         * This is to make sure that there is no resource leak, because unregistering
         * these events will reset the ring. */
        if((UDMA_EVENT_TYPE_DMA_COMPLETION == eventPrms->eventType) ||
           (UDMA_EVENT_TYPE_TEARDOWN_COMPLETION == eventPrms->eventType) ||
           (UDMA_EVENT_TYPE_RING == eventPrms->eventType))
        {
            if((UDMA_EVENT_TYPE_DMA_COMPLETION == eventPrms->eventType) || (UDMA_EVENT_TYPE_TEARDOWN_COMPLETION == eventPrms->eventType))
            {
                DebugP_assert(eventPrms->chHandle != NULL_PTR);
                ringHandle = ((Udma_ChHandleInt) (eventPrms->chHandle))->cqRing;
            }
            else
            {
                ringHandle = eventPrms->ringHandle;
            }

            DebugP_assert(ringHandle != NULL_PTR);
            fOcc = Udma_ringGetForwardRingOcc(ringHandle);
            rOcc = Udma_ringGetReverseRingOcc(ringHandle);

            if((0U != fOcc) || (0U != rOcc))
            {
                retVal = UDMA_EFAIL;
                DebugP_logError("[UDMA] Can't unregister the event when descriptors are present in the ring!!!\r\n");
            }
        }
    }

    return (retVal);
}

static int32_t Udma_eventAllocResource(Udma_DrvHandleInt drvHandle,
                                       Udma_EventHandleInt eventHandle)
{
    int32_t                 retVal = UDMA_SOK;
    uint32_t                vintrNum;
    uint32_t                preferredIrIntrNum;
    const Udma_EventPrms   *eventPrms;
    Udma_EventHandleInt     lastEvent;
    uintptr_t               cookie;

    DebugP_assert(eventHandle != NULL_PTR);
    eventPrms = &eventHandle->eventPrms;

    /* Allocate event irrespective of all modes except global master event */
    if(UDMA_EVENT_TYPE_MASTER != eventPrms->eventType)
    {
        eventHandle->globalEvent = Udma_rmAllocEvent(drvHandle);
        if(UDMA_EVENT_INVALID == eventHandle->globalEvent)
        {
            retVal = UDMA_EALLOC;
            DebugP_logError("[UDMA] Global event alloc failed!!!\r\n");
        }
        else
        {
            DebugP_assert(drvHandle->iaRegs.pImapRegs != NULL_PTR);
            eventHandle->pIaGeviRegs =
                &drvHandle->iaRegs.pImapRegs->GEVI[eventHandle->globalEvent];
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Allocate IA register for master and exclusive events */
        if((UDMA_EVENT_MODE_EXCLUSIVE == eventPrms->eventMode) ||
            ((UDMA_EVENT_MODE_SHARED == eventPrms->eventMode) &&
                (NULL_PTR == eventPrms->masterEventHandle)))
        {
            eventHandle->vintrNum = Udma_rmAllocVintr(drvHandle);
            if(UDMA_EVENT_INVALID == eventHandle->vintrNum)
            {
                retVal = UDMA_EALLOC;
                DebugP_logError("[UDMA] VINTR alloc failed!!!\r\n");
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Allocate IA bit for all event modes except global master event */
        if(UDMA_EVENT_TYPE_MASTER != eventPrms->eventType)
        {
            eventHandle->vintrBitNum = Udma_rmAllocVintrBit(eventHandle);
            if(UDMA_EVENT_INVALID == eventHandle->vintrBitNum)
            {
                retVal = UDMA_EALLOC;
                DebugP_logError("[UDMA] VINTR bit alloc failed!!!\r\n");
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Allocate interrupt when callback is requested and only for
         * exclusive and master shared events (master handle is NULL_PTR) */
        if((((Udma_EventCallback) NULL_PTR != eventPrms->eventCb) &&
                (NULL_PTR == eventPrms->masterEventHandle)) ||
            (UDMA_EVENT_TYPE_MASTER == eventPrms->eventType))
        {
            if(UDMA_CORE_INTR_ANY != eventPrms->preferredCoreIntrNum)
            {
                preferredIrIntrNum = Udma_rmTranslateCoreIntrInput(drvHandle, eventPrms->preferredCoreIntrNum);
            }
            else
            {
                preferredIrIntrNum = eventPrms->preferredCoreIntrNum;
            }
            if(UDMA_INTR_INVALID != preferredIrIntrNum)
            {
                eventHandle->irIntrNum =
                    Udma_rmAllocIrIntr(preferredIrIntrNum, drvHandle);
                if(UDMA_INTR_INVALID != eventHandle->irIntrNum)
                {
                    eventHandle->coreIntrNum = Udma_rmTranslateIrOutput(drvHandle, eventHandle->irIntrNum);

                }
            }
            if(UDMA_INTR_INVALID == eventHandle->coreIntrNum)
            {
                retVal = UDMA_EALLOC;
                DebugP_logError("[UDMA] Core intr alloc failed!!!\r\n");
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Do atomic link list update as the same is used in ISR */
        cookie = HwiP_disable();

        /* Link shared events to master event */
        eventHandle->prevEvent = (Udma_EventHandleInt) NULL_PTR;
        eventHandle->nextEvent = (Udma_EventHandleInt) NULL_PTR;
        if(NULL_PTR != eventPrms->masterEventHandle)
        {
            /* Go to the last node - insert node at the end */
            lastEvent = (Udma_EventHandleInt) eventPrms->masterEventHandle;
            while(NULL_PTR != lastEvent->nextEvent)
            {
                /* Move to next node */
                lastEvent = lastEvent->nextEvent;
            }
            /* Cross link between last and current node */
            eventHandle->prevEvent = lastEvent;
            lastEvent->nextEvent   = eventHandle;
        }
        HwiP_restore(cookie);
    }

    if(UDMA_SOK == retVal)
    {
        if(UDMA_EVENT_TYPE_TR == eventPrms->eventType)
        {
            Udma_ChHandleInt chHandle;
            DebugP_assert(eventPrms->chHandle != NULL_PTR);
            chHandle = (Udma_ChHandleInt) eventPrms->chHandle;

            if(TRUE == chHandle->chOesAllocDone)
            {
                retVal = UDMA_EALLOC;
                DebugP_logError("[UDMA] Channel OES not de-allocated!!!\r\n");
            }
        }
    }

    if(UDMA_SOK != retVal)
    {
        Udma_eventFreeResource(drvHandle, eventHandle);
    }
    else
    {
        if(NULL_PTR == eventPrms->masterEventHandle)
        {
            vintrNum = eventHandle->vintrNum;
        }
        else
        {
            /* Use master event's info */
            vintrNum = ((Udma_EventHandleInt) (eventPrms->masterEventHandle))->vintrNum;
        }
        DebugP_assert(drvHandle->iaRegs.pIntrRegs != NULL_PTR);
        eventHandle->pIaVintrRegs = &drvHandle->iaRegs.pIntrRegs->VINT[vintrNum];
    }

    return (retVal);
}

static void Udma_eventFreeResource(Udma_DrvHandleInt drvHandle,
                                   Udma_EventHandleInt eventHandle)
{
    uintptr_t   cookie;

    /* Do atomic link list update as the same is used in ISR */
    cookie = HwiP_disable();

    /*
     * Remove this event node - link previous to next
     * Note: This is applicable only for shared mode. But the pointers will
     * be NULL_PTR for exclusive mode. Hence the logic is same.
     */
    /* Link previous's next to current's next */
    if(NULL_PTR != eventHandle->prevEvent)
    {
        eventHandle->prevEvent->nextEvent = eventHandle->nextEvent;
    }
    /* Link next's previous to current's previous */
    if(NULL_PTR != eventHandle->nextEvent)
    {
        eventHandle->nextEvent->prevEvent = eventHandle->prevEvent;
    }

    HwiP_restore(cookie);

    if(NULL_PTR != eventHandle->hwiHandle)
    {
        HwiP_destruct(&eventHandle->hwiObject);
        eventHandle->hwiHandle = NULL_PTR;
    }
    if(UDMA_INTR_INVALID != eventHandle->irIntrNum)
    {
        Udma_rmFreeIrIntr(eventHandle->irIntrNum, drvHandle);
        eventHandle->irIntrNum = UDMA_INTR_INVALID;
        eventHandle->coreIntrNum = UDMA_INTR_INVALID;
    }

    if(UDMA_EVENT_INVALID != eventHandle->globalEvent)
    {
        /* Reset steering */
        Udma_eventResetSteering(drvHandle, eventHandle);
        Udma_rmFreeEvent(eventHandle->globalEvent, drvHandle);
        eventHandle->globalEvent = UDMA_EVENT_INVALID;
    }
    if(UDMA_EVENT_INVALID != eventHandle->vintrBitNum)
    {
        Udma_rmFreeVintrBit(eventHandle->vintrBitNum, drvHandle, eventHandle);
        eventHandle->vintrBitNum = UDMA_EVENT_INVALID;
    }
    if(UDMA_EVENT_INVALID != eventHandle->vintrNum)
    {
        Udma_rmFreeVintr(eventHandle->vintrNum, drvHandle);
        eventHandle->vintrNum = UDMA_EVENT_INVALID;
    }

    return;
}

static int32_t Udma_eventConfig(Udma_DrvHandleInt drvHandle,
                                Udma_EventHandleInt eventHandle)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            vintrNum, coreIntrNum;
    Udma_ChHandleInt    chHandle;
    Udma_RingHandleInt  ringHandle;
    Udma_EventPrms     *eventPrms;
    HwiP_Params         hwiPrms;
    struct tisci_msg_rm_irq_set_req     rmIrqReq;
    struct tisci_msg_rm_irq_set_resp    rmIrqResp;

    DebugP_assert(eventHandle != NULL_PTR);
    eventPrms = &eventHandle->eventPrms;

    rmIrqReq.valid_params           = 0U;
    rmIrqReq.global_event           = 0U;
    rmIrqReq.src_id                 = 0U;
    rmIrqReq.src_index              = 0U;
    rmIrqReq.dst_id                 = 0U;
    rmIrqReq.dst_host_irq           = 0U;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    /* Event is always allocated except global master event */
    if(UDMA_EVENT_TYPE_MASTER != eventPrms->eventType)
    {
        rmIrqReq.valid_params  |= TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID;
        rmIrqReq.global_event   = (uint16_t)Udma_eventGetId(eventHandle);
    }

    /* IR setup */
    if(UDMA_INTR_INVALID != eventHandle->coreIntrNum)
    {
        /* Route Virtual interrupt (VINT) to core interrupt */
        DebugP_assert(eventHandle->vintrNum != UDMA_EVENT_INVALID);

        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
        rmIrqReq.dst_id        = drvHandle->devIdCore;
        rmIrqReq.dst_host_irq  = (uint16_t)eventHandle->coreIntrNum;
    }

    /* Get master IA register number for slaves */
    if(NULL_PTR != eventHandle->eventPrms.masterEventHandle)
    {
        vintrNum = ((Udma_EventHandleInt) (eventHandle->eventPrms.masterEventHandle))->vintrNum;
    }
    else
    {
        /* For master use the own register number */
        vintrNum = eventHandle->vintrNum;
    }
    DebugP_assert(vintrNum != UDMA_EVENT_INVALID);
    rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_IA_ID_VALID;
    rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_VINT_VALID;
    rmIrqReq.ia_id         = drvHandle->devIdIa;
    rmIrqReq.vint          = (uint16_t)vintrNum;

    if(UDMA_EVENT_INVALID != eventHandle->vintrBitNum)
    {
        DebugP_assert(
            eventHandle->vintrBitNum <= UDMA_MAX_EVENTS_PER_VINTR);
        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID;
        rmIrqReq.vint_status_bit_index  = (uint8_t)eventHandle->vintrBitNum;
    }

    if((UDMA_EVENT_TYPE_DMA_COMPLETION == eventPrms->eventType) ||
       (UDMA_EVENT_TYPE_TEARDOWN_PACKET == eventPrms->eventType) ||
       (UDMA_EVENT_TYPE_TEARDOWN_COMPLETION == eventPrms->eventType))
    {
        DebugP_assert(eventPrms->chHandle != NULL_PTR);
        chHandle = (Udma_ChHandleInt) eventPrms->chHandle;

        rmIrqReq.src_id = drvHandle->srcIdRingIrq;
        if((UDMA_EVENT_TYPE_DMA_COMPLETION == eventPrms->eventType) || (UDMA_EVENT_TYPE_TEARDOWN_COMPLETION == eventPrms->eventType))
        {
            DebugP_assert(chHandle->cqRing != NULL_PTR);
            DebugP_assert(chHandle->cqRing->ringNum != UDMA_RING_INVALID);
            rmIrqReq.src_index = chHandle->cqRing->ringNum;
            if((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
            {
                rmIrqReq.src_index += drvHandle->blkCopyRingIrqOffset;
            }
            else if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
            {
                rmIrqReq.src_index += drvHandle->txRingIrqOffset;
            }
            else
            {
                rmIrqReq.src_index += drvHandle->rxRingIrqOffset;
            }
        }
        else
        {
            /* For devices like AM64x in which Teardown event is not supported,
             * it dosen't reach here since it is bypassed in eventRegister */
            DebugP_assert(chHandle->tdCqRing != NULL_PTR);
            DebugP_assert(chHandle->tdCqRing->ringNum != UDMA_RING_INVALID);
            rmIrqReq.src_index = chHandle->tdCqRing->ringNum;
            rmIrqReq.src_index += TISCI_RINGACC0_OES_IRQ_SRC_IDX_START;
        }
    }

    if(UDMA_EVENT_TYPE_TR == eventPrms->eventType)
    {
        if(UDMA_INST_TYPE_LCDMA_PKTDMA == drvHandle->instType)
        {
            /* TR Event is not supported for PKTMDA */
            retVal = UDMA_EFAIL;
            DebugP_logError("[UDMA] TR event not supported for PKTDMA instance; Event config failed!!!\r\n");
        }
        else
        {
            DebugP_assert(eventPrms->chHandle != NULL_PTR);
            chHandle = (Udma_ChHandleInt) eventPrms->chHandle;
            rmIrqReq.src_id = drvHandle->srcIdTrIrq;
            if((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
            {
                DebugP_assert(chHandle->txChNum != UDMA_DMA_CH_INVALID);
                rmIrqReq.src_index = (uint16_t)chHandle->txChNum;
                rmIrqReq.src_index += drvHandle->blkCopyTrIrqOffset;
            }
            else if((chHandle->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX)
            {
                DebugP_assert(chHandle->rxChNum != UDMA_DMA_CH_INVALID);
                rmIrqReq.src_index = (uint16_t)chHandle->rxChNum;
                rmIrqReq.src_index += drvHandle->rxTrIrqOffset;
            }
            else if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
            {
                DebugP_assert(chHandle->txChNum != UDMA_DMA_CH_INVALID);
                rmIrqReq.src_index = (uint16_t)chHandle->txChNum;
                rmIrqReq.src_index += drvHandle->txTrIrqOffset;
            }
            else
            {
                /* DMSC RM doesn't program the DRU OES - program locally for now
                * in Udma_eventProgramSteering() */
                /* Use a SRC which doesn't need a OES programming so that DMSC will skip */
                rmIrqReq.src_id = drvHandle->devIdIa;
                rmIrqReq.src_index = 0U;                /* Not used by DMSC RM */
            }
        }
    }

    if(UDMA_EVENT_TYPE_RING == eventPrms->eventType)
    {
        DebugP_assert(eventPrms->ringHandle != NULL_PTR);
        ringHandle = (Udma_RingHandleInt) eventPrms->ringHandle;
        DebugP_assert(ringHandle->ringNum != UDMA_RING_INVALID);

        rmIrqReq.src_id     = drvHandle->srcIdRingIrq;
        rmIrqReq.src_index  = ringHandle->ringNum;
        rmIrqReq.src_index += drvHandle->txRingIrqOffset;

#if ((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
        /* For mapped RX rings, subtract the already added TX offset and add RX offset */
        if((ringHandle->mappedRingGrp >= UDMA_NUM_MAPPED_TX_GROUP) &&
           (ringHandle->mappedRingGrp < (UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP)))
        {
            rmIrqReq.src_index -= drvHandle->txRingIrqOffset;
            rmIrqReq.src_index += drvHandle->rxRingIrqOffset;
        }
#endif
    }

    if(UDMA_SOK == retVal)
    {
        /* Program Output event steering based on event type */
        Udma_eventProgramSteering(drvHandle, eventHandle);

        if((drvHandle->instType    != UDMA_INST_TYPE_NORMAL) &&
           (UDMA_EVENT_TYPE_MASTER == eventPrms->eventType))
        {
            /* In case of devices like AM64x, where there are no IRs to configure
               no need to config the Global Master event using DMSC RM */
        }
        else
        {
            /* Config event */
            retVal = Sciclient_rmIrqSet(
                         &rmIrqReq, &rmIrqResp, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                DebugP_logError("[UDMA] Sciclient event config failed!!!\r\n");
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        /* Register after programming IA, so that when spurious interrupts
         * occur, we have a sane state/variables to handle it */
        if(UDMA_INTR_INVALID != eventHandle->coreIntrNum)
        {
            coreIntrNum = eventHandle->coreIntrNum;

            /* Register interrupt only when asked for */
            HwiP_Params_init(&hwiPrms);
            hwiPrms.intNum = coreIntrNum;
            hwiPrms.callback = &Udma_eventIsrFxn;
            hwiPrms.args = eventHandle;
            hwiPrms.priority = eventHandle->eventPrms.intrPriority;
            retVal = HwiP_construct(&eventHandle->hwiObject, &hwiPrms);
            if(SystemP_SUCCESS != retVal)
            {
                DebugP_logError("[UDMA] Intr registration failed!!!\r\n");
            }
            else
            {
                eventHandle->hwiHandle = &eventHandle->hwiObject;
            }
        }
    }

    return (retVal);
}

static int32_t Udma_eventReset(Udma_DrvHandleInt drvHandle,
                               Udma_EventHandleInt eventHandle)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            vintrNum;
    Udma_ChHandleInt    chHandle;
    Udma_RingHandleInt  ringHandle;
    Udma_EventPrms     *eventPrms;
    struct tisci_msg_rm_irq_release_req     rmIrqReq;

    DebugP_assert(eventHandle != NULL_PTR);
    eventPrms = &eventHandle->eventPrms;

    rmIrqReq.valid_params           = 0U;
    rmIrqReq.global_event           = 0U;
    rmIrqReq.src_id                 = 0U;
    rmIrqReq.src_index              = 0U;
    rmIrqReq.dst_id                 = 0U;
    rmIrqReq.dst_host_irq           = 0U;
    rmIrqReq.ia_id                  = 0U;
    rmIrqReq.vint                   = 0U;
    rmIrqReq.vint_status_bit_index  = 0U;
    rmIrqReq.secondary_host         = TISCI_MSG_VALUE_RM_UNUSED_SECONDARY_HOST;

    /* Event is always allocated except global master event */
    if(UDMA_EVENT_TYPE_MASTER != eventPrms->eventType)
    {
        rmIrqReq.valid_params  |= TISCI_MSG_VALUE_RM_GLOBAL_EVENT_VALID;
        rmIrqReq.global_event   = (uint16_t)Udma_eventGetId(eventHandle);
    }

    /* IR clear */
    if(UDMA_INTR_INVALID != eventHandle->coreIntrNum)
    {
        DebugP_assert(eventHandle->vintrNum != UDMA_EVENT_INVALID);

        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_DST_ID_VALID;
        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_DST_HOST_IRQ_VALID;
        rmIrqReq.dst_id       = drvHandle->devIdCore;
        rmIrqReq.dst_host_irq = (uint16_t)eventHandle->coreIntrNum;
    }

    /* Get master IA register number for slaves */
    if(NULL_PTR != eventHandle->eventPrms.masterEventHandle)
    {
        vintrNum = ((Udma_EventHandleInt) (eventHandle->eventPrms.masterEventHandle))->vintrNum;
    }
    else
    {
        /* For master use the own register number */
        vintrNum = eventHandle->vintrNum;
    }
    DebugP_assert(vintrNum != UDMA_EVENT_INVALID);
    rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_IA_ID_VALID;
    rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_VINT_VALID;
    rmIrqReq.ia_id         = drvHandle->devIdIa;
    rmIrqReq.vint          = (uint16_t)vintrNum;

    if(UDMA_EVENT_INVALID != eventHandle->vintrBitNum)
    {
        DebugP_assert(eventHandle->vintrBitNum <= UDMA_MAX_EVENTS_PER_VINTR);
        rmIrqReq.valid_params |= TISCI_MSG_VALUE_RM_VINT_STATUS_BIT_INDEX_VALID;
        rmIrqReq.vint_status_bit_index  = (uint8_t)eventHandle->vintrBitNum;
    }

    if((UDMA_EVENT_TYPE_DMA_COMPLETION == eventPrms->eventType) ||
       (UDMA_EVENT_TYPE_TEARDOWN_COMPLETION == eventPrms->eventType)||
       (UDMA_EVENT_TYPE_TEARDOWN_PACKET == eventPrms->eventType))
    {
        DebugP_assert(eventPrms->chHandle != NULL_PTR);
        chHandle = (Udma_ChHandleInt) eventPrms->chHandle;

        rmIrqReq.src_id = drvHandle->srcIdRingIrq;
        if((UDMA_EVENT_TYPE_DMA_COMPLETION == eventPrms->eventType) || (UDMA_EVENT_TYPE_TEARDOWN_COMPLETION == eventPrms->eventType))
        {
            DebugP_assert(chHandle->cqRing != NULL_PTR);
            DebugP_assert(chHandle->cqRing->ringNum != UDMA_RING_INVALID);
            rmIrqReq.src_index = chHandle->cqRing->ringNum;
            if((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
            {
                rmIrqReq.src_index += drvHandle->blkCopyRingIrqOffset;
            }
            else if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
            {
                rmIrqReq.src_index += drvHandle->txRingIrqOffset;
            }
            else
            {
                rmIrqReq.src_index += drvHandle->rxRingIrqOffset;
            }
        }
        else
        {
            /* For devices like AM64x in which Teardown event is not supported,
             * it dosen't reach here since its bypassed in eventUnregister */
            DebugP_assert(chHandle->tdCqRing != NULL_PTR);
            DebugP_assert(chHandle->tdCqRing->ringNum != UDMA_RING_INVALID);
            rmIrqReq.src_index = chHandle->tdCqRing->ringNum;
            rmIrqReq.src_index += TISCI_RINGACC0_OES_IRQ_SRC_IDX_START;
        }
    }

    if(UDMA_EVENT_TYPE_TR == eventPrms->eventType)
    {
        if(UDMA_INST_TYPE_LCDMA_PKTDMA == drvHandle->instType)
        {
            /* TR Event is not supported for PKTMDA */
            retVal = UDMA_EFAIL;
            DebugP_logError("[UDMA] TR event not supported for PKTDMA instance; Event reset failed!!!\r\n");
        }
        else
        {
            DebugP_assert(eventPrms->chHandle != NULL_PTR);
            chHandle = (Udma_ChHandleInt) eventPrms->chHandle;
            rmIrqReq.src_id = drvHandle->srcIdTrIrq;
            if((chHandle->chType & UDMA_CH_FLAG_BLK_COPY) == UDMA_CH_FLAG_BLK_COPY)
            {
                DebugP_assert(chHandle->txChNum != UDMA_DMA_CH_INVALID);
                rmIrqReq.src_index = (uint16_t)chHandle->txChNum;
                rmIrqReq.src_index += drvHandle->blkCopyTrIrqOffset;
            }
            else if((chHandle->chType & UDMA_CH_FLAG_RX) == UDMA_CH_FLAG_RX)
            {
                DebugP_assert(chHandle->rxChNum != UDMA_DMA_CH_INVALID);
                rmIrqReq.src_index = (uint16_t)chHandle->rxChNum;
                rmIrqReq.src_index += drvHandle->rxTrIrqOffset;
            }
            else if((chHandle->chType & UDMA_CH_FLAG_TX) == UDMA_CH_FLAG_TX)
            {
                DebugP_assert(chHandle->txChNum != UDMA_DMA_CH_INVALID);
                rmIrqReq.src_index = (uint16_t)chHandle->txChNum;
                rmIrqReq.src_index += drvHandle->txTrIrqOffset;
            }
            else
            {
                /* DMSC RM doesn't program the DRU OES - program locally for now
                * in Udma_eventProgramSteering() */
                /* Use a SRC which doesn't need a OES programming so that DMSC will skip */
                rmIrqReq.src_id = drvHandle->devIdIa;
                rmIrqReq.src_index = 0U;                /* Not used by DMSC RM */
            }
        }
    }

    if(UDMA_EVENT_TYPE_RING == eventPrms->eventType)
    {
        DebugP_assert(eventPrms->ringHandle != NULL_PTR);
        ringHandle = (Udma_RingHandleInt) eventPrms->ringHandle;
        DebugP_assert(ringHandle->ringNum != UDMA_RING_INVALID);

        rmIrqReq.src_id     = drvHandle->srcIdRingIrq;
        rmIrqReq.src_index  = ringHandle->ringNum;
        rmIrqReq.src_index += drvHandle->txRingIrqOffset;

#if ((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
        /* For mapped RX rings, subtract the already added TX offset and add RX offset */
        if((ringHandle->mappedRingGrp >= UDMA_NUM_MAPPED_TX_GROUP) &&
           (ringHandle->mappedRingGrp < (UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP)))
        {
            rmIrqReq.src_index -= drvHandle->txRingIrqOffset;
            rmIrqReq.src_index += drvHandle->rxRingIrqOffset;
        }
#endif
    }

    if(UDMA_SOK == retVal)
    {
        if((drvHandle->instType    != UDMA_INST_TYPE_NORMAL) &&
           (UDMA_EVENT_TYPE_MASTER == eventPrms->eventType))
        {
            /* In case of devices like AM64x, where there are no IRs
               no need to release the Global Master event using DMSC RM */
        }
        else
        {
            /* Reset event */
            retVal = Sciclient_rmIrqRelease(&rmIrqReq, UDMA_SCICLIENT_TIMEOUT);
            if(CSL_PASS != retVal)
            {
                DebugP_logError("[UDMA] Sciclient event reset failed!!!\r\n");
            }
        }
    }

    return (retVal);
}

static void Udma_eventProgramSteering(Udma_DrvHandleInt drvHandle,
                                      Udma_EventHandleInt eventHandle)
{
    Udma_ChHandleInt    chHandle;
    Udma_EventPrms     *eventPrms;

    DebugP_assert(eventHandle != NULL_PTR);
    eventPrms = &eventHandle->eventPrms;

    if(UDMA_EVENT_TYPE_TR == eventPrms->eventType)
    {
        DebugP_assert(eventPrms->chHandle != NULL_PTR);
        chHandle = (Udma_ChHandleInt) eventPrms->chHandle;

        /* Mark OES alloc flag */
        chHandle->chOesAllocDone = TRUE;
    }

    return;
}

static void Udma_eventResetSteering(Udma_DrvHandleInt drvHandle,
                                    Udma_EventHandleInt eventHandle)
{
    Udma_ChHandleInt    chHandle;
    Udma_EventPrms     *eventPrms;

    DebugP_assert(eventHandle != NULL_PTR);
    eventPrms = &eventHandle->eventPrms;

    if(UDMA_EVENT_TYPE_TR == eventPrms->eventType)
    {
        DebugP_assert(eventPrms->chHandle != NULL_PTR);
        chHandle = (Udma_ChHandleInt) eventPrms->chHandle;

        /* Mark OES alloc flag */
        chHandle->chOesAllocDone = FALSE;
    }

    return;
}
