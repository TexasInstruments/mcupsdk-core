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
 *  \file udma.c
 *
 *  \brief File containing the UDMA generic driver APIs.
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

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t Udma_init(Udma_DrvHandle drvHandle, const Udma_InitPrms *initPrms)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandleInt;

    /* Structure size assert */
    DebugP_assert(sizeof(Udma_DrvObjectInt) <= sizeof(Udma_DrvObject));
    DebugP_assert(sizeof(Udma_ChObjectInt) <= sizeof(Udma_ChObject));
    DebugP_assert(sizeof(Udma_EventObjectInt) <= sizeof(Udma_EventObject));
    DebugP_assert(sizeof(Udma_RingObjectInt) <= sizeof(Udma_RingObject));
    DebugP_assert(sizeof(Udma_FlowObjectInt) <= sizeof(Udma_FlowObject));

    if((drvHandle == NULL_PTR) || (initPrms == NULL_PTR))
    {
        retVal = UDMA_EBADARGS;
    }

    if(UDMA_SOK == retVal)
    {
        drvHandleInt = (Udma_DrvHandleInt) drvHandle;
        (void) memset(drvHandleInt, 0, sizeof(*drvHandleInt));
        (void) memcpy(&drvHandleInt->initPrms, initPrms, sizeof(Udma_InitPrms));
        UdmaRmInitPrms_init(initPrms->instId, &drvHandleInt->rmInitPrms);
        Udma_initDrvHandle(drvHandleInt);

        SemaphoreP_constructMutex(&drvHandleInt->rmLockObj);
        drvHandleInt->rmLock = &drvHandleInt->rmLockObj;

        Udma_rmInit(drvHandleInt);

        drvHandleInt->drvInitDone = UDMA_INIT_DONE;
        if(FALSE == initPrms->skipGlobalEventReg)
        {
            Udma_EventPrms  eventPrms;

            UdmaEventPrms_init(&eventPrms);
            eventPrms.eventType = UDMA_EVENT_TYPE_MASTER;
            eventPrms.eventMode = UDMA_EVENT_MODE_SHARED;
            retVal = Udma_eventRegister(
                            drvHandle, &drvHandleInt->globalEventObj, &eventPrms);
            if(UDMA_SOK != retVal)
            {
                DebugP_logError("[UDMA] Global master event register failed!!!\r\n");
            }
            else
            {
                drvHandleInt->globalEventHandle = &drvHandleInt->globalEventObj;
            }
        }

        if(UDMA_SOK != retVal)
        {
            /* Free-up allocated resources */
            SemaphoreP_destruct(&drvHandleInt->rmLockObj);
            drvHandleInt->rmLock = NULL_PTR;
            drvHandleInt->drvInitDone = UDMA_DEINIT_DONE;
        }
    }

    return (retVal);
}

int32_t Udma_deinit(Udma_DrvHandle drvHandle)
{
    int32_t             retVal = UDMA_SOK;
    Udma_DrvHandleInt   drvHandleInt = (Udma_DrvHandleInt) drvHandle;

    /* Error check */
    if((NULL_PTR == drvHandleInt) || (drvHandleInt->drvInitDone != UDMA_INIT_DONE))
    {
        retVal = UDMA_EBADARGS;
    }

    if(UDMA_SOK == retVal)
    {
        if(NULL_PTR != drvHandleInt->globalEventHandle)
        {
            retVal = Udma_eventUnRegister(drvHandleInt->globalEventHandle);
            if(UDMA_SOK != retVal)
            {
                DebugP_logError("[UDMA] Global event free failed!!!\r\n");
            }
            drvHandleInt->globalEventHandle = (Udma_EventHandleInt) NULL_PTR;
        }

        retVal += Udma_rmDeinit(drvHandleInt);
        if(UDMA_SOK != retVal)
        {
            DebugP_logError("[UDMA] RM deinit failed!!!\r\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        if(NULL_PTR != drvHandleInt->rmLock)
        {
            SemaphoreP_destruct(&drvHandleInt->rmLockObj);
            drvHandleInt->rmLock = NULL_PTR;
        }
        (void) memset(drvHandleInt, 0, sizeof(*drvHandleInt));
        drvHandleInt->drvInitDone = UDMA_DEINIT_DONE;
    }

    return (retVal);
}

int32_t UdmaInitPrms_init(uint32_t instId, Udma_InitPrms *initPrms)
{
    int32_t retVal = UDMA_SOK;

    /* Error check */
    if(NULL_PTR == initPrms)
    {
        retVal = UDMA_EBADARGS;
    }

    if(UDMA_SOK == retVal)
    {
        initPrms->instId                = instId;
        initPrms->skipGlobalEventReg    = FALSE;
        initPrms->virtToPhyFxn          = &Udma_defaultVirtToPhyFxn;
        initPrms->phyToVirtFxn          = &Udma_defaultPhyToVirtFxn;
    }

    return (retVal);
}
