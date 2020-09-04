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

#include<kernel/dpl/EventP.h>
#include<kernel/dpl/HwiP.h>
#include <FreeRTOS.h>
#include <event_groups.h>

typedef struct EventP_Struct_
{
    StaticEventGroup_t eventObj;
    EventGroupHandle_t eventHndl;
} EventP_Struct;

int32_t EventP_construct(EventP_Object *obj)
{
    EventP_Struct   *pEvent = (EventP_Struct *)obj;
    int32_t         status = SystemP_SUCCESS;

    DebugP_assert(sizeof(EventP_Struct) <= sizeof(EventP_Object));

    pEvent->eventHndl = xEventGroupCreateStatic(&(pEvent->eventObj));

    if(pEvent->eventHndl == NULL)
    {
        status = SystemP_FAILURE;
    }

    return status;
}

void EventP_destruct(EventP_Object *obj)
{
    EventP_Struct *pEvent = (EventP_Struct *)obj;

    if(pEvent != NULL)
    {
        vEventGroupDelete(pEvent->eventHndl);
        pEvent->eventHndl = NULL;
    }
}

int32_t EventP_waitBits(EventP_Object  *obj,
                        uint32_t       bitsToWaitFor,
                        uint8_t        clearOnExit,
                        uint8_t        waitForAll,
                        uint32_t       timeToWaitInTicks,
                        uint32_t       *eventBits)
{
    EventP_Struct   *pEvent = (EventP_Struct *)obj;
    int32_t         status;

    if((pEvent == NULL) || (clearOnExit > 2) || (waitForAll > 2) || (eventBits == NULL))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        *eventBits = (uint32_t)xEventGroupWaitBits(pEvent->eventHndl,
                                                   (EventBits_t)bitsToWaitFor,
                                                   (BaseType_t)clearOnExit,
                                                   (BaseType_t)waitForAll,
                                                   (TickType_t)timeToWaitInTicks);
        status = SystemP_SUCCESS;
    }

    return status;
}

int32_t EventP_setBits(EventP_Object *obj, uint32_t bitsToSet)
{
    EventP_Struct   *pEvent = (EventP_Struct *)obj;
    int32_t         status;

    if(pEvent == NULL)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        if(HwiP_inISR())
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            BaseType_t xResult;

            xResult  = xEventGroupSetBitsFromISR(pEvent->eventHndl,
                                                 bitsToSet,
                                                 &xHigherPriorityTaskWoken);

            if(xResult != pdFAIL)
            {
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                status = SystemP_SUCCESS;
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
        else
        {
            xEventGroupSetBits(pEvent->eventHndl, (EventBits_t)bitsToSet);
            status  = SystemP_SUCCESS;
        }
    }
    return status;
}

int32_t EventP_clearBits(EventP_Object *obj, uint32_t bitsToClear)
{
    EventP_Struct   *pEvent = (EventP_Struct *)obj;
    int32_t         status;

    if(pEvent == NULL)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        if(HwiP_inISR())
        {
            BaseType_t xResult;

            xResult  = xEventGroupClearBitsFromISR(pEvent->eventHndl, (EventBits_t)bitsToClear);

            if(xResult != pdFAIL)
            {
                status = SystemP_SUCCESS;
            }
            else
            {
                status = SystemP_FAILURE;
            }
        }
        else
        {
            xEventGroupClearBits(pEvent->eventHndl, (EventBits_t)bitsToClear);
            status  = SystemP_SUCCESS;
        }
    }
    return status;
}

int32_t EventP_getBits(EventP_Object *obj, uint32_t *eventBits)
{
    EventP_Struct   *pEvent = (EventP_Struct *)obj;
    int32_t         status;

    if((pEvent == NULL) || (eventBits == NULL))
    {
        status = SystemP_FAILURE;
    }
    else
    {
        if(HwiP_inISR())
        {
            *eventBits = (uint32_t)xEventGroupGetBitsFromISR(pEvent->eventHndl);
        }
        else
        {
            *eventBits = (uint32_t)xEventGroupGetBits(pEvent->eventHndl);
        }
        status = SystemP_SUCCESS;
    }
    return status;
}
