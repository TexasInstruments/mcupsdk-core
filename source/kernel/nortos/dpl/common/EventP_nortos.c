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
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/EventP.h>


typedef struct EventP_Struct_
{
    volatile uint32_t eventMask;
} EventP_Struct;


static inline uint32_t EventP_ffs(uint32_t num)
{
    uint32_t pos = 0U;
    uint32_t numValue = num;
    if (numValue != 0U)
    {
        numValue = (numValue ^ (numValue & (numValue - 1U)));
        while (numValue != 0U)
        {
            numValue >>= 1;
            pos++;
        }
    }
    return pos;
}

int32_t EventP_construct(EventP_Object *obj)
{
    EventP_Struct *pEvent = (EventP_Struct *)obj;
    DebugP_assert(sizeof(EventP_Object) >= sizeof(EventP_Struct));
    pEvent->eventMask = 0;

    return SystemP_SUCCESS;
}

void EventP_destruct(EventP_Object *obj)
{
    /* Empty Function */
}

int32_t EventP_waitBits(EventP_Object  *obj,
                        uint32_t       bitsToWaitFor,
                        uint8_t        clearOnExit,
                        uint8_t        waitForAll,
                        uint32_t       timeout,
                        uint32_t       *eventBits)
{

    EventP_Struct *pEvent = (EventP_Struct *)obj;
    ClockP_Params clockParams;
    ClockP_Object clockObj;
    uintptr_t     key;
    int32_t       status  = SystemP_SUCCESS;

    /*
     * Always add Clock (but don't start) so that ClockP_isActive() below
     * is valid.  It's OK to add a Clock even when timeout is 0 or forever
     * (but it is not OK to start it).
     */
    ClockP_Params_init(&clockParams);
    clockParams.timeout = timeout;
    ClockP_construct(&clockObj, &clockParams);

    if ((timeout != SystemP_NO_WAIT) && (timeout != SystemP_WAIT_FOREVER))
    {
        ClockP_start(&clockObj);
    }

    key = HwiP_disable();

    while ((((waitForAll!=0U) && ((pEvent->eventMask &  bitsToWaitFor) != bitsToWaitFor)) ||
            ((waitForAll==0U) && ((pEvent->eventMask &  bitsToWaitFor) == 0U))) && (status == SystemP_SUCCESS))
    {
        HwiP_restore(key);

        key = HwiP_disable();

        if ((timeout != SystemP_WAIT_FOREVER) && (ClockP_isActive(&clockObj) == 0U))
        {
            /* Fall here when timeout has expired */
            status = SystemP_TIMEOUT;
        }
        if (timeout == SystemP_NO_WAIT)
        {
            /* Break without waiting */
            status = SystemP_TIMEOUT;
        }
    }
    *eventBits = (pEvent->eventMask &  bitsToWaitFor);

    if (clearOnExit != 0U)
    {
        pEvent->eventMask &= ~bitsToWaitFor;
    }

    HwiP_restore(key);

    ClockP_destruct(&clockObj);

    return status;

}

int32_t EventP_setBits(EventP_Object *obj, uint32_t bitsToSet)
{
    EventP_Struct *pEvent = (EventP_Struct *)obj;
    uintptr_t key;

    key = HwiP_disable();
    pEvent->eventMask |=  bitsToSet;
    HwiP_restore(key);
    return SystemP_SUCCESS;
}

int32_t EventP_clearBits(EventP_Object *obj, uint32_t bitsToClear)
{
   EventP_Struct *pEvent = (EventP_Struct *)obj;
   uintptr_t key;
   key = HwiP_disable();
   pEvent->eventMask &= ~bitsToClear;
   HwiP_restore(key);

    return SystemP_SUCCESS;
}

int32_t EventP_getBits(EventP_Object *obj, uint32_t *eventBits)
{
    EventP_Struct *pEvent = (EventP_Struct *)obj;
    uintptr_t key;
    key = HwiP_disable();
    *eventBits = pEvent->eventMask;
    HwiP_restore(key);

    return SystemP_SUCCESS;
}
