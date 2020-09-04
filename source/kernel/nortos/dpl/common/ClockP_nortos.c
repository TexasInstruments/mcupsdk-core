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

#include "ClockP_nortos_priv.h"

static void ClockP_sleepTicks(uint32_t ticks);
static void ClockP_addToList(ClockP_Struct *obj);

ClockP_Control gClockCtrl;

void ClockP_timerTickIsr(void *args)
{
    ClockP_Struct *obj, *temp;

    /* increment the systick counter */
    gClockCtrl.ticks++;

    /*  check if the clock instance has expired */
    while (((obj = gClockCtrl.list) != NULL) &&
           (obj->timeout == gClockCtrl.ticks)) {
        temp = obj->next;

        /* clock instance expired */
        obj->timeout = 0;

        /*
         *  Remove the clock object from the list before calling the
         *  callback function.  The callback may call ClockP_start(),
         *  setting gClockCtrl.list back to the object (if its timeout
         *  is the smallest).
         */
        gClockCtrl.list = temp;

        if (obj->period != 0) {
            obj->timeout = obj->period + gClockCtrl.ticks;

            /*
             *  Put the clock object back in the list before calling the
             *  callback.  The callback may call ClockP_stop(), so we don't
             *  want to put the clock object back in the list if it has been
             *  stopped.
             */
            ClockP_addToList(obj);
        }

        if(obj->callback!=NULL)
        {
            (obj->callback)((ClockP_Object*)obj, obj->args);
        }
    }
    ClockP_timerClearOverflowInt(gClockConfig.timerBaseAddr);
}

int32_t ClockP_construct(ClockP_Object *handle, ClockP_Params *params)
{
    ClockP_Struct *obj = (ClockP_Struct *)handle;

    DebugP_assert(sizeof(ClockP_Struct) < sizeof(ClockP_Object));

    /* populate the new clock instance */
    obj->callback = params->callback;
    obj->args = params->args;
    obj->timeout = 0;
    obj->period = params->period;
    obj->startTimeout = params->timeout;
    obj->next = NULL;

    if (params->start) {
        ClockP_start(handle);
    }

    return SystemP_SUCCESS;
}

void ClockP_destruct(ClockP_Object *handle)
{
    ClockP_Struct *obj = (ClockP_Struct*) handle;
    ClockP_Struct *temp;
    uintptr_t   key;

    key = HwiP_disable();

    if (gClockCtrl.list == NULL) {
        HwiP_restore(key);
    }
    else
    {
        if (gClockCtrl.list == obj) {
            /* Obj is first on list */
            gClockCtrl.list = gClockCtrl.list->next;
        }
        else {
            /* Search through list for obj */
            temp = gClockCtrl.list;
            while ((temp->next != obj) && (temp->next != NULL)) {
                temp = temp->next;
            }

            if (temp->next == obj) {
                temp->next = obj->next;
            }
        }

        HwiP_restore(key);
    }
}

uint32_t ClockP_usecToTicks(uint64_t usecs)
{
    return (uint32_t) (usecs / gClockCtrl.usecPerTick);
}

uint64_t ClockP_ticksToUsec(uint32_t ticks)
{
    return ( (uint64_t)ticks * gClockCtrl.usecPerTick);
}

uint32_t ClockP_getTicks()
{
    return ((uint32_t)gClockCtrl.ticks);
}

uint32_t ClockP_getTimeout(ClockP_Object *handle)
{
    ClockP_Struct *obj = (ClockP_Struct *)handle;

    if (obj->timeout > 0) {
        return (obj->timeout - gClockCtrl.ticks);
    }
    else {
        return (obj->startTimeout);
    }
}

uint32_t ClockP_isActive(ClockP_Object *handle)
{
    ClockP_Struct *obj = (ClockP_Struct *)handle;

    return (obj->timeout > 0);
}

void ClockP_Params_init(ClockP_Params *params)
{
    params->start = 0;
    params->timeout = 0;
    params->period = 0;
    params->callback = NULL;
    params->args = NULL;    
}

void ClockP_setTimeout(ClockP_Object *handle, uint32_t timeout)
{
    ClockP_Struct *obj = (ClockP_Struct*)handle;

    obj->startTimeout = timeout;
}

void ClockP_start(ClockP_Object *handle)
{
    ClockP_Struct *obj = (ClockP_Struct*)handle;
    uintptr_t   key;

    /* protect the context by disable the interrupt */
    key = HwiP_disable();

    /* in case the timer is active, restart it with the new timeout */
    if (obj->timeout > 0) {
        ClockP_stop(handle);
    }
    obj->timeout = gClockCtrl.ticks + obj->startTimeout;

    ClockP_addToList(obj);

    /* Enable interrupts */
    HwiP_restore(key);
}

void ClockP_stop(ClockP_Object *handle)
{
    ClockP_Struct *obj = (ClockP_Struct*)handle;
    ClockP_Struct *temp;
    uintptr_t   key;

    /* protect the context by disable the interrupt */
    key = HwiP_disable();

    temp = gClockCtrl.list;

    if (temp==NULL) {
        /* Enable interrupts */
        HwiP_restore(key);
    }
    else
    {
        if (gClockCtrl.list == obj) {
            gClockCtrl.list = gClockCtrl.list->next;
            obj->next = NULL;
            obj->timeout = 0;
            /* Enable interrupts */
            HwiP_restore(key);
        }
        else
        {
            while ((temp->next != obj) && (temp->next != NULL)) {
                temp = temp->next;
            }

            if (temp->next == obj) {
                temp->next = obj->next;
                obj->next = NULL;
                obj->timeout = 0;
            }

            /* Enable the systick interrupt */
            HwiP_restore(key);
        }
    }
}

void ClockP_sleep(uint32_t sec)
{
    uint64_t ticks = (uint64_t)sec * 1000000 / (uint64_t)gClockCtrl.usecPerTick;

    ClockP_sleepTicks((uint32_t)ticks);
}

void ClockP_usleep(uint32_t usec)
{
    uint64_t curTime, endTime;
    uint32_t ticksToSleep;

    curTime = ClockP_getTimeUsec();
    endTime = curTime + usec;

    if (usec >= gClockCtrl.usecPerTick) {
        ticksToSleep = usec / gClockCtrl.usecPerTick;
        ClockP_sleepTicks(ticksToSleep);
    }

    curTime = ClockP_getTimeUsec();
    while (curTime < endTime) {
        curTime = ClockP_getTimeUsec();
    }
}

/*
 *  Get the current time in microseconds.
 */
uint64_t ClockP_getTimeUsec()
{
    uint64_t ts;
    uint32_t timerCount;
    uint64_t ticks1;
    uint64_t ticks2;

    do {
        ticks1 = gClockCtrl.ticks;
        timerCount = ClockP_getTimerCount(gClockCtrl.timerBaseAddr);
        ticks2 = gClockCtrl.ticks;
    } while (ticks1 != ticks2);

    /* Get the current time in microseconds */
    ts = ticks2 * (uint64_t)gClockCtrl.usecPerTick 
             + (uint64_t) ( /* convert timer count to usecs */
                (uint64_t)(timerCount - gClockCtrl.timerReloadCount)*gClockCtrl.usecPerTick/(0xFFFFFFFFu - gClockCtrl.timerReloadCount) 
                );

    return (ts);
}

/*
 *  Add clock to control list.  Call with interrupts disabled.
 */
static void ClockP_addToList(ClockP_Struct *obj)
{
    /* check if the linked list exists */
    if (gClockCtrl.list != NULL) {
        ClockP_Struct *temp = gClockCtrl.list;

        if (temp->timeout > obj->timeout) {
            obj->next = gClockCtrl.list;
            gClockCtrl.list = obj;
        }
        else {
            while (temp->next != NULL) {
                if ((temp->next)->timeout > obj->timeout) {
                    obj->next = temp->next;
                    temp->next = obj;
                    break;
                }
                else {
                    temp = temp->next;
                }
            }

            if (temp->next == NULL) {
                temp->next = obj;
                obj->next = NULL;
            }
        }
    }
    else {
        gClockCtrl.list = obj;
        obj->next = NULL;
    }
}

/*
 *  Sleep for a given number of ClockP ticks.
 */
static void ClockP_sleepTicks(uint32_t ticks)
{
    SemaphoreP_Object semObj;

    if (ticks > 0) {
        /*
         *  Construct a semaphore with 0 count that will never be posted.
         *  We will timeout pending on this semaphore.
         */
        SemaphoreP_constructBinary(&semObj, 0);
        SemaphoreP_pend(&semObj, ticks);
        SemaphoreP_destruct(&semObj);
    }
}


