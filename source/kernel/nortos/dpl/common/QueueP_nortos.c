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


#include <kernel/dpl/QueueP.h>
#include <kernel/dpl/HwiP.h>
#include <drivers/hw_include/csl_types.h>

/*!
 *  @brief    QueueP_nortos structure
 */
typedef struct QueueP_nortos_s
{
    QueueP_Elem          queueHndl;
} QueueP_nortos;

/*
 *  ======== QueueP_create ========
 */
QueueP_Handle QueueP_create(QueueP_Object *obj)
{
    QueueP_Handle       ret_handle;
    QueueP_nortos     *handle = (QueueP_nortos *) NULL;
    QueueP_Elem          *q;

    /* Grab the memory */
    handle = (QueueP_nortos *) obj;

    q = &handle->queueHndl;

    q->next = q;
    q->prev = q;

    ret_handle = ((QueueP_Handle)handle);

    return ret_handle;
}

/*
 *  ======== QueueP_delete ========
 */
int32_t QueueP_delete(QueueP_Handle handle)
{
    DebugP_assert((handle != NULL));

    QueueP_nortos *queue = (QueueP_nortos *)handle;
    QueueP_Elem      *q;

    q = &queue->queueHndl;

    q->next = (QueueP_Elem *)NULL;
    q->prev = (QueueP_Elem *)NULL;

    return SystemP_SUCCESS;
}

/*
 *  ======== QueueP_get ========
 */
void * QueueP_get(QueueP_Handle handle)
{
    DebugP_assert((handle != NULL));

    uintptr_t       key;
    QueueP_nortos *queue = (QueueP_nortos *)handle;
    QueueP_Elem      *pElem = NULL;
    QueueP_Elem      *q;

    key = HwiP_disable();

    q = &queue->queueHndl;

    pElem = q->next;

    q->next = pElem->next;
    pElem->next->prev = q;

    HwiP_restore(key);

    return (pElem);
}

/*
 *  ======== QueueP_put ========
 */
int32_t QueueP_put(QueueP_Handle handle, void *elem)
{
    DebugP_assert((handle != NULL));

    uintptr_t       key;
    int32_t   ret_val = SystemP_SUCCESS;
    QueueP_nortos *queue = (QueueP_nortos *)handle;
    QueueP_Elem      *pElem = (QueueP_Elem *)elem;
    QueueP_Elem      *q;

    key = HwiP_disable();

    if((queue != NULL) && (elem != NULL))
    {
        q = &queue->queueHndl;

        pElem->next = q;
        pElem->prev = q->prev;
        q->prev->next = pElem;
        q->prev = pElem;
    }
    else
    {
       ret_val = SystemP_FAILURE;
    }

    HwiP_restore(key);

    return ret_val;
}

/*
 *  ======== QueueP_isEmpty ========
 */
uint32_t QueueP_isEmpty(QueueP_Handle handle)
{
    DebugP_assert((handle != NULL));

    uint32_t        ret_val;
    QueueP_nortos *queue = (QueueP_nortos *)handle;

    if(queue->queueHndl.next == &(queue->queueHndl))
    {
        ret_val = QueueP_EMPTY;
    }
    else
    {
        ret_val = QueueP_NOTEMPTY;
    }

    return (ret_val);
}

/* Nothing past this point */
