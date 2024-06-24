/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
/*
 *  ======== MailboxP_freertos.c ========
 */


#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <FreeRTOS.h>
#include "queue.h"
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/MailboxP.h>


/*!
 *  @brief    MailboxP_freertos structure
 */
typedef struct MailboxP_freertos_s
{
    StaticQueue_t       xqueueObj;
    QueueHandle_t       xqueueHndl;
} MailboxP_freertos;

void MailboxP_Params_init(MailboxP_Params *params)
{
    if (params != NULL)
    {
        params->pErrBlk =  NULL;
        params->name    =  NULL;
        params->size    =  0U;
        params->count   =  0U;
        params->bufsize =  0U;
        params->buf     =  NULL;
    }
    return;
}

MailboxP_Handle MailboxP_create(MailboxP_Object* pObj, const MailboxP_Params *params)
{
    DebugP_assert(sizeof(MailboxP_Object) >= sizeof(MailboxP_freertos));

    MailboxP_Handle     ret_handle; 
    MailboxP_freertos   *handle = (MailboxP_freertos *) pObj;

    DebugP_assert((params != NULL));
    DebugP_assert((params->buf != NULL));
    DebugP_assert((params->bufsize >= (params->size * params->count)));

    if (handle == NULL)
    {
        ret_handle = NULL;
    }
    else
    {
        handle->xqueueHndl = xQueueCreateStatic((UBaseType_t)params->count,
                                        (UBaseType_t)params->size, 
                                        (uint8_t*)params->buf,
                                        &handle->xqueueObj);
        if(handle->xqueueHndl == NULL)
        {
            /* If there was an error reset the mailbox object and return NULL. */
            ret_handle = NULL;
        }
        else
        {
            ret_handle = ((MailboxP_Handle)handle);
        }
    }

    return ret_handle;
}

int32_t MailboxP_delete(MailboxP_Handle handle)
{
    DebugP_assert((handle != NULL));

    int32_t ret_val = SystemP_SUCCESS;
    MailboxP_freertos *mailbox = (MailboxP_freertos *)handle;

    if (mailbox != NULL)
    {
        vQueueDelete(mailbox->xqueueHndl);
        ret_val = SystemP_SUCCESS;
    }
    else
    {
        ret_val = SystemP_FAILURE;
    }

    return ret_val;
}

int32_t MailboxP_post(MailboxP_Handle handle,
                              void * msg,
                              uint32_t timeout)
{
    DebugP_assert((handle != NULL));

    BaseType_t qStatus;
    int32_t ret_val = SystemP_SUCCESS;
    MailboxP_freertos *mailbox = (MailboxP_freertos *)handle;

    if (HwiP_inISR() != 0U)
    {
        BaseType_t xHigherPriorityTaskWoken = 0;

        /* timeout is ignored when in ISR mode */
        qStatus = xQueueSendToBackFromISR(mailbox->xqueueHndl, msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        if (timeout == SystemP_WAIT_FOREVER)
        {
            timeout = portMAX_DELAY;
        }
        qStatus = xQueueSendToBack(mailbox->xqueueHndl, msg, timeout);
    }

    if (qStatus == pdPASS)
    {
        ret_val = SystemP_SUCCESS;
    }
    else
    {
        ret_val = SystemP_TIMEOUT;
    }

    return ret_val;
}

int32_t MailboxP_pend(MailboxP_Handle handle,
                              void * msg,
                              uint32_t timeout)
{
    DebugP_assert((handle != NULL));
    
    BaseType_t qStatus;
    int32_t ret_val = SystemP_SUCCESS;
    MailboxP_freertos *mailbox = (MailboxP_freertos *)handle;

    if (HwiP_inISR() != 0U)
    {
        BaseType_t xHigherPriorityTaskWoken = 0;

        /* timeout is ignored when in ISR mode */
        qStatus = xQueueReceiveFromISR(mailbox->xqueueHndl, msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        if (timeout == SystemP_WAIT_FOREVER)
        {
            timeout = portMAX_DELAY;
        }
        qStatus = xQueueReceive(mailbox->xqueueHndl,
                                msg,
                                (timeout == SystemP_WAIT_FOREVER) ? portMAX_DELAY : timeout);
    }

    if (qStatus == pdPASS)
    {
        ret_val = SystemP_SUCCESS;
    }
    else
    {
        ret_val = SystemP_TIMEOUT;
    }

    return ret_val;
}

int32_t MailboxP_getNumPendingMsgs(MailboxP_Handle handle)
{
    DebugP_assert((handle != NULL));
    
    BaseType_t numMsgs;
    MailboxP_freertos *mailbox = (MailboxP_freertos *)handle;

    if (HwiP_inISR() != 0U)
    {
        numMsgs = uxQueueMessagesWaitingFromISR(mailbox->xqueueHndl);
    }
    else
    {
        numMsgs = uxQueueMessagesWaiting(mailbox->xqueueHndl);
    }

    return ((int32_t)numMsgs);
}

/* Nothing past this point */
