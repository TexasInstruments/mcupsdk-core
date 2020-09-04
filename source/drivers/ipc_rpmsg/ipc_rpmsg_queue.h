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

#ifndef IPC_RPMSG_QUEUE_H_
#define IPC_RPMSG_QUEUE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* queue implementation using single linked list
 * no locks taken inside these APIs, locks should be
 * taken ouside as needed.
 */

struct RPMessage_QueueElem_s;

/* Q element structure, this MUST be the first field of a larger application
 * specific structure
 */
typedef struct RPMessage_QueueElem_s {

    struct RPMessage_QueueElem_s *next; /* pointer to next element in list */

} RPMessage_QueueElem;

/* Q data structure, when Q is empty head and tail point to NULL
 */
typedef struct RPMessage_Queue_s {

    struct RPMessage_QueueElem_s *head; /* pointer to head of Q */
    struct RPMessage_QueueElem_s *tail; /* pointer to tail of Q */

} RPMessage_Queue;

/* reset Q data structure to empty state */
static inline void RPMessage_queueReset(RPMessage_Queue *q)
{
    q->head = NULL;
    q->tail = NULL;
}

/* add element into Q, adds to `tail` */
static inline void RPMessage_queuePut(RPMessage_Queue *q, RPMessage_QueueElem *elem)
{
    elem->next = NULL;
    if( q->tail == NULL )
    {
        /* Q is empty, head and tail point to new element */
        q->head = elem;
        q->tail = elem;
    }
    else
    {
        /* Q is not empty, add to tail */
        q->tail->next = elem;
        q->tail = elem;
    }
}

/* get element from Q, extracts from `head` */
static inline RPMessage_QueueElem *RPMessage_queueGet(RPMessage_Queue *q)
{
    RPMessage_QueueElem *elem;

    if(q->head == NULL)
    {
        /* Q is empty, return NULL */
        elem = NULL;
    }
    else
    {
        /* Q is not empty, return head */
        elem = q->head;
        if(q->head == q->tail)
        {
            /* Q becomes empty due to extraction from head */
            q->head = NULL;
            q->tail = NULL;
        }
        else
        {
            /* Q is not empty due to extraction from head, move head to next element */
            q->head = q->head->next;
        }
        /* init next to NULL before returning */
        elem->next = NULL;
    }
    return elem;
}

#ifdef __cplusplus
}
#endif

#endif /* IPC_RPMSG_QUEUE_H_ */
