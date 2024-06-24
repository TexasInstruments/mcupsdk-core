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
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \ingroup KERNEL_DPL
 *  \defgroup KERNEL_DPL_MAILBOX for MailboxP
 *            MailboxP interface
 *
 *  @{
 */
/** ============================================================================
 *  @file       MailboxP.h
 *
 *  @brief      Mailbox module for the RTOS Porting Interface.
  *
 *  ============================================================================
 */

#ifndef KERNEL_DPL_MAILBOX_H
#define KERNEL_DPL_MAILBOX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * \brief Max size of semaphore object across no-RTOS and all OS's
 */
#define MailboxP_OBJECT_SIZE_MAX    (152U)
/**
 * \brief Opaque semaphore object used with the semaphore APIs
 */
typedef struct MailboxP_Object_ {

    /* uintptr_t translates to uint64_t for A53 and uint32_t for R5 and M4 */
    /* This accounts for the 64bit pointer in A53 and 32bit pointer in R5 and M4 */
    uintptr_t rsv[MailboxP_OBJECT_SIZE_MAX/sizeof(uint32_t)]; /**< reserved, should NOT be modified by end users */

} MailboxP_Object;

/*!
 *  @brief    Opaque client reference to an instance of a MailboxP
 *
 *  A MailboxP_Handle returned from the ::MailboxP_create represents that
 *  instance and  is used in the other instance based functions
 */
typedef  void *MailboxP_Handle;

/*!
 *  @brief    Basic MailboxP Parameters
 *
 *  Structure that contains the parameters are passed into ::MailboxP_create
 *  when creating a MailboxP instance.
 *  NOTE : bufsize in SafeRTOS should have  additional portQUEUE_OVERHEAD_BYTES.
 */
typedef struct MailboxP_Params_s
{
    void *pErrBlk;     /*!< Pointer to the error block for mailbox create */
    void *name;        /*!< Name of the mailbox */
    uint32_t size;     /*!< The size of message, in bytes*/
    uint32_t count;    /*!< Length of mailbox. The maximum number of items the mailbox can hold at any one time. */
    uint32_t bufsize;  /*!< Buffer size. This should be large enough to hold the maximum number of items */
    void *buf;         /*!< pointer to buffer memory */
} MailboxP_Params;


/*!
 *  @brief  Initialize params structure to default values.
 *
 *  @param params  Pointer to the instance configuration parameters.
 */
void MailboxP_Params_init(MailboxP_Params *params);

/*!
 *  @brief  Function to create a mailbox.
 *
 *  @param  pObj [out] created object
 *  @param  params  Pointer to the instance configuration parameters.
 *
 *  @return A MailboxP_Handle on success or a NULL on an error
 */
MailboxP_Handle MailboxP_create(MailboxP_Object* pObj, const MailboxP_Params *params);

/*!
 *  @brief  Function to delete a mailbox.
 *
 *  @param  handle  A MailboxP_Handle returned from MailboxP_create
 *
 *  @return Status of the functions
 *    - SystemP_SUCCESS: Deleted the mailbox instance
 *    - SystemP_FAILURE: Failed to delete the mailbox instance
 */
int32_t MailboxP_delete(MailboxP_Handle handle);

/*!
 *  @brief  Function to post an message to the mailbox.
 *
 *  @param  handle  A MailboxP_Handle returned from MailboxP_create
 *
 *  @param  msg  The message to post
 *
 *  @param  timeout Timeout (in milliseconds) to wait for post a
 *                  message to the mailbox.
 *
 *  @return Status of the functions
 *    - SystemP_SUCCESS: Deleted the mailbox instance
 *    - SystemP_TIMEOUT: Timed out posting of message
 *    - SystemP_FAILURE: Failed to start the mailbox instance
 */
int32_t MailboxP_post(MailboxP_Handle handle,
                      void* pMsg,
                      uint32_t timeout);

/*!
 *  @brief  Function to pend on a message for the mailbox.
 *
 *  @param  handle  A MailboxP_Handle returned from MailboxP_create
 *
 *  @param  msg  The message to pend
 *
 *  @param  timeout Timeout (in milliseconds) to wait for pend on a 
 *                  message to the mailbox.
 *
 *  @return Status of the functions
 *    - SystemP_SUCCESS: Deleted the mailbox instance
 *    - SystemP_TIMEOUT: Timed out pending on message
 *    - SystemP_FAILURE: Failed to stop the mailbox instance
 */
int32_t MailboxP_pend(MailboxP_Handle handle,
                       void* pMsg,
                       uint32_t timeout);

/*!
 *  @brief  Function to return the count of the pending messages
 *
 *  @param  handle  A MailboxP_Handle returned from MailboxP_create
 *
 *  @return The count of the pending messages
 */
int32_t MailboxP_getNumPendingMsgs(MailboxP_Handle handle);

#ifdef __cplusplus
}
#endif

#endif /* KERNEL_DPL_MAILBOX_H */
/* @} */
