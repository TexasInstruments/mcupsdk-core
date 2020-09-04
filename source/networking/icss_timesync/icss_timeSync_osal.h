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

#ifndef ICSS_TIMESYNC_OSAL_H_
#define ICSS_TIMESYNC_OSAL_H_

#ifdef __cplusplus
extern "C"
{
#endif


#include "icss_timeSync.h"
#include "icss_timeSyncApi.h"
#include "icss_timeSync_init.h"

/**
 * \def PTP_BG_TASK_TICK_PERIOD
 *      PTP BG task ticks at this rate
 */
#define PTP_BG_TASK_TICK_PERIOD                     10

/**
 * \def PTP_TX_CALLBACK_HWI_PRIORITY
 *      Priority for Tx callback interrupt. Range 0-15
 */
#define PTP_TX_CALLBACK_HWI_PRIORITY                15

/**
 * \internal
 * \brief Initialize PTP ISRs, semaphores, tasks and events
 * \param timeSyncHandle pointer to PTP Handle structure
 * \return Error status
 */
int8_t TimeSync_isrAndTaskInit(TimeSync_ParamsHandle_t timeSyncHandle);

/**
 *
 * \internal
 * \brief This is used for
 * 1. Sending Peer delay request messages used to calculate peer delay
 * \param  args generic argument. PTP Handle is passed through this
 * \return none
 */
void TimeSync_PdelayReqSendTask(void *args);

/**
 *
 * \internal
 * \brief This is used for
 *  Sending delay request messages to master to calculate line delay
 * \param  args generic argument. PTP Handle is passed through this
 * \return none
 */
void TimeSync_delayReqSendTask(void *args);

/**
 *
 * \internal
 * \brief This is used for
 * 1. Extracting Tx Timestamp information when a callback interrupt is triggered from firmware
 * 2. Checks for the type of packet and calls relevant Tx API
 * 3. For Port 1
 * \param  args generic argument. PTP Handle is passed through this
 * \return none
 */
void TimeSync_TxTSTask_P1(void *args);

/**
 *
 * \internal
 * \brief This is used for
 * 1. Extracting Tx Timestamp information when a callback interrupt is triggered from firmware
 * 2. Checks for the type of packet and calls relevant Tx API
 * 3. For Port 2
 * \param  args generic argument. PTP Handle is passed through this
 * \return none
 */
void TimeSync_TxTSTask_P2(void *args);
/**
 *
 * \internal
 * \brief This is used for
 * 1. Checking for missed Sync frames
 * 2. Running part of offset stabilization algorithm
 * \param  args generic argument. PTP Handle is passed through this
 * \return none
 */
void TimeSync_BackgroundTask(void *args);

/**
 *
 * \internal
 * \brief NRT stands for non-real time
 * This task is responsible for processing Peer delay messages and
 * calculating peer delay
 * \param  args generic argument. PTP Handle is passed through this
 * \return none
 */
void TimeSync_NRT_Task(void *args);

/**
 *
 * \internal
 * \brief This task sends a Follow Up frame on a tx callback interrupt
 * when device is configured as 2-step master
 * \param  args generic argument. PTP Handle is passed through this
 * \return none
 */
void TimeSync_FollowUpTxTask(void *args);

/**
 *
 * \internal
 * \brief This is triggered by a Time sync frame getting timestamped on exit. It posts
 * a semaphore which in turn triggers the copying of Tx timestamps
 * \param  arg generic argument PTP Handle is passed through this
 * \return none
 */
void TimeSync_txTSIsr(uintptr_t arg);


#ifdef __cplusplus
}
#endif

#endif /* ICSS_TIMESYNC_OSAL_H_ */
