/*
 *  Copyright (c) Texas Instruments Incorporated 2022
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

/*!
 * \file timeSync_ptp_osal_priv.h
 *
 * \brief This file contains the interface to OSAL functions
 *        of PTP stack
 */

#ifndef TIMESYNC_PTP_OSAL_PRIV_H_
#define TIMESYNC_PTP_OSAL_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <include/core/enet_osal.h>
#include <kernel/dpl/TaskP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/**
 * @def TIMESYNC_PTP_DELAY_REQ_SEND_TASK_PRIORITY
 *      Priority of task which sends Pdelay request frames
 */
#define TIMESYNC_PTP_DELAY_REQ_SEND_TASK_PRIORITY            (6U)

/**
 * @def TIMESYNC_PTP_BG_TASK_TICK_PERIOD
 *      PTP BG task ticks at this rate
 */
#define TIMESYNC_PTP_BG_TASK_TICK_PERIOD                     (6U)

/**
 * @def TIMESYNC_PTP_TX_NOTIFY_TASK_PRIORITY
 *      Priority of task which process packet Tx notification
 */
#define TIMESYNC_PTP_TX_NOTIFY_TASK_PRIORITY                 (6U)

/**
 * @def TIMESYNC_PTP_RX_NOTIFY_TASK_PRIORITY
 *      Priority of task which process packet Rx notification
 */
#define TIMESYNC_PTP_RX_NOTIFY_TASK_PRIORITY                 (6U)

/**
 * @def TIMESYNC_PTP_NRT_TASK_PRIORITY
 *      Priority of task which process peer delay frames
 */
#define TIMESYNC_PTP_NRT_TASK_PRIORITY                       (2U)

/**
 * @def TIMESYNC_PTP_BACKGROUND_TASK_PRIORITY
 *      Priority of background Task to do computation
 */
#define TIMESYNC_PTP_BACKGROUND_TASK_PRIORITY                (3U)

/**
 * @def TIMESYNC_PTP_DELAY_REQ_SEND_TASK_STACKSIZE
 *      Stack size of task which sends Pdelay request frames
 */
#define TIMESYNC_PTP_DELAY_REQ_SEND_TASK_STACKSIZE            (8U * 1024U)

/**
 * @def TIMESYNC_PTP_TX_NOTIFY_TASK_STACKSIZE
 *      Stack size of task which process packet Tx notification
 */
#define TIMESYNC_PTP_TX_NOTIFY_TASK_STACKSIZE                 (8U * 1024U)

/**
 * @def TIMESYNC_PTP_RX_NOTIFY_TASK_STACKSIZE
 *      Stack size of task which process packet Rx notification
 */
#define TIMESYNC_PTP_RX_NOTIFY_TASK_STACKSIZE                 (8U * 1024U)

/**
 * @def TIMESYNC_PTP_NRT_TASK_STACKSIZE
 *      Stack size of task which process peer delay frames
 */
#define TIMESYNC_PTP_NRT_TASK_STACKSIZE                       (8U * 1024U)

/**
 * @def TIMESYNC_PTP_BACKGROUND_TASK_STACKSIZE
 *      Stack size of background Task to do computation
 */
#define TIMESYNC_PTP_BACKGROUND_TASK_STACKSIZE                (8U * 1024U)

#define TIME_SYNC_UNABLE_TO_CREATE_EVENT                                    (-7)


/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/**
 * @brief PTP Tx notification  message structure
 */
typedef struct TimeSyncPtp_TxNotifyMsg_s
{
     /** Port number to which frame was transmitted*/
     uint8_t portNum;

     /** PTP frame type of transmitted frame*/
     uint8_t frameType;

     /** PTP sequence Id of transmitted frame */
     uint16_t seqId;
} TimeSyncPtp_TxNotifyMsg;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Create PTP semaphores, tasks and timers
 * @param timeSyncHandle pointer to PTP Handle structure
 * @return Error status
 */
int8_t TimeSyncPtp_createPtpTasks(TimeSyncPtp_Handle timeSyncPtpHandle);

/**
 * @brief This function processes Tx notification called from LLD.
 * @param  arg generic argument PTP Handle is passed through this
 * @return none
 */
void TimeSyncPtp_processTxNotifyTask(void *arg);

/**
 * @brief This function processes Rx notification called from LLD.
 * @param  arg generic argument PTP Handle is passed through this
 * @return none
 */
void TimeSyncPtp_processRxNotifyTask(void *arg);
#endif /* TIMESYNC_PTP_OSAL_PRIV_H_ */
