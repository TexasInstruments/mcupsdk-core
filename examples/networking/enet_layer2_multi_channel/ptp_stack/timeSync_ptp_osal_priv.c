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
 * \file timeSync_ptp_osal_priv.c
 *
 * \brief This file contains the implementation of OSAL functions
 *        of PTP stack
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "timeSync.h"
#include "timeSync_tools.h"

#include "timeSync_ptp.h"
#include "timeSync_ptp_init_priv.h"
#include "timeSync_ptp_osal_priv.h"
#include "timeSync_ptp_priv.h"
#include <networking/enet/utils/include/enet_apputils.h>
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/**< Task Stack memory */
static uint8_t gTimeSyncPtp_processTxNotifyTaskStackBuf[TIMESYNC_PTP_TX_NOTIFY_TASK_STACKSIZE] __attribute__ ((aligned(32)));
static uint8_t gTimeSyncPtp_processRxNotifyTaskStackBuf[TIMESYNC_PTP_RX_NOTIFY_TASK_STACKSIZE] __attribute__ ((aligned(32)));
static uint8_t gTimeSyncPtp_pdelayReqSendTaskStackBuf[TIMESYNC_PTP_DELAY_REQ_SEND_TASK_STACKSIZE] __attribute__ ((aligned(32)));
static uint8_t gTimeSyncPtp_nRTTaskStackBuf[TIMESYNC_PTP_NRT_TASK_STACKSIZE] __attribute__ ((aligned(32)));
static uint8_t gTimeSyncPtp_backgroundTaskStackBuf[TIMESYNC_PTP_BACKGROUND_TASK_STACKSIZE] __attribute__ ((aligned(32)));

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
/**
 * @brief This is used for
 * 1. Sending Peer delay request messages used to calculate peer delay
 * @param  a0 generic argument. PTP Handle is passed through this
 * @param  a1 is NULL
 * @return none
 */
static void TimeSyncPtp_pdelayReqSendTask(void* args);

/**
 * @brief This is used for
 * 1. Checking for missed Sync frames
 * 2. Running part of offset stabilization algorithm
 * @param  a0 generic argument. PTP Handle is passed through this
 * @param  a1 is NULL
 * @return none
 */
static void TimeSyncPtp_backgroundTask(void* args);

/**
 * @brief NRT stands for non-real time
 * This task is responsible for processing Peer delay messages and
 * calculating peer delay
 * @param  a0 generic argument. PTP Handle is passed through this
 * @param  a1 is NULL
 * @return none
 */
static void TimeSyncPtp_nRTTask(void* args);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int8_t TimeSyncPtp_createPtpTasks(TimeSyncPtp_Handle hTimeSyncPtp)
{
    TaskP_Params taskParams;

    uint8_t portNum = 0;
    int8_t status = TIMESYNC_OK;//TODO check status values

	status = SemaphoreP_constructBinary(&hTimeSyncPtp->pktTxSemHandle, 0);
	DebugP_assert(SystemP_SUCCESS == status);
  
	status = SemaphoreP_constructBinary(&hTimeSyncPtp->pktRxSemHandle, 0);
	DebugP_assert(SystemP_SUCCESS == status);
    /*Assign event ID's*/
    hTimeSyncPtp->eventIdSync = EventP_ID_02;
    hTimeSyncPtp->eventIdPdelayReq  = EventP_ID_03;
    hTimeSyncPtp->eventIdPdelayResp = EventP_ID_04;
    hTimeSyncPtp->eventIdPdelayRespFlwUp = EventP_ID_05;
    hTimeSyncPtp->eventIdDelayReq = EventP_ID_06;
    hTimeSyncPtp->eventIdFlwUpGenerated = EventP_ID_07;

    for (portNum = 0; portNum < TIMESYNC_PTP_MAX_PORTS_SUPPORTED; portNum++)
    {
        if (TIMESYNC_IS_BIT_SET(hTimeSyncPtp->ptpConfig.portMask, portNum))
        {
			status = EventP_construct(&(hTimeSyncPtp->ptpPdelayResEvtHandle[portNum]));

			if(status == SystemP_FAILURE)
			{
				return TIME_SYNC_UNABLE_TO_CREATE_EVENT;
			}
        }
    }

    /*-----------Create Tasks------------*/

    /* Packet Tx process Task*/
    TaskP_Params_init(&taskParams);
    taskParams.priority = TIMESYNC_PTP_TX_NOTIFY_TASK_PRIORITY;
    taskParams.args = (void *)hTimeSyncPtp;
    taskParams.stack = &gTimeSyncPtp_processTxNotifyTaskStackBuf[0];
    taskParams.stackSize = TIMESYNC_PTP_TX_NOTIFY_TASK_STACKSIZE;
	taskParams.name           = "Tx Task";
    taskParams.taskMain           = &TimeSyncPtp_processTxNotifyTask;
	
	status = TaskP_construct(&hTimeSyncPtp->pktTxNotifyTask, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Packet Rx process Task*/
    TaskP_Params_init(&taskParams);
    taskParams.priority = TIMESYNC_PTP_RX_NOTIFY_TASK_PRIORITY;
    taskParams.args = (void *)hTimeSyncPtp;
    taskParams.stack = &gTimeSyncPtp_processRxNotifyTaskStackBuf[0];
    taskParams.stackSize = TIMESYNC_PTP_RX_NOTIFY_TASK_STACKSIZE;
	taskParams.name           = "Rx Task";
    taskParams.taskMain           = (void*)TimeSyncPtp_processRxNotifyTask;
	
	status = TaskP_construct(&hTimeSyncPtp->pktRxNotifyTask, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);

    if (hTimeSyncPtp->ptpConfig.type == TIMESYNC_PTP_DELAY_P2P)
    {
        TaskP_Params_init(&taskParams);
        taskParams.priority = TIMESYNC_PTP_DELAY_REQ_SEND_TASK_PRIORITY;
        taskParams.args = (void *)hTimeSyncPtp;
        taskParams.stack = &gTimeSyncPtp_pdelayReqSendTaskStackBuf[0];
        taskParams.stackSize = TIMESYNC_PTP_DELAY_REQ_SEND_TASK_STACKSIZE;
        taskParams.name = "Ptp delay Req Send Task";
        taskParams.taskMain = (void*) TimeSyncPtp_pdelayReqSendTask;

        status = TaskP_construct(&hTimeSyncPtp->pDelayReqSendTask, &taskParams);
        DebugP_assert(SystemP_SUCCESS == status);

    }

    /* NRT Task to process peer delay frames*/
    TaskP_Params_init(&taskParams);
    taskParams.priority  = TIMESYNC_PTP_NRT_TASK_PRIORITY;
    taskParams.args      = (void *)hTimeSyncPtp;
    taskParams.stack     = &gTimeSyncPtp_nRTTaskStackBuf[0];
    taskParams.stackSize = TIMESYNC_PTP_NRT_TASK_STACKSIZE;
    taskParams.name = "nRT Task";
    taskParams.taskMain = (void*) TimeSyncPtp_nRTTask;

    status = TaskP_construct(&hTimeSyncPtp->nRTTask, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);

    /* Background Task to do computation*/
    TaskP_Params_init(&taskParams);
    taskParams.priority = TIMESYNC_PTP_BACKGROUND_TASK_PRIORITY;
    taskParams.args     = (void *)hTimeSyncPtp;
    taskParams.stack    = &gTimeSyncPtp_backgroundTaskStackBuf[0];
    taskParams.stackSize = TIMESYNC_PTP_BACKGROUND_TASK_STACKSIZE;
	taskParams.name      = "Background Task";
    taskParams.taskMain  = (void*)TimeSyncPtp_backgroundTask;
	
	status = TaskP_construct(&hTimeSyncPtp->backgroundTask, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);

    return TIMESYNC_OK;
}

/* ========================================================================== */
/*                           Task Definitions                                 */
/* ========================================================================== */

static void TimeSyncPtp_pdelayReqSendTask(void* args)
{
    TimeSyncPtp_Handle hTimeSyncPtp;
    uint8_t linkStatus = 0U;
    uint8_t frameCount = 0U;
    uint8_t offset  = 0U;
    int32_t retVal  = 0U;
    uint8_t portNum = 0U;

    hTimeSyncPtp = (TimeSyncPtp_Handle)args;
    offset = hTimeSyncPtp->ptpConfig.frameOffset;

    while (1)
    {
        if (TIMESYNC_PTP_DELAY_P2P == hTimeSyncPtp->ptpConfig.type)
        {
            if (hTimeSyncPtp->enabled == TRUE)
            {
                /*Construct a Delay Request packet and send it on all ports */
                /*Send delay request frames in a burst*/
                for (frameCount = 0; frameCount < hTimeSyncPtp->ptpConfig.pdelayBurstNumPkts; frameCount++)
                {
                    for (portNum = 0; portNum < TIMESYNC_PTP_MAX_PORTS_SUPPORTED; portNum++)
                    {
                        if (TIMESYNC_IS_BIT_SET(hTimeSyncPtp->ptpConfig.portMask, portNum))
                        {
                            linkStatus = EnetApp_isPortLinkUp(hTimeSyncPtp->enet_perctxt, portNum);
                            if (linkStatus)
                            {
                                /*write sequence id into memory*/
                                TimeSync_addHalfWord(hTimeSyncPtp->timeSyncBuff.pdelayReqTxBuf[portNum] + TIMESYNC_PTP_SEQ_ID_OFFSET - offset,
                                                     hTimeSyncPtp->tsRunTimeVar.pDelReqSequenceID[portNum]);

                                /**Add Source Port ID*/
                                TimeSync_addHalfWord(hTimeSyncPtp->timeSyncBuff.pdelayReqTxBuf[portNum] + TIMESYNC_PTP_SRC_PORT_ID_OFFSET - offset,
                                                     (portNum + 1U));

                                retVal = EnetApp_sendPtpFrame(hTimeSyncPtp->enet_perctxt,
                                                               hTimeSyncPtp->timeSyncBuff.pdelayReqTxBuf[portNum],
                                                               TIMESYNC_PTP_PDELAY_BUF_SIZE,
                                                               portNum);
                                if (retVal == TIMESYNC_OK)
                                {
                                    hTimeSyncPtp->tsRunTimeVar.pDelReqSequenceID[portNum]++;
                                }
                            }

                            /*Add small delay between two requests on two ports
                             * to avoid collision scenarios
                             */
                            ClockP_usleep(ClockP_ticksToUsec(10));
                        }
                    }
                    ClockP_usleep(ClockP_ticksToUsec(hTimeSyncPtp->ptpConfig.pdelayBurstInterval));
                }
            }
            ClockP_usleep(ClockP_ticksToUsec(hTimeSyncPtp->ptpConfig.pDelReqPktInterval));
        }
    }
	TaskP_exit();
}

static void TimeSyncPtp_nRTTask(void* args)
{
    uint32_t events = 0U;
    TimeSyncPtp_Handle hTimeSyncPtp = (TimeSyncPtp_Handle)args;
    uint8_t port = 0U;
    int8_t linkStatus = 0;

    while (1)
    {
        for (port = 0; port < TIMESYNC_PTP_MAX_PORTS_SUPPORTED; port++)
        {
            if (TIMESYNC_IS_BIT_SET(hTimeSyncPtp->ptpConfig.portMask, port))
            {
                linkStatus = EnetApp_isPortLinkUp(hTimeSyncPtp->enet_perctxt, port);
                if (linkStatus)
                {
                    /*Wait 1ms then move on to other tasks*/
                    EventP_waitBits(&hTimeSyncPtp->ptpPdelayResEvtHandle[port],
                                         (hTimeSyncPtp->eventIdPdelayResp + hTimeSyncPtp->eventIdPdelayRespFlwUp),
                                         1,
                                         1,
                                         1,
                                         &events);

                    /*Calculate Peer Delay on port*/
                    if (events)
                    {
                        TimeSyncPtp_processPdelayRespFrame(hTimeSyncPtp,
                                                           hTimeSyncPtp->timeSyncBuff.pdelayResRxBuf[port],
                                                           FALSE,
                                                           port);

                        if (hTimeSyncPtp->pDelayParams[port].ifTwoStep)
                        {
                            TimeSyncPtp_processPdelayRespFrame(hTimeSyncPtp,
                                                               hTimeSyncPtp->timeSyncBuff.pdelayResFlwUpRxBuf[port],
                                                               TRUE,
                                                               port);
                        }
                    }
                }
                else
                {
                    /* linkStatus being false is not an issue, wait for sometime and retry.
                     * Note: Do not remove this sleep, this is required to prevent the task from
                     * becoming while(1); when only one port is used and the link is down for that port.*/
                    //TaskP_sleep(1);
                    ClockP_usleep(ClockP_ticksToUsec(1));
                }
            }
        }
    }
	TaskP_exit();
}

static void TimeSyncPtp_backgroundTask(void* args)
{
    TimeSyncPtp_Handle hTimeSyncPtp = (TimeSyncPtp_Handle)args;
    uint8_t count = 0;
    int32_t avgCorrection = 0;

    while (1)
    {
        /*Increment the tick counter*/
        hTimeSyncPtp->tsRunTimeVar.tickCounter++;

        /*check for sync timeout*/
        if (hTimeSyncPtp->tsRunTimeVar.syncLastSeenCounter++ >
            hTimeSyncPtp->tsRunTimeVar.syncTimeoutInterval)
        {
            hTimeSyncPtp->numSyncMissed++;
            /*If we continuously miss sync frames then reset*/
            if ((hTimeSyncPtp->numSyncMissed % TIMESYNC_PTP_NUM_SYNC_MISSED_THRESHOLD) == 0)
            {
                if (hTimeSyncPtp->ptpConfig.isMaster)
                {
                    /* No action for master mode as there is no Sync frame to
                       receive, Master sends the Sync Frame */
                }
                else
                {
                    TimeSyncPtp_reset(hTimeSyncPtp);
                }
            }

            /*reset so that we don't keep getting this error*/
            hTimeSyncPtp->tsRunTimeVar.syncLastSeenCounter = 0;
        }

        /*Check if we have to process the data*/
        if (hTimeSyncPtp->offsetAlgo.binFull == 1)
        {
            /*get average of all offsets*/
            avgCorrection = 0;

            for (count = 0; count < 5; count++)
            {
                avgCorrection += hTimeSyncPtp->offsetAlgo.correction[count];
            }

            avgCorrection /= 5;
            /*add the new correction value to initial offset or the PPM value*/
            hTimeSyncPtp->tsRunTimeVar.initialOffset += avgCorrection;
            /*Reset the variables so another round of measurements can begin*/
            hTimeSyncPtp->offsetAlgo.binFull = 0;
            hTimeSyncPtp->offsetAlgo.numEntriesIndex = 0;
        }
        ClockP_usleep(ClockP_ticksToUsec(TIMESYNC_PTP_BG_TASK_TICK_PERIOD));
    }
    TaskP_exit();
}
