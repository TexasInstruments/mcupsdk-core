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
 * \file  enet_cpsw_est_ts.c
 *
 * \brief This file contains the implementation of the EST timestamping APIs
 *        for the CPSW EST example app.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "enet_cpsw_est_common.h"
#include "enet_cpsw_est_ts.h"
#include "enet_cpsw_est_cfg.h"
#include <kernel/dpl/ClockP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Domain used to identify EST timestamp events */
#define ENET_APP_EST_TIMESTAMP_DOMAIN             (100U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EnetApp_computeEstIntvls(Enet_MacPort macPort,
                                     EnetApp_EstIntvls *intvls);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetApp_enableTimestamp(Enet_MacPort macPort)
{
    Enet_IoctlPrms prms;
    CpswMacPort_EstTimestampCfg timestampCfg;
    int32_t status;

    timestampCfg.macPort  = macPort;
    timestampCfg.mode     = CPSW_MACPORT_EST_TIMESTAMP_ALL;
    timestampCfg.domain   = ENET_APP_EST_TIMESTAMP_DOMAIN;
    ENET_IOCTL_SET_IN_ARGS(&prms, &timestampCfg);

    ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, CPSW_MACPORT_IOCTL_EST_ENABLE_TIMESTAMP, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to enable timestamping: %d\r\n", status);
    }
}

void EnetApp_disableTimestamp(Enet_MacPort macPort)
{
    Enet_IoctlPrms prms;
    EnetMacPort_GenericInArgs inArgs;
    int32_t status;

    inArgs.macPort  = macPort;
    ENET_IOCTL_SET_IN_ARGS(&prms, &inArgs);

    ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, CPSW_MACPORT_IOCTL_EST_DISABLE_TIMESTAMP, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to disable timestamping: %d\r\n", status);
    }
}

static void EnetApp_computeEstIntvls(Enet_MacPort macPort,
                                     EnetApp_EstIntvls *intvls)
{
    Enet_IoctlPrms prms;
    EnetTas_GenericInArgs inArgs;
    EnetTas_ControlList list;
    EnetApp_EstIntvl *intvl;
    uint64_t start = 0ULL;
    uint32_t i;
    int32_t status;

    inArgs.macPort = macPort;
    ENET_IOCTL_SET_INOUT_ARGS(&prms, &inArgs, &list);

    /* Get current operation list */
    ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, ENET_TAS_IOCTL_GET_OPER_LIST, &prms, status);
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to get TAS oper list: %d\r\n", status);
    }

    if (status == ENET_SOK)
    {
        memset(intvls, 0, sizeof(*intvls));

        /* Compute start and end time of each EST interval */
        for (i = 0U; i < list.listLength; i++)
        {
            intvls->numIntvls++;

            intvl = &intvls->intvl[i];
            intvl->gateMask = list.gateCmdList[i].gateStateMask;
            intvl->start = start;
            intvl->end   = start + list.gateCmdList[i].timeInterval - 1ULL;

            /* Truncation case: list gets truncated by an early end of cycle */
            if (intvl->end > list.cycleTime)
            {
                intvl->end = list.cycleTime;
                break;
            }

            start = intvl->end + 1U;
        }

        /* Stretch case: last window gets extended till end of cycle */
        if (intvl->end < list.cycleTime)
        {
            intvl->end = list.cycleTime;
        }

        intvls->cycleTime = list.cycleTime;

        /* Print time interval list */
        EnetAppUtils_print("Programmed EST time intervals:\r\n");
        for (i = 0U; i < intvls->numIntvls; i++)
        {
            intvl = &intvls->intvl[i];

            EnetAppUtils_print("Gate mask=%s%s%s%s%s%s%s%s (0x%02x), start=%llu ns, end=%llu ns\r\n",
                               ENET_IS_BIT_SET(intvl->gateMask, 7U) ? "o" : "C",
                               ENET_IS_BIT_SET(intvl->gateMask, 6U) ? "o" : "C",
                               ENET_IS_BIT_SET(intvl->gateMask, 5U) ? "o" : "C",
                               ENET_IS_BIT_SET(intvl->gateMask, 4U) ? "o" : "C",
                               ENET_IS_BIT_SET(intvl->gateMask, 3U) ? "o" : "C",
                               ENET_IS_BIT_SET(intvl->gateMask, 2U) ? "o" : "C",
                               ENET_IS_BIT_SET(intvl->gateMask, 1U) ? "o" : "C",
                               ENET_IS_BIT_SET(intvl->gateMask, 0U) ? "o" : "C",
                               intvl->gateMask, intvl->start, intvl->end);
        }
    }
}

void EnetApp_checkEstTimestamps(void)
{
    Enet_IoctlPrms prms;
    EnetTas_TasState state;
    CpswCpts_EstEventMatchParams matchParams;
    CpswCpts_EstEvent event;
    EnetApp_EstIntvls estIntvls;
    EnetApp_EstIntvl *intvl;
    Enet_MacPort macPort;
    uint64_t estfStartTime;
    uint64_t tsVal;
    uint8_t priority;
    uint32_t numTs;
    uint32_t i;
    uint32_t j;
    uint32_t k;
    int32_t status;
    bool test = false;
    bool overalFailStatus = 0;

    /* EST timestamps are read from CPTS FIFO and place into a software pool of size
     * ENET_CFG_CPSW_CPTS_EVENTS_POOL_SIZE which will hold the events from all MAC ports.
     * So that determines the maximum number of timestamps we can retrieve and check */
    numTs = ENETAPP_TEST_TX_PKT_CNT;
    if (numTs > (ENET_CFG_CPSW_CPTS_EVENTS_POOL_SIZE / gEnetApp.macPortNum))
    {
        numTs = ENET_CFG_CPSW_CPTS_EVENTS_POOL_SIZE / gEnetApp.macPortNum;
    }

    for (i = 0U; i < gEnetApp.macPortNum; i++)
    {
        macPort = gEnetApp.macPort[i];
        estfStartTime = gEnetApp.estfStartTime[i];

        /* There will not be EST timestamps if EST is disabled */
        state = EnetApp_getEstState(macPort);
        if (state != ENET_TAS_ENABLE)
        {
            continue;
        }

        EnetAppUtils_print("\r\nMAC %u: EST timestamp verification\r\n", ENET_MACPORT_ID(macPort));
        EnetAppUtils_print("-------------------------------------------\r\n");

        /* Compute the start and end times of each EST interval */
        EnetApp_computeEstIntvls(macPort, &estIntvls);

        /* Retrieve the timestamps of the sent packets */
        EnetAppUtils_print("\r\nRetrieved EST timestamps:\r\n");
        EnetAppUtils_print("Note: last %u timestamps per port are stored with current CPTS pool size (%u)\r\n",
                           numTs, ENET_CFG_CPSW_CPTS_EVENTS_POOL_SIZE);
        for (j = 0U; j < numTs; j++)
        {
            test = false;
            /* Get last EST timestamp event */
            matchParams.macPort = macPort;
            matchParams.domain  = ENET_APP_EST_TIMESTAMP_DOMAIN;
            ENET_IOCTL_SET_INOUT_ARGS(&prms, &matchParams, &event);

            ENET_IOCTL(gEnetApp.hEnet, gEnetApp.coreId, CPSW_CPTS_IOCTL_LOOKUP_EST_EVENT, &prms, status);
            if (status == ENET_SOK)
            {
                tsVal = event.tsVal;
                priority = event.priority;
            }
            else if (status == ENET_ENOTFOUND)
            {
                EnetAppUtils_print("No MAC %u EST timestamp events found\r\n", ENET_MACPORT_ID(macPort), status);
                break;
            }
            else
            {
                EnetAppUtils_print("Failed to lookup for EST timestamp events: %d\r\n", status);
                break;
            }

            /* Check if timestamp meets the programmed EST schedule */
            if (status == ENET_SOK)
            {
                /* Timestamp verification approach works only when ESTF start time is accurately known,
                 * which can be guaranteed only when an admin time base at future time was configured.
                 * This is true only when the example application sets up the default EST schedule */
                if (gEnetApp.usingDfltSched)
                {
                    tsVal = (tsVal - estfStartTime) % estIntvls.cycleTime;

                    for (k = 0U; k < estIntvls.numIntvls; k++)
                    {
                        intvl = &estIntvls.intvl[k];

                        if ((ENET_BIT(priority) & intvl->gateMask) != 0U)
                        {
                            /* Check if timestamp is within the interval boundaries, if not let it
                             * check other intervals in case of multiple intervals matching packet's priority */
                            test = ((tsVal >= intvl->start) && (tsVal <= intvl->end));
                            EnetAppUtils_print("MAC Port %u: packet with priority %u timestamp %llu (norm %llu) "
                                               "interval (%llu, %llu) : %s\r\n",
                                               ENET_MACPORT_ID(macPort),
                                               priority, event.tsVal, tsVal,
                                               intvl->start, intvl->end,
                                               test ? "PASS" : "FAIL");

                            if (test)
                            {
                                break;
                            }
                        }
                    }
                }
                else
                {
                    EnetAppUtils_print("MAC Port %u: packet with priority %u timestamp %llu\r\n",
                                       ENET_MACPORT_ID(macPort), priority, event.tsVal);
                }
            }
            overalFailStatus |= !test;
        }
    }
    EnetAppUtils_print("EST Self Test Result: %s", overalFailStatus ? "FAIL" : "PASS");
}
