/*
 *  Copyright (c) Texas Instruments Incorporated 2023
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
 * \file  cpsw_fast_startup_main.c
 *
 * \brief This file contains the implementation of the Enet CPSW Fast Startup example
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "cpsw_fast_startup_common.h"
#include "cpsw_fast_startup_cfg.h"
#include "cpsw_fast_startup_dataflow.h"
#include "enet_profiler.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void EnetApp_handleEvent(const uint32_t eventMask);

static uint32_t EnetApp_receiveEvents(EventP_Object* pEvent);

static void EnetApp_terminateApp();

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/* Use this array to select the ports that will be used in the test */
static EnetApp_TestParams testParams[] =
{
    { ENET_CPSW_3G,       0U, { ENET_MAC_PORT_1, ENET_MAC_PORT_2 }, 2U, "cpsw-3g", },
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetApp_main(void *args)
{
    uint32_t i;
    int32_t status = ENET_SOK;
    
    PROFILE_TIME(ClockP_getTimeUsec());

    Drivers_open();
    Board_driversOpen();

    EnetAppUtils_print("==========================\r\n");
    EnetAppUtils_print("    CPSW Fast Startup     \r\n");
    EnetAppUtils_print("==========================\r\n");

    /* Initialize test config */
    memset(&gEnetApp, 0, sizeof(gEnetApp));
    gEnetApp.run = true;
    
    gEnetApp.enetType = testParams[0U].enetType;
    gEnetApp.instId   = testParams[0U].instId;
    gEnetApp.name     = testParams[0U].name; /* shallow copy */
    gEnetApp.macPortNum = testParams[0U].macPortNum;
    
    for (i = 0; i < gEnetApp.macPortNum; i++)
    {
        gEnetApp.macPort[i]  = testParams[0U].macPort[i];
    }

    /* Create Global Event Object */
    status = EventP_construct(&gEnetApp.appEvents);
    DebugP_assert(SystemP_SUCCESS == status);
    
    /* Init driver */
    status = EnetApp_init();

    /* Open all peripherals */
    status = EnetApp_open();
    DebugP_assert(SystemP_SUCCESS == status);

    PROFILE_TIME(ClockP_getTimeUsec());
    
    /* Wait for link up */
    EnetApp_initPhyStateHandlerTask(&gEnetApp.appEvents);
    status = EnetApp_waitForLinkUp();
    DebugP_assert(SystemP_SUCCESS == status);

    /* Do packet transmission */
    EnetApp_postTxEvent((void*) &gEnetApp.appEvents);

    while (true)
    {
        uint32_t txRetrievePktCnt = EnetApp_retrieveFreeTxPkts();
        if (txRetrievePktCnt != 0)
        {
            EnetApp_postTxEvent((void*) &gEnetApp.appEvents);
        }
        
        const uint32_t recvdEventsMask = EnetApp_receiveEvents(&gEnetApp.appEvents);

        if (recvdEventsMask != AppEventId_NONE)
        {
            EnetApp_handleEvent(recvdEventsMask);
            
            if (gEnetApp.run == false)
            {
                EnetAppUtils_print("Terminating App\r\n");
                break;
            }
        }
    }

    /* Close all peripherals */
    EnetApp_close();

    /* Deinit driver */
    EnetApp_deinit();
}

static uint32_t EnetApp_receiveEvents(EventP_Object* pEvent)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t recvdEventsMask = AppEventId_NONE;

    status = EventP_waitBits(pEvent,
                        AppEventId_ANY_EVENT, // bitsToWaitFor
                        1,
                        0,
                        SystemP_NO_WAIT,
                        &recvdEventsMask);

    if ((status != SystemP_SUCCESS) && (status != SystemP_TIMEOUT))
    {
        EnetAppUtils_print("Failed to receive Event handle\r\n");
        EnetAppUtils_assert(false);
    }

    return recvdEventsMask;
}


static void EnetApp_handleEvent(const uint32_t eventMask)
{

    if (AppEventId_RXPKT & eventMask)
    {
        EnetApp_handleRxPkt();
    }
    
    if (AppEventId_TXPKT & eventMask)
    {
        /* Do a single packet transmission */
        EnetDma_PktQ TxPktQueue;
        EnetQueue_initQ(&TxPktQueue);
        EnetApp_preparePktQ(&TxPktQueue);
        EnetApp_sendPktQ(&TxPktQueue);
    }
    
    if (AppEventId_TERMINATE & eventMask)
    {
        EnetApp_terminateApp();
    }
    
    if (AppEventId_PERIODIC_POLL & eventMask)
    {
        EnetApp_phyStateHandler();
    }

    return;
}


static void EnetApp_terminateApp()
{
    DebugP_log("=====================================================\r\n");
    DebugP_log("   Fast Startup Profiling logs in Microseconds(us)    \r\n");
    DebugP_log("=====================================================\r\n");
    
    DebugP_log("Application start time: %d \r\n", gProfilerArr[0U][0U]); 
    DebugP_log("Enet-lld initialisation done time : %d\r\n", gProfilerArr[1U][0U]); 
    DebugP_log("Both ports linked up time: %d \r\n", gProfilerArr[3U][0U]); 

    DebugP_log("Time for First packet sent out from Port 1: %d \r\n", gProfilerArr[4U][0U]);
    DebugP_log("Time for First packet received on Port 2: %d \r\n", gProfilerArr[5U][0U]);
    
    DebugP_log("Application Terminating...\r\n"); 
    gEnetApp.run = false;
}