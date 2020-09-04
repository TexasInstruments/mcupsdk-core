/*
 *  Copyright (c) Texas Instruments Incorporated 2019
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

/**
 *  \file udma_test_bug.c
 *
 *  \brief UDMA test case file to test bugs filed.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <udma_test.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t udmaTestBugTcPDK_4654(UdmaTestTaskObj *taskObj)
{
    int32_t                 retVal = UDMA_SOK;
    uint32_t                instId;
    Udma_DrvHandle          drvHandle;
    Udma_EventObject        eventObj;
    Udma_EventHandle        eventHandle = &eventObj;
    Udma_EventPrms          eventPrms;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: PDK-4654 Bug Testcase: Deinit RM check ::\r\n", taskObj->taskId);

    /* Deinit already init driver */
    retVal = udmaTestDeinitDriver(taskObj->testObj);
    if(UDMA_SOK != retVal)
    {
        GT_0trace(taskObj->traceMask, GT_ERR, " UDMA deinit failed!!\n");
    }

    if(UDMA_SOK == retVal)
    {
        /* Do a fresh init */
        retVal = udmaTestInitDriver(taskObj->testObj);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " UDMA re-init failed!!\n");
        }

        if(UDMA_SOK == retVal)
        {
            for(instId = UDMA_INST_ID_START; instId <= UDMA_INST_ID_MAX; instId++)
            {
                drvHandle = &taskObj->testObj->drvObj[instId];

                /* Alloc VINTR - By registering Master event in Shared mode */
                UdmaEventPrms_init(&eventPrms);
                eventPrms.eventType         = UDMA_EVENT_TYPE_MASTER;
                eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
                eventPrms.masterEventHandle = NULL;
                retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Event register/alloc failed!!\n");
                }

                if(UDMA_SOK == retVal)
                {
                    /* Deinit with a resource not-freed */
                    retVal = Udma_deinit(drvHandle);
                    if(UDMA_SOK == retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR,
                            " UDMA passed when it should have failed!!\n");
                    }
                    else
                    {
                        retVal = Udma_eventUnRegister(eventHandle);
                        if(UDMA_SOK != retVal)
                        {
                            GT_0trace(taskObj->traceMask, GT_ERR,
                                " Event unregister/free failed!!\n");
                        }
                    }
                }

                if(UDMA_SOK != retVal)
                {
                    break;
                }
            }

            /* This deinit should pass */
            retVal += udmaTestDeinitDriver(taskObj->testObj);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " UDMA deinit failed!!\n");
            }
        }

        /* Init before ending the testcase */
        retVal = udmaTestInitDriver(taskObj->testObj);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " UDMA re-init failed!!\n");
        }
    }

    return (retVal);
}
