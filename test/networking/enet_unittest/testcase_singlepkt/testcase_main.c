/*
 *  Copyright (c) Texas Instruments Incorporated 2020
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
 * \file  testcase_main.c
 *
 * \brief This file contains the main task of the Testcases.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "testcase_common_cfg.h"
#include "testcase_singlepkt_tc1.h"
#include "testcase_singlepkt_tc2.h"
#include "testcase_singlepkt_tc3.h"
#include "testcase_cptsrxts.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define TOTAL_TESTCASES     18

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void TestApp_runTestcase(int8_t option);

static void Testcase_init();

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static const char TestCaseMenu[] =
{
    "\r\n Testcases:"
    " \r\n 0: Run all testcases"
    " \r\n 1: Internal MAC loopback with Single Packet API where Number of Scatter Segment = 1 where txNumPkt=rxNumPkt=1"
    " \r\n 2: Internal MAC loopback with Single Packet API where Number of Scatter Segment = 2 where txNumPkt=rxNumPkt=1"
    " \r\n 3: Internal MAC loopback with Single Packet API where Number of Scatter Segment = 3 where txNumPkt=rxNumPkt=1"
    " \r\n 4: Internal MAC loopback with Single Packet API where Number of Scatter Segment = 4 where txNumPkt=rxNumPkt=1"
    " \r\n 5: External PHY loopback with Single Packet API where Number of Scatter Segment = 1 where txNumPkt=rxNumPkt=1"
    " \r\n 6: External PHY loopback with Single Packet API where Number of Scatter Segment = 2 where txNumPkt=rxNumPkt=1"
    " \r\n 7: External PHY loopback with Single Packet API where Number of Scatter Segment = 3 where txNumPkt=rxNumPkt=1"
    " \r\n 8: External PHY loopback with Single Packet API where Number of Scatter Segment = 4 where txNumPkt=rxNumPkt=1"
    " \r\n 9: Internal PHY loopback with Single Packet API where Number of Scatter Segment = 1 where txNumPkt=rxNumPkt=4"
    " \r\n 10: External PHY loopback with Single Packet API where Number of Scatter Segment = 1 where txNumPkt=rxNumPkt=4"
    " \r\n 11: Internal PHY loopback with Single Packet API where Number of Scatter Segment = 2 where txNumPkt=rxNumPkt=4"
    " \r\n 12: External PHY loopback with Single Packet API where Number of Scatter Segment = 2 where txNumPkt=rxNumPkt=4"
    " \r\n 13: Internal PHY loopback with Single Packet API where Number of Scatter Segment = 3 where txNumPkt=rxNumPkt=4"
    " \r\n 14: External PHY loopback with Single Packet API where Number of Scatter Segment = 3 where txNumPkt=rxNumPkt=4"
    " \r\n 15: Internal PHY loopback with Single Packet API where Number of Scatter Segment = 4 where txNumPkt=rxNumPkt=4"
    " \r\n 16: External PHY loopback with Single Packet API where Number of Scatter Segment = 4 where txNumPkt=rxNumPkt=4"
    " \r\n 17: Internal PHY loopback with Single Packet API as well as submit/retrieve Queue APIs together where Number of Scatter Segment = 1 where txNumPkt=rxNumPkt=16"
    " \r\n 18: External PHY loopback to check the CPTS Rx pkt time stamping"
    " \r\n"
    " \r\n Enter option: "
};

/* Test App object */
TestApp_Obj gTestApp;

/* Test App config object */
TestCfg_Obj gTestCfg;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetLpbk_mainTask(void *args)
{
    uint32_t i;
    Enet_MacPort macPortList[ENET_MAC_PORT_NUM];
    uint8_t numMacPorts;
    int8_t option = -1;
    uint8_t exitTestcase = 0U;

    Drivers_open();
    Board_driversOpen();

    /* Initialize test app config */
    memset(&gTestApp, 0, sizeof(gTestApp));
    memset(&gTestCfg, 0, sizeof(gTestCfg));
    gTestApp.exitFlag = false;

    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &gTestApp.enetType,
                                &gTestApp.instId);

    EnetApp_getEnetInstMacInfo(gTestApp.enetType,
                               gTestApp.instId,
                               macPortList,
                               &numMacPorts);

    EnetAppUtils_assert(numMacPorts == 1);
    gTestApp.macPort          = macPortList[0];

    Testcase_init();

    while(!exitTestcase)
    {
        DebugP_log("%s", TestCaseMenu);
        DebugP_scanf("%d", &option);
        if(option == 0)
        {
            for(i = 1; i <= TOTAL_TESTCASES; i++)
            {
                TestApp_runTestcase(i);
            }
        }
        else
        {
            TestApp_runTestcase(option);
        }
    }
    return;
}

static void TestApp_runTestcase(int8_t option)
{
    uint32_t i;
    uint32_t status = ENET_SOK;

    switch(option)
    {
        case 1U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_MAC;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_LOOPBACK_ID;
            gTestCfg.txScatterSeg     = 1;
            gTestCfg.rxScatterSeg     = 1;
            gTestCfg.rxNumPkt         = 1;
            gTestCfg.txNumPkt         = 1;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 2U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_MAC;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_LOOPBACK_ID;
            gTestCfg.txScatterSeg     = 2;
            gTestCfg.rxScatterSeg     = 2;
            gTestCfg.rxNumPkt         = 1;
            gTestCfg.txNumPkt         = 1;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 3U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_MAC;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_LOOPBACK_ID;
            gTestCfg.txScatterSeg     = 3;
            gTestCfg.rxScatterSeg     = 3;
            gTestCfg.rxNumPkt         = 1;
            gTestCfg.txNumPkt         = 1;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 4U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_MAC;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_LOOPBACK_ID;
            gTestCfg.txScatterSeg     = 4;
            gTestCfg.rxScatterSeg     = 4;
            gTestCfg.rxNumPkt         = 1;
            gTestCfg.txNumPkt         = 1;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 5U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_PHY;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_CPB_ID;
            gTestCfg.txScatterSeg     = 1;
            gTestCfg.rxScatterSeg     = 1;
            gTestCfg.rxNumPkt         = 1;
            gTestCfg.txNumPkt         = 1;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 6U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_PHY;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_CPB_ID;
            gTestCfg.txScatterSeg     = 2;
            gTestCfg.rxScatterSeg     = 2;
            gTestCfg.rxNumPkt         = 1;
            gTestCfg.txNumPkt         = 1;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 7U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_PHY;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_CPB_ID;
            gTestCfg.txScatterSeg     = 3;
            gTestCfg.rxScatterSeg     = 3;
            gTestCfg.rxNumPkt         = 1;
            gTestCfg.txNumPkt         = 1;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 8U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_PHY;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_CPB_ID;
            gTestCfg.txScatterSeg     = 4;
            gTestCfg.rxScatterSeg     = 4;
            gTestCfg.rxNumPkt         = 1;
            gTestCfg.txNumPkt         = 1;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 9U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_MAC;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_LOOPBACK_ID;
            gTestCfg.txScatterSeg     = 1;
            gTestCfg.rxScatterSeg     = 1;
            gTestCfg.rxNumPkt         = 4;
            gTestCfg.txNumPkt         = 4;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase2();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 10U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_PHY;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_CPB_ID;
            gTestCfg.txScatterSeg     = 1;
            gTestCfg.rxScatterSeg     = 1;
            gTestCfg.rxNumPkt         = 4;
            gTestCfg.txNumPkt         = 4;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase2();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 11U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_MAC;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_LOOPBACK_ID;
            gTestCfg.txScatterSeg     = 2;
            gTestCfg.rxScatterSeg     = 2;
            gTestCfg.rxNumPkt         = 4;
            gTestCfg.txNumPkt         = 4;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase2();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 12U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_PHY;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_CPB_ID;
            gTestCfg.txScatterSeg     = 2;
            gTestCfg.rxScatterSeg     = 2;
            gTestCfg.rxNumPkt         = 4;
            gTestCfg.txNumPkt         = 4;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase2();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 13U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_MAC;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_LOOPBACK_ID;
            gTestCfg.txScatterSeg     = 3;
            gTestCfg.rxScatterSeg     = 3;
            gTestCfg.rxNumPkt         = 4;
            gTestCfg.txNumPkt         = 4;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase2();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 14U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_PHY;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_CPB_ID;
            gTestCfg.txScatterSeg     = 3;
            gTestCfg.rxScatterSeg     = 3;
            gTestCfg.rxNumPkt         = 4;
            gTestCfg.txNumPkt         = 4;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase2();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 15U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_MAC;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_LOOPBACK_ID;
            gTestCfg.txScatterSeg     = 4;
            gTestCfg.rxScatterSeg     = 4;
            gTestCfg.rxNumPkt         = 4;
            gTestCfg.txNumPkt         = 4;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase2();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 16U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_PHY;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_CPB_ID;
            gTestCfg.txScatterSeg     = 4;
            gTestCfg.rxScatterSeg     = 4;
            gTestCfg.rxNumPkt         = 4;
            gTestCfg.txNumPkt         = 4;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase2();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 17U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_MAC;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_LOOPBACK_ID;
            gTestCfg.txScatterSeg     = 1;
            gTestCfg.rxScatterSeg     = 1;
            gTestCfg.rxNumPkt         = 16;
            gTestCfg.txNumPkt         = 16;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_SinglePktTestcase3();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
                EnetAppUtils_print("All tests have passed!!\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        case 18U:
            gTestApp.testLoopBackType = LOOPBACK_TYPE_PHY;
            gTestApp.macMode          = RGMII;
            gTestApp.boardId          = ENETBOARD_CPB_ID;
            gTestCfg.txScatterSeg     = 1;
            gTestCfg.rxScatterSeg     = 1;
            gTestCfg.rxNumPkt         = 8;
            gTestCfg.txNumPkt         = 8;
            for (i = 0U; i < ENETLPBK_NUM_ITERATION; i++)
            {
                EnetAppUtils_print("=============================\r\n");
                EnetAppUtils_print(" TestCase %d: Iteration %u \r\n", option, i + 1);
                EnetAppUtils_print("=============================\r\n");

                /* Run the loopback test */
                status = TestApp_CptsRxTsTestcase();
                EnetAppUtils_assert(status == ENET_SOK);

                /* Sleep at end of each iteration to allow idle task to delete all terminated tasks */
                ClockP_usleep(1000);
            }

            if (status == ENET_SOK)
            {
                EnetAppUtils_print("Loopback application completed\r\n");
            }
            else
            {
                EnetAppUtils_print("Loopback application failed to complete\r\n");
            }
            break;
        default:
           DebugP_log(" \r\n Invalid option... Try Again!!!\r\n");
           break;
    }
    TaskP_yield();
}

static void Testcase_init()
{
    gTestApp.testLoopBackType = LOOPBACK_TYPE_MAC;
    gTestApp.macMode          = RGMII;
    gTestApp.boardId          = ENETBOARD_LOOPBACK_ID;
    gTestCfg.txScatterSeg     = 1;
    gTestCfg.rxScatterSeg     = 1;
    gTestCfg.rxNumPkt         = 1;
    gTestCfg.txNumPkt         = 1;
}
