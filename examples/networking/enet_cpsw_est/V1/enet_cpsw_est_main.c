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
 * \file  enet_cpsw_est_main.c
 *
 * \brief This file contains the implementation of the CPSW EST example app.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "enet_cpsw_est_common.h"
#include "enet_cpsw_est_cfg.h"
#include "enet_cpsw_est_ts.h"
#include "enet_cpsw_est_dataflow.h"

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

static void EnetApp_showMenu(void);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Use this array to select the ports that will be used in the test */
static EnetApp_TestParams testParams =
{
    .enetType = ENET_CPSW_3G,
    .instId   = 0U,
    .portTestParams =
    {
        {
            .macPort = ENET_MAC_PORT_1,
            .tasControlList =
            {
                .baseTime    = 0ULL, /* will be updated later */
                .cycleTime   = 250000ULL,
                .gateCmdList =
                {
                    { .gateStateMask = ENET_TAS_GATE_MASK(1, 1, 0, 0, 0, 0, 0, 0), .timeInterval = 62500U },
                    { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 1, 1, 0, 0, 0, 0), .timeInterval = 62500U },
                    { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 1, 1, 0, 0), .timeInterval = 62500U },
                    { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 0, 0, 1, 1), .timeInterval = 62500U },
                },
                .listLength = 4U,
            },
        },
        {
            .macPort     = ENET_MAC_PORT_2,
            .tasControlList =
            {
                .baseTime    = 0ULL, /* will be updated later */
                .cycleTime   = 250000ULL,
                .gateCmdList =
                {
                    { .gateStateMask = ENET_TAS_GATE_MASK(1, 1, 0, 0, 0, 0, 0, 0), .timeInterval = 62500U },
                    { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 1, 1, 0, 0, 0, 0), .timeInterval = 62500U },
                    { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 1, 1, 0, 0), .timeInterval = 62500U },
                    { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 0, 0, 1, 1), .timeInterval = 62500U },
                },
                .listLength = 4U,
            },
        },
    },
    .macPortNum = 1U,
};

static EnetTas_ControlList testLists[] =
{
    /* Admin list #1 (stretch) */
    {
        .baseTime    = 0ULL,
        .cycleTime   = 250000ULL,
        .gateCmdList =
        {
            { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 0, 0, 0, 0), .timeInterval =  12144U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 1, 0, 0, 0, 0, 0, 0), .timeInterval =  12144U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 1, 0, 0, 0, 0, 0), .timeInterval =  12144U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 1, 0, 0, 0, 0), .timeInterval =  12144U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 1, 0, 0, 0), .timeInterval =  12144U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 0, 1, 0, 0), .timeInterval =  12144U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 0, 0, 1, 0), .timeInterval =  12144U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 0, 0, 0, 1), .timeInterval =  12144U }, /* stretch */
        },
        .listLength = 8U,
    },
    /* Admin list #2 (truncate) */
    {
        .baseTime    = 0ULL,
        .cycleTime   = 250000ULL,
        .gateCmdList =
        {
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 0, 0, 0, 0), .timeInterval = 125000U },
            { .gateStateMask = ENET_TAS_GATE_MASK(1, 1, 0, 0, 0, 0, 0, 0), .timeInterval = 100000U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 1, 1, 0, 0, 0, 0), .timeInterval = 100000U }, /* truncate */
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 1, 1, 0, 0), .timeInterval = 100000U }, /* truncate */
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 0, 0, 1, 1), .timeInterval = 100000U }, /* truncate */
        },
        .listLength = 5U,
    },
    /* Admin list #3 (guard band) */
    {
        .baseTime    = 0ULL,
        .cycleTime   = 250000ULL,
        .gateCmdList =
        {
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 0, 0, 0, 0), .timeInterval =  97152U }, /* guard band */
            { .gateStateMask = ENET_TAS_GATE_MASK(1, 1, 1, 1, 0, 0, 0, 0), .timeInterval = 131064U },
        },
        .listLength = 2U,
    },
    /* Admin list #4 (single priority) */
    {
        .baseTime    = 0ULL,
        .cycleTime   = 250000ULL,
        .gateCmdList =
        {
            { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 0, 0, 0, 0), .timeInterval = 12144U }, /* single priority */
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 0, 0, 0, 0), .timeInterval = 12144U },
        },
        .listLength = 2U,
    },
    /* Admin list #5 (admin basetime in future) */
    {
        .baseTime    = 1413865740000000000LL, /* admin basetime in the future */
        .cycleTime   = 250000ULL,
        .gateCmdList =
        {
            { .gateStateMask = ENET_TAS_GATE_MASK(1, 1, 0, 0, 0, 0, 0, 0), .timeInterval = 62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 1, 1, 0, 0, 0, 0), .timeInterval = 62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 1, 1, 0, 0), .timeInterval = 62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 0, 0, 1, 1), .timeInterval = 62500U },
        },
        .listLength = 4U,
    },
    /* Admin list #6 (unsupported list - new cycle time) */
    {
        .baseTime    = 0ULL,
        .cycleTime   = 500000ULL, /* cycle time reconfiguration not supported */
        .gateCmdList =
        {
            { .gateStateMask = ENET_TAS_GATE_MASK(1, 0, 0, 0, 0, 0, 0, 0), .timeInterval =  62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 1, 0, 0, 0, 0, 0, 0), .timeInterval =  62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 1, 0, 0, 0, 0, 0), .timeInterval =  62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 1, 0, 0, 0, 0), .timeInterval =  62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 1, 0, 0, 0), .timeInterval =  62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 0, 1, 0, 0), .timeInterval =  62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 0, 0, 1, 0), .timeInterval =  62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 0, 0, 0, 1), .timeInterval =  62500U },
        },
        .listLength = 8U,
    },
    /* Admin list #7 (unsupported list - too small interval) */
    {
        .baseTime    = 0ULL,
        .cycleTime   = 250000ULL,
        .gateCmdList =
        {
            { .gateStateMask = ENET_TAS_GATE_MASK(1, 1, 0, 0, 0, 0, 0, 0), .timeInterval =  62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 1, 1, 0, 0, 0, 0), .timeInterval =     64U }, /* too small */
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 1, 1, 0, 0), .timeInterval =  62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 0, 0, 1, 1), .timeInterval =  62500U },
        },
        .listLength = 4U,
    },
    /* Admin list #8 (invalid list - zero length) */
    {
        .baseTime    = 0ULL,
        .cycleTime   = 250000ULL,
        .gateCmdList =
        {
            { .gateStateMask = ENET_TAS_GATE_MASK(1, 1, 0, 0, 0, 0, 0, 0), .timeInterval =  62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 1, 1, 0, 0, 0, 0), .timeInterval =  62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 1, 1, 0, 0), .timeInterval =  62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 0, 0, 1, 1), .timeInterval =  62500U },
        },
        .listLength = 0U, /* invalid */
    },
    /* Admin list #9 (invalid list - zero interval) */
    {
        .baseTime    = 0ULL,
        .cycleTime   = 250000ULL,
        .gateCmdList =
        {
            { .gateStateMask = ENET_TAS_GATE_MASK(1, 1, 0, 0, 0, 0, 0, 0), .timeInterval =  62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 1, 1, 0, 0, 0, 0), .timeInterval =  62500U },
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 1, 1, 0, 0), .timeInterval =      0U }, /* invalid */
            { .gateStateMask = ENET_TAS_GATE_MASK(0, 0, 0, 0, 0, 0, 1, 1), .timeInterval =  62500U },
        },
        .listLength = 4U,
    },
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetApp_mainTask(void *args)
{
    char option;
    uint64_t tsVal;
    uint32_t i;
    int32_t status = ENET_SOK;
    Enet_Type enetType;
    uint32_t instId;

    Drivers_open();
    Board_driversOpen();

    DebugP_log("==========================\r\n");
    DebugP_log("       CPSW EST Test      \r\n");
    DebugP_log("==========================\r\n");

    EnetApp_getEnetInstInfo(CONFIG_ENET_CPSW0, &enetType,
                            &instId);

    /* Initialize test config */
    memset(&gEnetApp, 0, sizeof(gEnetApp));
    gEnetApp.run = true;
    gEnetApp.enableTs = false;

    /* Copy test params */
    gEnetApp.enetType    = testParams.enetType;
    gEnetApp.instId      = testParams.instId;
    gEnetApp.macPortNum  = testParams.macPortNum;

    for (i = 0U; i < gEnetApp.macPortNum; i++)
    {
        gEnetApp.macPort[i] = testParams.portTestParams[i].macPort;
        memcpy(&gEnetApp.tasControlList[i],
               &testParams.portTestParams[i].tasControlList,
               sizeof(EnetTas_ControlList));
    }

    /* Init driver */
    EnetApp_init();

    /* Open CPSW peripheral */
    status = EnetApp_open();
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to open peripherals: %d\r\n", status);
    }

    if (status == ENET_SOK)
    {
        /* Wait for user input to exit the test */
        EnetApp_showMenu();
        while (true)
        {
            option = ' ';
            status = DebugP_scanf("%c", &option);
            if (option == 'x')
            {
                EnetAppUtils_print("Stopping...\r\n");
                gEnetApp.run = false;
                break;
            }
            else if (option == 'c')
            {
                tsVal = EnetApp_getCurrentTime();
                EnetAppUtils_print("Current time is: %llu\r\n", tsVal);
            }
            else if (option == 't')
            {
                gEnetApp.enableTs = !gEnetApp.enableTs;
                EnetAppUtils_print("\n%s timestamp printing\r\n", (gEnetApp.enableTs ? "Enable" : "Disable"));
                for (i = 0U; i < gEnetApp.macPortNum; i++)
                {
                    if (gEnetApp.enableTs)
                    {
                        EnetApp_enableTimestamp(gEnetApp.macPort[i]);
                    }
                    else
                    {
                        EnetApp_disableTimestamp(gEnetApp.macPort[i]);
                    }
                }
            }
            else if (option == 's')
            {
                EnetApp_printStats();
            }
            else if (option == 'r')
            {
                EnetApp_resetStats();
            }
            else if (option == 'E')
            {
                for (i = 0U; i < gEnetApp.macPortNum; i++)
                {
                    EnetApp_setEstState(gEnetApp.macPort[i], ENET_TAS_ENABLE);
                }
            }
            else if (option == 'D')
            {
                gEnetApp.usingDfltSched = false;

                for (i = 0U; i < gEnetApp.macPortNum; i++)
                {
                    EnetApp_setEstState(gEnetApp.macPort[i], ENET_TAS_DISABLE);
                }
            }
            else if (option == 'R')
            {
                gEnetApp.usingDfltSched = false;

                for (i = 0U; i < gEnetApp.macPortNum; i++)
                {
                    EnetApp_setEstState(gEnetApp.macPort[i], ENET_TAS_RESET);
                }
            }
            else if (option == 'T')
            {
                EnetApp_txTest();
                if (gEnetApp.enableTs)
                {
                    EnetApp_checkEstTimestamps();
                }
            }
            else if ((option >= '1') && (option <= '9'))
            {
                gEnetApp.usingDfltSched = false;

                for (i = 0U; i < gEnetApp.macPortNum; i++)
                {
                    EnetApp_setAdminList(gEnetApp.macPort[i], &testLists[option - '1']);
                }
            }
            else if ((option == 'h') || (option == 'H'))
            {
                EnetApp_showMenu();
            }
            else
            {
                EnetAppUtils_print("Invalid option, try again...\r\n");
                EnetApp_showMenu();
            }

            TaskP_yield();
        }

        /* Print statistics */
        EnetApp_printStats();

        /* Wait until RX task has exited */
        EnetAppUtils_print("Waiting for tasks to exit\r\n");
        SemaphoreP_post(&gEnetApp.rxSemObj);
        SemaphoreP_pend(&gEnetApp.rxDoneSemObj, SystemP_WAIT_FOREVER);

        EnetAppUtils_print("All tasks have exited\r\n");

    }

    /* Close peripheral */
    EnetApp_close();

    /* Deinit driver */
    EnetApp_deinit();
}

static void EnetApp_showMenu(void)
{
    EnetAppUtils_print("\r\nCPSW EST Test Menu:\r\n");
    EnetAppUtils_print(" EST control list tests:\r\n");
    EnetAppUtils_print(" 'T'  -  Send test packets from host port\r\n");
    EnetAppUtils_print(" '1'  -  Set admin list #1 (stretch)\r\n");
    EnetAppUtils_print(" '2'  -  Set admin list #2 (truncate)\r\n");
    EnetAppUtils_print(" '3'  -  Set admin list #3 (guard band)\r\n");
    EnetAppUtils_print(" '4'  -  Set admin list #4 (single priority)\r\n");
    EnetAppUtils_print(" '5'  -  Set admin list #5 (admin basetime in future)\r\n");
    EnetAppUtils_print(" '6'  -  Set admin list #6 (unsupported list - new cycle time)\r\n");
    EnetAppUtils_print(" '7'  -  Set admin list #7 (unsupported list - too small interval)\r\n");
    EnetAppUtils_print(" '8'  -  Set admin list #8 (invalid list - zero length)\r\n");
    EnetAppUtils_print(" '9'  -  Set admin list #9 (invalid list - zero interval)\r\n");

    EnetAppUtils_print(" EST state:\r\n");
    EnetAppUtils_print(" 'E'  -  Set EST state to 'ENABLE'\r\n");
    EnetAppUtils_print(" 'D'  -  Set EST state to 'DISABLE'\r\n");
    EnetAppUtils_print(" 'R'  -  Set EST state to 'RESET'\r\n");

    EnetAppUtils_print(" Others:\r\n");
    EnetAppUtils_print(" 'c'  -  Get current time\r\n");
    EnetAppUtils_print(" 't'  -  Toggle printing timestamps\r\n");
    EnetAppUtils_print(" 's'  -  Print statistics\r\n");
    EnetAppUtils_print(" 'r'  -  Reset statistics\r\n");
    EnetAppUtils_print(" 'x'  -  Stop the test\r\n");
    EnetAppUtils_print(" 'h'  -  Show this menu\r\n");

    EnetAppUtils_print("\r\n");
}
