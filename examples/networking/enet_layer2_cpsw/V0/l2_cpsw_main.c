/*
 *  Copyright (c) Texas Instruments Incorporated 2021
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
 * \file  l2_cpsw_main.c
 *
 * \brief This file contains the implementation of the Enet L2 cpsw example.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "l2_cpsw_common.h"
#include "l2_cpsw_cfg.h"
#include "l2_cpsw_dataflow.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Use this array to select the ports that will be used in the test */
static EnetApp_TestParams testParams[] =
{
    { ENET_CPSW_3G,       0U, ENET_MAC_PORT_1, "cpsw-3g", },
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void EnetApp_mainTask(void *args)
{
    char option;
    uint32_t i;
    int32_t status;

	Drivers_open();
    Board_driversOpen();

    DebugP_log("==========================\r\n");
    DebugP_log("     Layer 2 CPSW Test    \r\n");
    DebugP_log("==========================\r\n");

	/* Initialize test config */
    memset(&gEnetApp, 0, sizeof(gEnetApp));
    gEnetApp.run = true;

    gEnetApp.numPerCtxts = ENET_ARRAYSIZE(testParams);

    for (i = 0U; i < gEnetApp.numPerCtxts; i++)
    {
        gEnetApp.perCtxt[i].enetType = testParams[i].enetType;
        gEnetApp.perCtxt[i].instId   = testParams[i].instId;
        gEnetApp.perCtxt[i].name     = testParams[i].name; /* shallow copy */
        gEnetApp.perCtxt[i].macPort  = testParams[i].macPort;
    }

    /* Init driver */
    status = EnetApp_init();
    if (status != ENET_SOK)
    {
        EnetAppUtils_print("Failed to initialize l2 cpsw test: %d\r\n", status);
    }

    /* Open all peripherals */
    if (status == ENET_SOK)
    {
        status = EnetApp_open(gEnetApp.perCtxt, gEnetApp.numPerCtxts);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open peripherals: %d\r\n", status);
        }
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
            else if (option == 's')
            {
                EnetApp_printStats(gEnetApp.perCtxt, gEnetApp.numPerCtxts);
            }
            else if (option == 'r')
            {
                EnetApp_resetStats(gEnetApp.perCtxt, gEnetApp.numPerCtxts);
            }
            else if (option == 'm')
            {
                EnetApp_showMacAddrs(gEnetApp.perCtxt, gEnetApp.numPerCtxts);
            }
            else
            {
                EnetAppUtils_print("Invalid option, try again...\r\n");
                EnetApp_showMenu();
            }
            TaskP_yield();
        }

        /* Print statistics */
        EnetApp_printStats(gEnetApp.perCtxt, gEnetApp.numPerCtxts);

        /* Wait until RX tasks have exited */
        for (i = 0U; i < gEnetApp.numPerCtxts; i++)
        {
            EnetAppUtils_print("Waiting for RX task %u to exit\r\n", i+1);
            SemaphoreP_post(&gEnetApp.perCtxt[i].rxSemObj);
            SemaphoreP_pend(&gEnetApp.perCtxt[i].rxDoneSemObj, SystemP_WAIT_FOREVER);
        }

        EnetAppUtils_print("All RX tasks have exited\r\n");
    }

    /* Close all peripherals */
    EnetApp_close(gEnetApp.perCtxt, gEnetApp.numPerCtxts);

    /* Deinit driver */
    EnetApp_deinit();
}