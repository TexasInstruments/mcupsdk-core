/*
 *  Copyright (c) Texas Instruments Incorporated 2024
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
 * \file  app_main.c
 *
 * \brief This file contains the implementation of the Enet Remote Connect example for main core.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include "FreeRTOS.h"
#include "task.h"
/* lwIP core includes */
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include <examples/lwiperf/lwiperf_example.h>

#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>

#include <drivers/sciclient.h>
#include <drivers/udma/udma_priv.h>
#include <enet.h>
#include <enet_cfg.h>
#include <include/core/enet_osal.h>
#include <include/core/enet_dma.h>
#include <include/per/cpsw.h>
#include <enet_apputils.h>
#include <enet_appmemutils.h>
#include <enet_appmemutils_cfg.h>
#include <enet_appboardutils.h>
/* SDK includes */
#include <enet_board.h>
#include "ti_board_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"
#include "ti_enet_config.h"
#include "ti_enet_open_close.h"
#include "ti_enet_lwipif.h"

#include "app_tcpserver.h"
#include "netif_common.h"
#include "app_control.h"
#include "app_netif.h"
#include "udp_iperf.h"
/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* UDP Iperf task should be highest priority task to ensure processed buffers
 * are freed without delay so that we get maximum throughput for
 * UDP Iperf.
 */
#define UDP_IPERF_THREAD_PRIO  (14U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct EnetApp_AppEnetInfo_s
{
    /* Peripheral type */
    Enet_Type enetType;

    /* Peripheral instance */
    uint32_t instId;

    /* MAC ports List to use for the above EnetType & InstId*/
    uint8_t     numMacPort;

    /* Num MAC ports to use for the above EnetType & InstId*/
    Enet_MacPort macPortList[ENET_SYSCFG_MAX_MAC_PORTS];
} EnetApp_AppEnetInfo;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void appMain(void *args);

static void EnetMp_remoteCoreTask(void *args);

static void App_printCpuLoad();

int32_t AppCtrl_addMacAddr2fbd(Icssg_MacAddr mac);

int32_t AppCtrl_addMcastAddr(Icssg_MacAddr mac);

int32_t AppCtrl_delMcastAddr(Icssg_MacAddr mac);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static EnetApp_AppEnetInfo gEnetAppParams[ENET_SYSCFG_MAX_ENET_INSTANCES];

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void appMain(void *args)
{
    int32_t status = 1; //ENET_SOK


    Drivers_open();
    Board_driversOpen();

    DebugP_log("=================================\r\n");
    DebugP_log("  ICSS LWIP TCP ECHO SERVER 0-0 \r\n");
    DebugP_log("=================================\r\n");

    /* Read MAC Port details and enable clock for each ENET instance */
    for (uint32_t enetInstIdx = 0; enetInstIdx < ENET_SYSCFG_MAX_ENET_INSTANCES; enetInstIdx++)
    {
        EnetApp_AppEnetInfo* pEnetInstInfo = &gEnetAppParams[enetInstIdx];
        EnetApp_getEnetInstInfo(CONFIG_ENET_ICSS0 + enetInstIdx, &pEnetInstInfo->enetType, &pEnetInstInfo->instId);
        EnetApp_getEnetInstMacInfo(pEnetInstInfo->enetType,
                                   pEnetInstInfo->instId,
                                   &pEnetInstInfo->macPortList[0],
                                   &pEnetInstInfo->numMacPort);
        EnetAppUtils_enableClocks(pEnetInstInfo->enetType, pEnetInstInfo->instId);
    }

    /* Open ENET driver for each ENET instance */
    EnetApp_driverInit();
    /*! Uncomment this line to debug issues using CCS */
//        while(status)
//        {
//            DebugP_log("%d", status);
//        }
    for(uint32_t enetInstIdx = 0; enetInstIdx < ENET_SYSCFG_MAX_ENET_INSTANCES; enetInstIdx++)
    {
        status = EnetApp_driverOpen(gEnetAppParams[enetInstIdx].enetType, gEnetAppParams[enetInstIdx].instId);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to open ENET[%d]: %d\r\n", enetInstIdx, status);
            EnetAppUtils_assert(status == ENET_SOK);
        }
    }

    DebugP_log("Main Core init");

    /*! Have to handled in a better way. ToDo: Get the SoC Mac Instead */
    Icssg_MacAddr mac; //0xF4, 0x84, 0x4C, 0xF9, 0x4D, 0x29
    mac.macAddr[0] = 0xF4;
    mac.macAddr[1] = 0x84;
    mac.macAddr[2] = 0x4C;
    mac.macAddr[3] = 0xF9;
    mac.macAddr[4] = 0x4D;
    mac.macAddr[5] = 0x29;

    status = App_addMacFdbEntry(gEnetAppParams[0].enetType, gEnetAppParams[0].instId, mac);
    EnetAppUtils_assert(status == ENET_SOK);
    EnetAppUtils_print("Bridge mac added to FDB \r\n");

    /*! Initialize the IPC Task */
    AppCtrl_createRecvTask();
    /*! Handle the IC up notify to bring IC-netif up */
    EthApp_lwipMain(NULL, NULL);

    App_waitForBridgeUp();
    DebugP_log("Network is UP ...\r\n");

    /* Start the TCP server for the core */
    AppTcp_startServer();

    /* Wait for the TCP initialization to complete
     * Send the IP for remote cores for self test
     */
    ClockP_sleep(2);
    AppCtrl_sendIPNotify();

    sys_lock_tcpip_core();
    lwiperf_example_init();
    sys_thread_new("UDP Iperf", start_application, NULL, DEFAULT_THREAD_STACKSIZE,
                               UDP_IPERF_THREAD_PRIO);
    sys_unlock_tcpip_core();

    while (1)
    {
        ClockP_usleep(1000);
        App_printCpuLoad();
    }

    App_shutdownNetworkStack();
}

bool App_IsLinuxPresent()
{
    return ENET_IS_LINUX_PRESENT;
}

uint32_t App_getSelfCoreId()
{
    uint32_t coreId = EnetSoc_getCoreId();
    return coreId;
}

static void App_printCpuLoad()
{
    static uint32_t startTime_ms = 0;
    const  uint32_t currTime_ms  = ClockP_getTimeUsec()/1000;
    const  uint32_t printInterval_ms = 5000;

    if (startTime_ms == 0)
    {
        startTime_ms = currTime_ms;
    }
    else if ( (currTime_ms - startTime_ms) > printInterval_ms )
    {
        const uint32_t cpuLoad = TaskP_loadGetTotalCpuLoad();

        DebugP_log(" %6d.%3ds : CPU load = %3d.%02d %%\r\n",
                    currTime_ms/1000, currTime_ms%1000,
                    cpuLoad/100, cpuLoad%100 );

        startTime_ms = currTime_ms;
        TaskP_loadResetAll();
    }
    return;
}

int32_t AppCtrl_addMacAddr2fbd(Icssg_MacAddr mac)
{
    int32_t status = ICVE_OK;

    if(status == ICVE_OK)
    {
        status = App_addMacFdbEntry(gEnetAppParams[0].enetType, gEnetAppParams[0].instId, mac);
        EnetAppUtils_print("Adding new MAC addr to FDB  \r\n");
    }

    if(status == ICVE_OK)
    {
        EnetAppUtils_print("Remote mac addr added to FDB \r\n");
    }

    return status;
}

int32_t AppCtrl_addMcastAddr(Icssg_MacAddr mac)
{
    int32_t status = ICVE_OK;

    if(status == ICVE_OK)
    {
        status = App_addMacFdbEntry(gEnetAppParams[0].enetType, gEnetAppParams[0].instId, mac);
        EnetAppUtils_print("Adding new MCast entry to FDB  \r\n");
    }

    if(status == ICVE_OK)
    {
        status = AddNetif_addBridgeMcastEntry(mac);
        EnetAppUtils_print("Adding new MCast entry to LwIP Bridge \r\n");
    }

    if(status == ICVE_OK)
    {
        EnetAppUtils_print("MC addr added Successfully \r\n");
    }

    return status;
}

int32_t AppCtrl_delMcastAddr(Icssg_MacAddr mac)
{
    int32_t status = ICVE_OK;

    if(status == ICVE_OK)
    {
        status = App_delMacFdbEntry(gEnetAppParams[0].enetType, gEnetAppParams[0].instId, mac);
        EnetAppUtils_print("Deleting MCast entry from FDB \r\n");
    }

    if(status == ICVE_OK)
    {
        status = AddNetif_delBridgeMcastEntry(mac);
        EnetAppUtils_print("Deleting new MCast entry from LwIP Bridge \r\n");
    }

    if(status == ICVE_OK)
    {
        EnetAppUtils_print("MCast address deleted from FDB \r\n");
    }

    return status;
}
