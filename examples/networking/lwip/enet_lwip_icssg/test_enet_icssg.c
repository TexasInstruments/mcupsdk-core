/*
 * Copyright (c) 2001,2002 Florian Schulze.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the authors nor the names of the contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * test.c - This file is part of lwIP test
 *
 */

/* C runtime includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "test_enet_lwip.h"
#include "ti_board_config.h"
/* SDK includes */
#include "ti_drivers_open_close.h"
#include "ti_drivers_config.h"
#include "ti_board_open_close.h"
#include "ti_enet_open_close.h"
#include "ti_enet_config.h"
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/QueueP.h>

#if (ENET_SYSCFG_ENABLE_EXTPHY == 1)
#include "enetextphy.h"
#include "enetextphy_phymdio_dflt.h"
#include "test_enet_extphy.h"
#endif

#if (ENET_SYSCFG_ENABLE_EXTPHY == 0U)
static void EnetApp_mdioLinkStatusChange(Icssg_MdioLinkStateChangeInfo *info,
                                         void *appArg);
#else
EnetExtPhy_Handle EnetApp_getExtPhyHandle(Enet_MacPort macPort);
#endif

static void EnetApp_handleLinkChangeEvent(Enet_Handle hEnet,
                                          Icssg_MdioLinkStateChangeInfo*
                                          pLinkChangeInfo);

#define MII_LINK0_EVENT      41
#define MII_LINK1_EVENT      53

#if (ENET_SYSCFG_ENABLE_EXTPHY == 1U)

#define ENETAPP_MDIOLINKINT_HANDLER_TASK_PRIORITY        (7U)
#define ENETAPP_MDIOLINKINT_HANDLER_TASK_STACK     (3 * 1024)

typedef struct EnetAppPhyLinkEventInfoMsg_s
{
    QueueP_Elem elem;
    Icssg_MdioLinkStateChangeInfo linkStateChangeInfo;
    void *mdioLinkStateChangeCbArg;
} EnetAppPhyLinkEventInfoMsg;

typedef struct EnetAppPhyPollState_s
{
    uint32_t aliveStatusPhyAddMask;
    uint32_t linkStatusPhyAddMask;
    uint32_t pollEnablePhyAddMask;
} EnetAppPhyPollState_s;


typedef struct EnetAppPhyLinkEventCtx_s
{
    Enet_Type port2EnetType[ENET_SYSCFG_MAX_MAC_PORTS];
    uint32_t port2InstId[ENET_SYSCFG_MAX_MAC_PORTS];
    uint32_t mdioLinkChangeIsrCount;
    QueueP_Object freeMsgQ;
    QueueP_Object processMsgQ;
    QueueP_Handle hFreeMsgQ;
    QueueP_Handle hProcessMsgQ;
    EnetAppPhyLinkEventInfoMsg infoMsg[QueueP_OBJECT_SIZE_MAX];
    /*! MDIO Link state change callback function pointer */
    Icssg_MdioLinkStateChangeCb appMdioLinkStateChangeCb;
    /*! Application data to be passed to the MDIO link state change callback */
    void *appMdioLinkStateChangeCbArg;
    bool mdioLinkIntTaskShutDownFlag;
    TaskP_Object taskMdioLinkIntHandlerObj;
    ClockP_Object phyRegPollTimerObj;
    SemaphoreP_Object linkIntSem;
    uint8_t appMdioLinkIntHandlerTaskStack[ENETAPP_MDIOLINKINT_HANDLER_TASK_STACK] __attribute__ ((aligned(32)));
    EnetAppPhyPollState_s phyPollState;
    bool isInitialized;
} EnetAppPhyLinkEventCtx;

EnetAppPhyLinkEventCtx gMdioLinkEventCtx = {.isInitialized = false};


void EnetApp_initPhyLinkHandlerCtx(EnetAppPhyLinkEventCtx * ctx, Enet_Type port2EnetType[ENET_SYSCFG_MAX_MAC_PORTS], uint32_t port2InstId[ENET_SYSCFG_MAX_MAC_PORTS])
{
    uint32_t i;
    int32_t status;

    for (uint32_t idx = 0; idx < ENET_SYSCFG_MAX_MAC_PORTS; idx++)
    {
        ctx->port2EnetType[idx] = port2EnetType[idx];
        ctx->port2InstId[idx]   = port2InstId[idx];
    }
    ctx->mdioLinkChangeIsrCount = 0;
    ctx->mdioLinkIntTaskShutDownFlag = false;
    status = SemaphoreP_constructCounting(&ctx->linkIntSem, 0, ENET_ARRAYSIZE(ctx->infoMsg));
    EnetAppUtils_assert(status == SystemP_SUCCESS);

    ctx->hFreeMsgQ = QueueP_create(&(ctx->freeMsgQ));
    EnetAppUtils_assert(ctx->hFreeMsgQ != NULL);
    ctx->hProcessMsgQ =  QueueP_create(&(ctx->processMsgQ));
    EnetAppUtils_assert(ctx->hProcessMsgQ != NULL);

    ENET_UTILS_COMPILETIME_ASSERT(offsetof(EnetAppPhyLinkEventInfoMsg, elem) == 0);
    for (i = 0; i < ENET_ARRAYSIZE(ctx->infoMsg); i++)
    {
        status = QueueP_put(ctx->hFreeMsgQ, &ctx->infoMsg[i].elem);
        EnetAppUtils_assert(status == SystemP_SUCCESS);
    }
}

static void EnetApp_mdioLinkIntHandler(void * appHandle)
{
    EnetAppPhyLinkEventCtx *linkIntCtx  = (EnetAppPhyLinkEventCtx *)appHandle;
    SemaphoreP_Object *linkIntSem       = &linkIntCtx->linkIntSem;
    while (linkIntCtx->mdioLinkIntTaskShutDownFlag != true)
    {
        SemaphoreP_pend(linkIntSem, SystemP_WAIT_FOREVER);
        {
            uint32_t status;
            Icssg_MdioLinkStateChangeInfo *pLinkChangeInfo;
            QueueP_Elem *qelem;
            EnetAppPhyLinkEventInfoMsg *infoMsg;

            qelem = QueueP_get(linkIntCtx->hProcessMsgQ);
            EnetAppUtils_assert(qelem != NULL);
            infoMsg = container_of(qelem, EnetAppPhyLinkEventInfoMsg, elem);
            pLinkChangeInfo = &infoMsg->linkStateChangeInfo;

            if (pLinkChangeInfo->linkChanged)
            {

                for (uint32_t macPort = 0; macPort < ENET_SYSCFG_MAX_MAC_PORTS; macPort++)
                {
                    const EnetExtPhy_Handle hPhy = EnetApp_getExtPhyHandle(macPort);
                    if (hPhy == NULL)
                    {
                        continue;
                    }
                    const uint32_t phyAddress = hPhy->phyCfg.phyAddr;
                    if (phyAddress == infoMsg->linkStateChangeInfo.phyAddr)
                    {
                        /* For here for the PHY whose link has changed */
                        Enet_Handle hEnet = Enet_getHandle(linkIntCtx->port2EnetType[macPort], linkIntCtx->port2InstId[macPort]);
                        EnetApp_handleLinkChangeEvent(hEnet, pLinkChangeInfo);
                        break;
                    }

                }
            }
            status = QueueP_put(linkIntCtx->hFreeMsgQ, &infoMsg->elem);
            EnetAppUtils_assert(status == SystemP_SUCCESS);
       }
    }
}

static void EnetApp_handleLinkChangeEvent(Enet_Handle hEnet, Icssg_MdioLinkStateChangeInfo* pLinkChangeInfo)
{
    uint32_t status = ENET_SOK;
    Enet_MacPort macPort;
    Enet_IoctlPrms prms;
    const uint32_t selfCoreId = EnetSoc_getCoreId();

    if (pLinkChangeInfo->isLinked)
    {
        Enet_Type enetType;
        uint32_t instId;

        Enet_ExtPhyLinkUpEventInfo linkupEventInfo;


        status = Enet_getHandleInfo(hEnet, &enetType, &instId);
        EnetAppUtils_assert(status == ENET_SOK);
        status = EnetApp_getExtPhyLinkCfgInfo(enetType,
                                     instId,
                                     &macPort,
                                     pLinkChangeInfo->phyAddr,
                                     &linkupEventInfo.phyLinkCfg);
        if (ENETEXTPHY_SOK == status)
        {
            linkupEventInfo.macPort = macPort;
            ENET_IOCTL_SET_IN_ARGS(&prms, &linkupEventInfo);
            ENET_IOCTL(hEnet, selfCoreId, ENET_PER_IOCTL_HANDLE_EXTPHY_LINKUP_EVENT, &prms, status);
        }
        else
        {
            EnetAppUtils_print("Link is not currently up. Ignore this msg");
            status = ENET_SOK;
        }
    }
    else
    {
        ENET_IOCTL_SET_IN_ARGS(&prms, &macPort);
        ENET_IOCTL(hEnet, selfCoreId, ENET_PER_IOCTL_HANDLE_EXTPHY_LINKDOWN_EVENT, &prms, status);
    }
    EnetAppUtils_assert(status == ENET_SOK);
}

int32_t EnetApp_mdioLinkIntHandlerTask(Enet_Type port2EnetType[ENET_SYSCFG_MAX_MAC_PORTS], uint32_t port2InstId[ENET_SYSCFG_MAX_MAC_PORTS])
{
    TaskP_Params tskParams;
    int32_t status;

    EnetApp_initPhyLinkHandlerCtx(&gMdioLinkEventCtx, port2EnetType, port2InstId);
    /* Initialize the taskperiodicTick params. Set the task priority higher than the
     * default priority (1) */
    TaskP_Params_init(&tskParams);
    tskParams.priority       = ENETAPP_MDIOLINKINT_HANDLER_TASK_PRIORITY;
    tskParams.stack          = &gMdioLinkEventCtx.appMdioLinkIntHandlerTaskStack[0];
    tskParams.stackSize      = sizeof(gMdioLinkEventCtx.appMdioLinkIntHandlerTaskStack);
    tskParams.args           = &gMdioLinkEventCtx;
    tskParams.name           = "EnetApp_MdioLinkIntHandlerTask";
    tskParams.taskMain       =  &EnetApp_mdioLinkIntHandler;

    status = TaskP_construct(&gMdioLinkEventCtx.taskMdioLinkIntHandlerObj, &tskParams);
    EnetAppUtils_assert(status == SystemP_SUCCESS);

    return status;

}

void EnetApp_mdioLinkChangeISR(Icssg_MdioLinkStateChangeInfo *info,
                               void *appArg)
{
    EnetAppPhyLinkEventCtx *linkIntCtx = (EnetAppPhyLinkEventCtx *)appArg;
    QueueP_Elem *qelem;
    EnetAppPhyLinkEventInfoMsg *infoMsg;

    linkIntCtx->mdioLinkChangeIsrCount++;
    qelem = QueueP_get(linkIntCtx->hFreeMsgQ);
    EnetAppUtils_assert(qelem != NULL);
    infoMsg = container_of(qelem, EnetAppPhyLinkEventInfoMsg, elem);
    infoMsg->linkStateChangeInfo = *info;
    infoMsg->mdioLinkStateChangeCbArg = appArg;
    QueueP_put(linkIntCtx->hProcessMsgQ, &infoMsg->elem);
    SemaphoreP_post(&linkIntCtx->linkIntSem);
}

static void EnetApp_initMdioLinkIntCfg(Enet_Type enetType, uint32_t instId, Icssg_Cfg *icssgCfg)
{
#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCb     = NULL;
    icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCbArg  = NULL;
#else
    /*! MDIO Link state change callback function pointer */
    icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCb = EnetApp_mdioLinkChangeISR;

    /*! Application data to be passed to the MDIO link state change callback */
    icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCbArg  = &gMdioLinkEventCtx;
#endif
}

#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)

static void EnetApp_phyRegPollHandler(void* appHandle)
{
    EnetAppPhyLinkEventCtx *linkIntCtx  = (EnetAppPhyLinkEventCtx *)appHandle;
    SemaphoreP_Object *pTimerSem        =  &linkIntCtx->linkIntSem;

    while (true)
    {
        // phyStatus
        SemaphoreP_pend(pTimerSem, SystemP_WAIT_FOREVER);
        const uint32_t pollEnablePhyAddressMask = linkIntCtx->phyPollState.pollEnablePhyAddMask;

        for (uint32_t macPort = 0; macPort < ENET_SYSCFG_MAX_MAC_PORTS; macPort++)
        {
            const EnetExtPhy_Handle hPhy = EnetApp_getExtPhyHandle(macPort);
            if (hPhy == NULL)
            {
                continue;
            }
            const uint32_t phyAddress = hPhy->phyCfg.phyAddr;

            const uint32_t phyaddressMask = (1 << phyAddress);
            if (pollEnablePhyAddressMask & phyaddressMask)
            {
                const bool isLinked = EnetExtPhy_isLinked(hPhy);

                if ((isLinked << phyAddress) ^ (linkIntCtx->phyPollState.linkStatusPhyAddMask & phyaddressMask))
                {
                    // fall here if link status has changed
                    if (isLinked)
                    {
                        linkIntCtx->phyPollState.linkStatusPhyAddMask |= phyaddressMask;
                    }
                    else
                    {
                        linkIntCtx->phyPollState.linkStatusPhyAddMask &= (~phyaddressMask);
                    }

                    Icssg_MdioLinkStateChangeInfo info = {
                                      .phyAddr      = phyAddress,
                                      .aliveChanged = false,
                                      .isAlive      = isLinked,
                                      .linkChanged  = true,
                                      .isLinked     = isLinked
                                      };

                    Enet_Handle hEnet = Enet_getHandle(linkIntCtx->port2EnetType[macPort], linkIntCtx->port2InstId[macPort]);
                    EnetApp_handleLinkChangeEvent(hEnet, &info);

                }
            } /* if (pollEnablePhyAddressMask & phyaddressMask)*/
        } /* for loop*/
    }

    TaskP_destruct(&linkIntCtx->taskMdioLinkIntHandlerObj);
    TaskP_exit();
}


static void EnetApp_phyRegPollTimerCb(ClockP_Object *clkInst, void * arg)
{
    SemaphoreP_Object * pTimerSem = (SemaphoreP_Object *)arg;

    /* Tick! */
    SemaphoreP_post(pTimerSem);
}

void EnetApp_enablePhyLinkPollingMask(const uint32_t phyEnableMask)
{
    gMdioLinkEventCtx.phyPollState.pollEnablePhyAddMask = phyEnableMask;
}

int32_t EnetApp_createPhyRegisterPollingTask(const uint32_t pollingPeriod_ms,
                                             const bool doStartimmediately,
                                             Enet_Type port2EnetType[ENET_SYSCFG_MAX_MAC_PORTS],
                                             uint32_t port2InstId[ENET_SYSCFG_MAX_MAC_PORTS])
{
    const uint32_t pollingPeriodTicks = ClockP_usecToTicks((ENETPHY_FSM_TICK_PERIOD_MS)*pollingPeriod_ms);  // Set timer expiry time in OS ticks

    EnetApp_initPhyLinkHandlerCtx(&gMdioLinkEventCtx, port2EnetType, port2InstId);
    EnetAppUtils_assert(SystemP_SUCCESS == SemaphoreP_constructCounting(&(gMdioLinkEventCtx.linkIntSem), 0, 128));
    ClockP_Params clkParams;
    ClockP_Params_init(&clkParams);
    clkParams.start     = doStartimmediately;
    clkParams.timeout   = pollingPeriodTicks;
    clkParams.period    = pollingPeriodTicks;
    clkParams.args      = &(gMdioLinkEventCtx.linkIntSem);
    clkParams.callback  = &EnetApp_phyRegPollTimerCb;

    /* Creating timer and setting timer callback function*/
    const int32_t timerStatus = ClockP_construct(&(gMdioLinkEventCtx.phyRegPollTimerObj), &clkParams);
    EnetAppUtils_assert(timerStatus == SystemP_SUCCESS);
    if (timerStatus != SystemP_SUCCESS)
    {
        EnetAppUtils_print("failed to PHY polling clock\r\n");
        return timerStatus;
    }

    /* Initialize the taskperiodicTick params. Set the task priority higher than the
     * default priority (1) */
    TaskP_Params tskParams;
    TaskP_Params_init(&tskParams);
    tskParams.priority       = ENETAPP_MDIOLINKINT_HANDLER_TASK_PRIORITY;
    tskParams.stack          = &(gMdioLinkEventCtx.appMdioLinkIntHandlerTaskStack[0]);
    tskParams.stackSize      = sizeof(gMdioLinkEventCtx.appMdioLinkIntHandlerTaskStack);
    tskParams.args           = &gMdioLinkEventCtx;
    tskParams.name           = "EnetApp_PhyRegPollTask";
    tskParams.taskMain       = &EnetApp_phyRegPollHandler;

    const int32_t taskStatus = TaskP_construct(&gMdioLinkEventCtx.taskMdioLinkIntHandlerObj, &tskParams);
    EnetAppUtils_assert(taskStatus == SystemP_SUCCESS);

    return (timerStatus | taskStatus);
}
#endif /*#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U) */

#endif

static void EnetApp_updateMdioLinkIntCfg(Enet_Type enetType, uint32_t instId, Icssg_mdioLinkIntCfg *mdioLinkIntCfg)
{
    /*! INTC Module mapping data passed by application for configuring PRU to R5F interrupts */
#if (ENET_SYSCFG_ICSSG0_ENABLED == 1)
    mdioLinkIntCfg->prussIntcInitData =  &icss0_intc_initdata;
#endif
#if (ENET_SYSCFG_ICSSG1_ENABLED == 1)
    mdioLinkIntCfg->prussIntcInitData =  &icss1_intc_initdata;
#endif
    mdioLinkIntCfg->coreIntrNum = 254;
    mdioLinkIntCfg->pruEvtNum[0] = MII_LINK0_EVENT;
    mdioLinkIntCfg->pruEvtNum[1] = MII_LINK1_EVENT;
    mdioLinkIntCfg->isPulseIntr = 0;
    mdioLinkIntCfg->intrPrio = 15;
}

void EnetApp_updateIcssgInitCfg(Enet_Type enetType, uint32_t instId, Icssg_Cfg *icssgCfg)
{
#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCb = NULL;
    icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCbArg  = NULL;
#else
    #if (ENET_SYSCFG_ENABLE_EXTPHY == 1U)
        EnetApp_initMdioLinkIntCfg(enetType, instId, icssgCfg);
    #else
        icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCb = &EnetApp_mdioLinkStatusChange;
        icssgCfg->mdioLinkIntCfg.mdioLinkStateChangeCbArg  = NULL;
    #endif
    EnetApp_updateMdioLinkIntCfg(enetType, instId, &icssgCfg->mdioLinkIntCfg);
#endif
}

#if (ENET_SYSCFG_ENABLE_EXTPHY == 0U)
static void EnetApp_mdioLinkStatusChange(Icssg_MdioLinkStateChangeInfo *info,
                                         void *appArg)
{
    EnetAppUtils_print("Link Status Changed. PHY: 0x%x, state: %s\r\n",
            info->phyAddr,
            info->isLinked? "up" : "down");
}
#endif
