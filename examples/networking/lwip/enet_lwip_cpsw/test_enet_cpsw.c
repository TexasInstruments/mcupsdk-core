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

/* SDK includes */
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
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
static void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                         void *appArg);
#else

EnetExtPhy_Handle EnetApp_getExtPhyHandle(Enet_MacPort macPort);

#endif

static void EnetApp_handleLinkChangeEvent(Enet_Handle hEnet, Cpsw_MdioLinkStateChangeInfo* pLinkChangeInfo);

static void EnetApp_portLinkStatusChangeCb(Enet_MacPort macPort,
                                           bool isLinkUp,
                                           void *appArg)
{
    EnetAppUtils_print("MAC Port %u: link %s\r\n",
                       ENET_MACPORT_ID(macPort), isLinkUp ? "up" : "down");
}

#if (ENET_SYSCFG_ENABLE_EXTPHY == 1U)

#define ENETAPP_MDIOLINKINT_HANDLER_TASK_PRIORITY        (7U)
#define ENETAPP_MDIOLINKINT_HANDLER_TASK_STACK     (3 * 1024)

typedef struct EnetAppPhyLinkEventInfoMsg_s
{
    QueueP_Elem elem;
    Cpsw_MdioLinkStateChangeInfo linkStateChangeInfo;
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
    Enet_Type enetType;
    uint32_t instId;
    uint32_t mdioLinkChangeIsrCount;
    QueueP_Object freeMsgQ;
    QueueP_Object processMsgQ;
    QueueP_Handle hFreeMsgQ;
    QueueP_Handle hProcessMsgQ;
    EnetAppPhyLinkEventInfoMsg infoMsg[QueueP_OBJECT_SIZE_MAX];
    /*! MDIO Link state change callback function pointer */
    Cpsw_MdioLinkStateChangeCb appMdioLinkStateChangeCb;
    /*! Application data to be passed to the MDIO link state change callback */
    void *appMdioLinkStateChangeCbArg;
    bool mdioLinkIntTaskShutDownFlag;
    TaskP_Object taskMdioLinkIntHandlerObj;
    ClockP_Object phyRegPollTimerObj;
    SemaphoreP_Object linkIntSem;
    uint8_t appMdioLinkIntHandlerTaskStack[ENETAPP_MDIOLINKINT_HANDLER_TASK_STACK] __attribute__ ((aligned(32)));
    EnetAppPhyPollState_s phyPollState;
} EnetAppPhyLinkEventCtx;

EnetAppPhyLinkEventCtx gMdioLinkEventCtx;


void EnetApp_initPhyLinkHandlerCtx(EnetAppPhyLinkEventCtx * ctx, Enet_Type enetType, uint32_t instId)
{
    uint32_t i;
    int32_t status;

    ctx->enetType = enetType;
    ctx->instId   = instId;
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
            Cpsw_MdioLinkStateChangeInfo *pLinkChangeInfo;
            uint32_t status;
            QueueP_Elem *qelem;
            EnetAppPhyLinkEventInfoMsg *infoMsg;

            Enet_Handle hEnet = Enet_getHandle(linkIntCtx->enetType, linkIntCtx->instId);

            qelem = QueueP_get(linkIntCtx->hProcessMsgQ);
            EnetAppUtils_assert(qelem != NULL);
            infoMsg = container_of(qelem, EnetAppPhyLinkEventInfoMsg, elem);
            pLinkChangeInfo = &infoMsg->linkStateChangeInfo;

            if (pLinkChangeInfo->linkChanged)
            {
                EnetApp_handleLinkChangeEvent(hEnet, pLinkChangeInfo);
            }
            status = QueueP_put(linkIntCtx->hFreeMsgQ, &infoMsg->elem);
            EnetAppUtils_assert(status == SystemP_SUCCESS);
       }
    }
}

static void EnetApp_handleLinkChangeEvent(Enet_Handle hEnet, Cpsw_MdioLinkStateChangeInfo* pLinkChangeInfo)
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

int32_t EnetApp_mdioLinkIntHandlerTask(Enet_Type enetType, uint32_t instId)
{
    TaskP_Params tskParams;
    int32_t status;

    EnetApp_initPhyLinkHandlerCtx(&gMdioLinkEventCtx, enetType, instId);
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

void EnetApp_mdioLinkChangeISR(Cpsw_MdioLinkStateChangeInfo *info,
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


static void EnetApp_initMdioLinkIntCfg(Enet_Type enetType, uint32_t instId, Cpsw_Cfg *cpswCfg)
{
#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    cpswCfg->mdioLinkStateChangeCb     = NULL;
    cpswCfg->mdioLinkStateChangeCbArg  = NULL;
#else
    /*! MDIO Link state change callback function pointer */
    cpswCfg->mdioLinkStateChangeCb     = EnetApp_mdioLinkChangeISR;

    /*! Application data to be passed to the MDIO link state change callback */
    cpswCfg->mdioLinkStateChangeCbArg  = &gMdioLinkEventCtx;
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

        for (uint32_t portIdx = 0; portIdx < ENET_SYSCFG_MAX_MAC_PORTS; portIdx++)
        {
            const EnetExtPhy_Handle hPhy = EnetApp_getExtPhyHandle(portIdx);
            if (hPhy == NULL)
            {
                continue;
            }
            const uint32_t phyAddress = hPhy->phyCfg.phyAddr;
            if (phyAddress == 0)
            {
                // invalid PHY address
                continue;
            }

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

                    Cpsw_MdioLinkStateChangeInfo info = {
                                      .phyAddr      = phyAddress,
                                      .aliveChanged = false,
                                      .isAlive      = isLinked,
                                      .linkChanged  = true,
                                      .isLinked     = isLinked
                                      };

                    Enet_Handle hEnet = Enet_getHandle(linkIntCtx->enetType, linkIntCtx->instId);
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

int32_t EnetApp_createPhyRegisterPollingTask(const uint32_t pollingPeriod_ms, const bool doStartimmediately, Enet_Type enetType, uint32_t instId)
{
    const uint32_t pollingPeriodTicks = ClockP_usecToTicks((ENETPHY_FSM_TICK_PERIOD_MS)*pollingPeriod_ms);  // Set timer expiry time in OS ticks

    EnetApp_initPhyLinkHandlerCtx(&gMdioLinkEventCtx, enetType, instId);
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



void EnetApp_updateCpswInitCfg(Enet_Type enetType, uint32_t instId, Cpsw_Cfg *cpswCfg)
{
#if defined (ENET_SOC_HOSTPORT_DMA_TYPE_CPDMA)
    EnetCpdma_Cfg * dmaCfg = (EnetCpdma_Cfg *)cpswCfg->dmaCfg;

    EnetAppUtils_assert(dmaCfg != NULL);
    EnetAppUtils_assert(EnetAppUtils_isDescCached() == false);
    dmaCfg->rxInterruptPerMSec = 8;
    dmaCfg->txInterruptPerMSec = 2;
    dmaCfg->enChOverrideFlag = true;
#endif


#if (ENET_SYSCFG_ENABLE_MDIO_MANUALMODE == 1U)
    cpswCfg->mdioLinkStateChangeCb    = NULL;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
#else
    #if (ENET_SYSCFG_ENABLE_EXTPHY == 1U)
    EnetApp_initMdioLinkIntCfg(enetType, instId, cpswCfg);
    #else
    cpswCfg->mdioLinkStateChangeCb    = &EnetApp_mdioLinkStatusChange;
    cpswCfg->mdioLinkStateChangeCbArg = NULL;
    #endif /*(ENET_SYSCFG_ENABLE_EXTPHY == 1U) */
#endif
    cpswCfg->portLinkStatusChangeCb = &EnetApp_portLinkStatusChangeCb;
    cpswCfg->portLinkStatusChangeCbArg = NULL;

}

#if (ENET_SYSCFG_ENABLE_EXTPHY == 0U)
static void EnetApp_mdioLinkStatusChange(Cpsw_MdioLinkStateChangeInfo *info,
                                         void *appArg)
{
    if (info->linkChanged)
    {
        EnetAppUtils_print("Link Status Changed. PHY: 0x%x, state: %s\r\n",
                info->phyAddr,
                info->isLinked? "up" : "down");
    }
}
#endif

