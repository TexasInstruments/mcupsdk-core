/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
/*
 * pnDrvConfig.h needs to be first first file included in driver!
 * It is application dependent and as such part of the application code.
 * It defines all basic feature options in driver (compile time!)
 */
#include "pnDrvConfig.h"
#include "PN_Handle.h"
#include "PN_HandleDef.h"
#include "iRtcDrv.h"
#include "iPnOs.h"
#ifdef PTCP_SUPPORT
#include "iPtcpDrv.h"
#include "iPtcpUtils.h"
#endif
#include <drivers/hw_include/hw_types.h>
#include <drivers/mdio.h>
#include <kernel/dpl/ClockP.h>
#include <string.h>

/* TODO: Review this*/
// extern SemaphoreP_Handle &(pnHandle->switchReady);
/* ========================================================================== */
/*                          Local Variables                                   */
/* ========================================================================== */

#ifdef MRP_SUPPORT
/** @addtogroup PN_MRP
@{ */
/**
* \brief MRP Task to control flush mode of ICSS
*
* @detail In flush mode ICSS will send all PPM on both ports. This task will
* enable/disable this mode based on the MRP state machine @ref tMrpStates .
* It implements a state machine according to ProfinetFlushTask\n
* Implements the following state machine
* @image html mrp_state.png
*
* @param arg0  not used
* @param arg1  not used
*/
void PN_MRP_CPMTask(uintptr_t arg0, uintptr_t arg1);

/**
@}
*/
#endif /* MRP_SUPPORT*/

/**
 * @brief dedicated task to tap the ICSS WatchDog Timer
 *
 *
 * @param arg0 not used
 * @param arg1 not used
 */
#ifdef WATCHDOG_SUPPORT
void PN_tapWatchDog_task(uintptr_t arg0, uintptr_t arg1);
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
#ifdef IRT_LEGACY_STARTUP_SUPPORT

/** @addtogroup PN_IRT_LEGACY
@{ */
/**
* @brief Dedicated task for legacy startup mode scheduling. Manages the state machine
* Required in IRT mode to support legacy startup. States defined by @ref tLegStates.
* Implements the following state machine \n
* @image html irt_legstate.png
*
* @param arg0  not used
* @param arg1  not used
*/
void PN_IRT_legacyTask(uintptr_t arg0, uintptr_t arg1);
/**
@}
*/
#endif /*IRT_LEGACY_STARTUP_SUPPORT*/

/**
 * @brief Dedicated task for PTCP delay measurement scheduling
 * Required in IRT mode
 *
 * @param arg0 not used
 * @param arg1 not used
 */
void PN_PTCP_task(uintptr_t arg0, uintptr_t arg1);

/**
 * @brief Dedicated task for PTCP sync monitor scheduling
 * Required in IRT mode
 *
 * @param arg0 not used
 * @param arg1 not used
 */
void PN_PTCP_syncMonitorTask(uintptr_t arg0, uintptr_t arg1);

#ifdef IRT_LEGACY_STARTUP_SUPPORT

#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
int32_t PN_initOs(PN_Handle pnHandle)
{
    TaskP_Params taskParams;
    uint32_t status = SystemP_FAILURE;

#ifdef PTCP_SUPPORT
    TaskP_Params_init(&taskParams);

    taskParams.name = "PTCPTask";
    taskParams.stackSize = PN_TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)(&(pnHandle->PTCP_taskStack));
    taskParams.priority = 11;
    taskParams.args = (void *)pnHandle;
    taskParams.taskMain = (TaskP_FxnMain)PN_PTCP_task;
    status = TaskP_construct(&(pnHandle->PTCPTaskObject), &taskParams);

    if(status == SystemP_FAILURE)
    {
        return -1;
    }

    TaskP_Params_init(&taskParams);

    taskParams.name = "SyncMonitorTask";
    taskParams.stackSize = PN_TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)(&(pnHandle->PTCP_syncMonitorTaskStack));
    taskParams.priority = 11;
    taskParams.args = (void *)pnHandle;
    taskParams.taskMain = (TaskP_FxnMain)PN_PTCP_syncMonitorTask;
    status = TaskP_construct(&(pnHandle->SyncMonitorTaskObject), &taskParams);

    if(status == SystemP_FAILURE)
    {
        return -2;
    }

#endif /*PTCP_SUPPORT*/

#ifdef IRT_LEGACY_STARTUP_SUPPORT
    TaskP_Params_init(&taskParams);

    taskParams.name = "LegacyModeTask";
    taskParams.stackSize = PN_TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)(&(pnHandle->IRT_legacyTaskStack));
    taskParams.priority = 3;
    taskParams.args = (void *)pnHandle;
    taskParams.taskMain = (TaskP_FxnMain)PN_IRT_legacyTask;
    status = TaskP_construct(&(pnHandle->LegModeTaskObject), &taskParams);

    if(status == SystemP_FAILURE)
    {
        return -3;
    }

    PN_registerSetState(pnHandle, PN_setLegState);
    PN_registerSetPkt(pnHandle, PN_setLegPkt);
#endif /*IRT_LEGACY_STARTUP_SUPPORT*/

#ifdef MRP_SUPPORT
    TaskP_Params_init(&taskParams);
    taskParams.name = "MrpMachine";

    taskParams.stackSize = PN_TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)(&(pnHandle->MRP_CPMTaskStack));
    taskParams.priority = 3;        /* FW low prio ok?*/
    taskParams.args = (void *)pnHandle;
    taskParams.taskMain = (TaskP_FxnMain)PN_MRP_CPMTask;
    status = TaskP_construct(&(pnHandle->MrpMachineTaskObject), &taskParams);
#endif /*MRP_SUPPORT*/
#ifdef WATCHDOG_SUPPORT
    TaskP_Params_init(&taskParams);

    taskParams.name = "WatchDogTimer";
    taskParams.stackSize = PN_TASK_STACK_SIZE;
    taskParams.stack = (uint8_t *)(&(pnHandle->tapWatchDog_taskStack));
    taskParams.priority = 11;
    taskParams.args = (void *)pnHandle;
    taskParams.taskMain = (TaskP_FxnMain)PN_tapWatchDog_task;
    status = TaskP_construct(&(pnHandle->WatchDogTimerTaskObject), &taskParams);

    if(status == SystemP_FAILURE)
    {
        return -4;
    }

#endif /*WATCHDOG_SUPPORT*/

    return 0;
}

int32_t PN_RTC_setupIsr(PN_Handle pnHandle)
{
    PN_IntConfig *intConfig = &(pnHandle->pnIntConfig);
    HwiP_Params hwiParams;
    uint32_t status = SystemP_FAILURE;


    HwiP_Params_init(&hwiParams);
    /* setup CPM ISR*/
    hwiParams.args = (void *)(intConfig->cpmIntConfig).args;        /*Emac SUub system handle*/
    /* TODO: Review this. This parameter is not available*/
    // hwiParams.enableIntr = FALSE;        /* enable later*/
    hwiParams.intNum = (intConfig->cpmIntConfig).coreIntNum;
    hwiParams.callback = (HwiP_FxnCallback)((intConfig->cpmIntConfig).isrFnPtr);
    hwiParams.priority = (intConfig->cpmIntConfig).intPrio;

    status = HwiP_construct(&((intConfig->cpmIntConfig).interruptObject), &hwiParams);

    if(status == SystemP_SUCCESS)
    {
        HwiP_Params_init(&hwiParams);
        /* setup PPM ISR*/
        hwiParams.args = (void *)((intConfig->ppmIntConfig).args);      /* just an ID*/
        /* TODO: Review this. This parameter is not available*/
        // hwiParams.enableIntr = FALSE;        /* enable later*/
        hwiParams.intNum = (intConfig->ppmIntConfig).coreIntNum;
        hwiParams.callback = (HwiP_FxnCallback)((intConfig->ppmIntConfig).isrFnPtr);
        hwiParams.priority = (intConfig->ppmIntConfig).intPrio;

        status = HwiP_construct(&((intConfig->ppmIntConfig).interruptObject), &hwiParams);
    }

    if(status == SystemP_SUCCESS)
    {
        HwiP_Params_init(&hwiParams);
        /* setup DHT expired ISR*/
        hwiParams.args = (void *)((intConfig->dhtIntConfig).args);        /* just an ID*/
        /* TODO: Review this. This parameter is not available*/
        // hwiParams.enableIntr = FALSE;        /* enable later*/
        hwiParams.intNum = (intConfig->dhtIntConfig).coreIntNum;
        hwiParams.callback = (HwiP_FxnCallback)((intConfig->dhtIntConfig).isrFnPtr);
        hwiParams.priority = (intConfig->dhtIntConfig).intPrio;    /* FW bump the ISR prio*/

        status = HwiP_construct(&((intConfig->dhtIntConfig).interruptObject), &hwiParams);
    }

    if(status == SystemP_SUCCESS)
    {
        /*TODO: Review this. Done to emulate enableIntr = FALSE*/
        PN_RTC_disableISR(pnHandle);
    }

    return status;
}

int32_t PN_RTC_enableISR(PN_Handle pnHandle)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    PN_IntAttrs *ppmIntConfig = &((pnHandle->pnIntConfig).ppmIntConfig);
    PN_IntAttrs *cpmIntConfig = &((pnHandle->pnIntConfig).cpmIntConfig);
    PN_IntAttrs *dhtIntConfig = &((pnHandle->pnIntConfig).dhtIntConfig);
    /* reset some flags used for ISR*/
    HW_WR_REG8(pruicssHwAttrs->pru0DramBase + RTC_NOTIFY_DHT_EVENT_OFFSET, 0);
    HW_WR_REG8(pruicssHwAttrs->pru0DramBase + RTC_NOTIFY_LIST_TOGGLE_EVENT_OFFSET, 0);
    /* clear pending PRU ISR flags*/
    PN_clearPruIRQ(pruicssHwAttrs, cpmIntConfig->pruIntNum);
    PN_clearPruIRQ(pruicssHwAttrs, ppmIntConfig->pruIntNum);
    PN_clearPruIRQ(pruicssHwAttrs, dhtIntConfig->pruIntNum);
    /*enable all IRQ*/
    HwiP_enableInt(cpmIntConfig->coreIntNum);
    HwiP_enableInt(ppmIntConfig->coreIntNum);
    HwiP_enableInt(dhtIntConfig->coreIntNum);

    return 0;
}

int32_t PN_RTC_disableISR(PN_Handle pnHandle)
{
    PN_IntAttrs *ppmIntConfig = &((pnHandle->pnIntConfig).ppmIntConfig);
    PN_IntAttrs *cpmIntConfig = &((pnHandle->pnIntConfig).cpmIntConfig);
    PN_IntAttrs *dhtIntConfig = &((pnHandle->pnIntConfig).dhtIntConfig);

    /*TODO: Review this*/
    /*disable all IRQ*/
    HwiP_disableInt(cpmIntConfig->coreIntNum);
    HwiP_disableInt(ppmIntConfig->coreIntNum);
    HwiP_disableInt(dhtIntConfig->coreIntNum);

    return 0;
}

/*-----------------------------------------------------
 * new data API
 *-----------------------------------------------------*/
/*
 * called by HWI!
 */
int32_t PN_checkLastPPM(PN_Handle pnHandle, t_rtcPacket *pkt)
{
    uint8_t temp;
    uintptr_t key;

    if(pkt->type != PPM)
    {
        return -1;   /* only usable with PPM packet*/
    }

    if(pkt->validLast)
    {
        key = HwiP_disable(); /* protect the swap operation*/
        temp = pkt->proc;               /*swap the tripple buffer indexes*/
        pkt->proc = pkt->last;
        pkt->last = temp;
        pkt->validLast = 0;
        PN_chgPpmBuffer(pnHandle, pkt);             /* update descriptor -> new data*/
        HwiP_restore(key);
        return 1;
    }

    return 0;
}

uint8_t *PN_getPpmBuff(t_rtcPacket *pkt)
{
    if(pkt->type != PPM)
    {
        return NULL;    /* only usable with PPM packet*/
    }

    return pkt->pBuffer->addr[pkt->next];   /* current NEXT buffer address*/

}

uint8_t *PN_relPpmBuff(PN_Handle pnHandle, t_rtcPacket *pkt)
{
    uint8_t temp;
    uintptr_t key;
    t_descList *pList;
    volatile uint8_t *pBcEvent;

    if(pkt->type != PPM)
    {
        return NULL;    /* only usable with PPM packet*/
    }

    if(0 == pkt->isActive)
    {
        return NULL;
    }

    key = HwiP_disable(); /* protect the swap operation*/
    temp = pkt->last;                   /* swap the tripple buffer indexes*/
    pkt->last = pkt->next;
    pkt->next = temp;
    pkt->validLast = 1;           /* queue up for next PPM BC HWI*/

    /* now check if we can directly write new last packet to PPM descriptor!*/
    pList = &(pnHandle->ppmList);
    pBcEvent = pList->pEvent + pkt->lIndex;

    if(*pBcEvent == 0)                  /* PPM not in send...*/
    {
        temp = pkt->proc;               /* start buffer swap*/
        pkt->proc = pkt->last;
        PN_chgPpmBuffer(pnHandle, pkt);             /* update descriptor -> new data*/
        pkt->validLast = 0;

        if(*pBcEvent == 0)              /* reread send status*/
        {
            pkt->last = temp;           /* complete the swap*/
            pkt->validLast = 0;
        }

        else
        {
            pkt->proc = temp;           /* reset old proc*/
            PN_chgPpmBuffer(pnHandle, pkt);         /* update descriptor -> old data!*/
            pkt->validLast = 1;             /* queue up for next PPM BC HWI*/
        }
    }

    HwiP_restore(key);
    return pkt->pBuffer->addr[pkt->next];   /* new NEXT buffer address*/
}

int32_t PN_nextCpmRdy(PN_Handle pnHandle, t_rtcPacket *pkt)
{
    uint8_t temp;
    int32_t ret = 0;
    int8_t buffIndex;

    uintptr_t key;

    if(pkt->type != CPM)
    {
        return -1;    /* only usable with CPM packet*/
    }

    key = HwiP_disable(); /* protect the swap operation*/
    buffIndex = PN_getLastCpmBuffIndex(pnHandle,
                                       pkt->lIndex);

    if(buffIndex >= 0)
    {
        temp = pkt->last;             /*swap the tripple buffer indexes*/
        pkt->last = buffIndex;
        pkt->next = temp;
    }

    HwiP_restore(key);

    if(buffIndex < 0)
    {
        ret = -1;
    }

    if(1 == pkt->validLast)        /*detect overrun*/
    {
        ret = 1;
    }

    pkt->validLast = 1;
    return ret;
}


uint8_t *PN_getLastCpm(PN_Handle pnHandle, t_rtcPacket *pkt)
{
    uint8_t temp;
    uintptr_t key;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    if(pkt->type != CPM)
    {
        return NULL;    /* only usable with CPM packet*/
    }

    if(1 == pkt->validLast)
    {
        key = HwiP_disable(); /* protect the swap operation*/
        temp = pkt->proc;                   /* swap the tripple buffer indexes*/
        pkt->proc = pkt->last;
        pkt->last = temp;
        PN_cpmBuffLock(pruicssHwAttrs, pkt->lIndex, (buffLocks)pkt->proc);   /* lock new PROC buffer*/
        HwiP_restore(key);
        pkt->validLast = 0;
    }

    return pkt->pBuffer->addr[pkt->proc];   /* current RX data address*/
}

/*-----------------------------------------------------
 * legacy mode
 *-----------------------------------------------------*/
#ifdef IRT_LEGACY_STARTUP_SUPPORT

void PN_IRT_legacyTask(uintptr_t arg0, uintptr_t arg1)
{
    PN_Handle pnHandle = (PN_Handle)arg0;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    uint8_t *const pRtcConfig = (uint8_t *)(pruicssHwAttrs->pru0DramBase);
    uint16_t fCycleCount = 0;
    uint16_t    sClk;                   /* current send clock factor*/

    while(pnHandle->legState != ABORT)
    {
        switch(pnHandle->legState)
        {
            case NOINIT:        /* wait for switch up*/
                SemaphoreP_pend(&(pnHandle->switchReady), SystemP_WAIT_FOREVER); /* wait for init*/
                SemaphoreP_post(&(pnHandle->switchReady));
                pnHandle->legState = READY;
                break;

            case READY:         /* wait until PPM packet initialized*/
                /*TODO: Review this*/
                // TaskP_sleep(10); /* need something better here...*/
                ClockP_usleep(10000); /* need something better here...*/
                break;

            case SENDPPM:       /* send as long as CPM is not in RUN*/

                /* check for CPM status first (assume RTC3 is in first descriptor ALWAYS)*/
                if(HW_RD_REG8(pRtcConfig + RTC_CPM_STATUS_OFFSET) == RTC_CPM_RUN)
                {
                    pnHandle->legState = READY;         /* connect done...*/
                    fCycleCount = 0;
                }

                else
                {
                    /* get current base clock*/
                    sClk =  HW_RD_REG16(pRtcConfig + RTC_SCF_OFFSET);

                    /* send a RTC3 PPM through low level PRU API (green phase!)*/
                    if((pnHandle->pLegPkt)->pBuffer->addr[(pnHandle->pLegPkt)->proc] != NULL)
                    {
                        /* write a faked cycle counter with right endianess...*/
                        HW_WR_REG16(((pnHandle->pLegPkt)->pBuffer->addr[(pnHandle->pLegPkt)->proc] +
                               (pnHandle->pLegPkt)->length - 4),
                                   (fCycleCount >> 8 | (fCycleCount & 0xFF) << 8));
                        PN_OS_txPacket(pnHandle->emacHandle,
                                       (pnHandle->pLegPkt)->pBuffer->addr[(pnHandle->pLegPkt)->proc],
                                       (pnHandle->pLegPkt)->port, ICSS_EMAC_QUEUE2,
                                       (pnHandle->pLegPkt)->length);      /* send high prio*/
                        fCycleCount += (sClk * 128);   /* RR=128*/

                        /* TODO: Review this*/
                        // TaskP_sleep(sClk * 4 * 1000 /
                        //            ClockP_getTickPeriod());         /* base clock * 128  period*/
                        ClockP_usleep(ClockP_ticksToUsec(sClk * 4));         /* base clock * 128  period*/
                    }

                    else
                    {
                        pnHandle->legState = READY;    /* bad state!*/
                    }
                }

                break;

            default:
                DebugP_log("undefined LegState\n");
                pnHandle->legState = NOINIT;
        }
    }

    DebugP_log("Legacy process task aborted\n");

    return;
}


void PN_setLegState(void *arg, void *arg2)
{
    PN_Handle pnHandle = (PN_Handle)arg;
    /*TODO: Review this*/
    pnHandle->legState = (tLegStates)(uint32_t)arg2;
}


void PN_setLegPkt(void *arg, void *arg2)
{
    PN_Handle pnHandle = (PN_Handle)arg;

    pnHandle->pLegPkt = (t_rtcPacket *)arg2;
}

#endif /*IRT_LEGACY_STARTUP_SUPPORT*/
/*-----------------------------------------------------
 * MRP support
 *-----------------------------------------------------*/
#ifdef MRP_SUPPORT

void PN_MRP_CPMTask(uintptr_t arg0, uintptr_t arg1)
{
    PN_Handle pnHandle = (PN_Handle)arg0;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    uint8_t *const pRtcConfig = (uint8_t *)(pruicssHwAttrs->pru0DramBase);

    while(1)
    {
        switch(pnHandle->mrpState)
        {
            case MRPREADY:
                /* our IDLE state, all links ok...*/
                break;

            case MRPENTER:
               /* set PRU flush mode*/
                HW_WR_REG8(pRtcConfig + RTC_MRP_FDB_FLUSH_OFFSET, 1);
                /* flush the FDB in the stack as it needs port number*/
                PN_resetCpmPorts(pnHandle);
                pnHandle->mrpState = MRPWAIT;
                break;

            case MRPWAIT:
                /*TODO: Review this*/
                // TaskP_sleep(10 * 1000 / ClockP_getTickPeriod());   /* 10 ms*/
                ClockP_usleep(10 * 1000);   /* 10 ms*/
                pnHandle->mrpState = MRPCHECK;
                break;

            case MRPCHECK:

                /* check CPMs and advance state accordingly*/
                if(PN_allCpmKnown(pnHandle))
                {
                    pnHandle->mrpState = MRPEXIT;
                }

                else
                {
                    pnHandle->mrpState = MRPWAIT;
                }

                break;

            case MRPEXIT:
                /*clear PRU flush mode*/
                HW_WR_REG8(pRtcConfig + RTC_MRP_FDB_FLUSH_OFFSET, 0);
                pnHandle->mrpState = MRPREADY;
                PN_clearMrpFlag(pnHandle);
                break;

            default:
                /* invalid state...*/
                break;
        }
        /*TODO: Review this*/
        // TaskP_sleep(1 * 1000 / ClockP_getTickPeriod());
        ClockP_usleep(1 * 1000);   /* 1 ms*/
    }
}

uint32_t PN_enterFlushMode(PN_Handle pnHandle)
{
    if(MRPREADY == pnHandle->mrpState)
    {
        pnHandle->mrpState = MRPENTER;
        return 0;
    }

    return 1;
}
#endif /*MRP_SUPPORT*/
/*-----------------------------------------------------
 * PTCP support
 *-----------------------------------------------------*/
#ifdef PTCP_SUPPORT

void PN_PTCP_task(uintptr_t arg0, uintptr_t arg1)
{
    PN_Handle pnHandle = (PN_Handle)arg0;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((pnHandle->pruicssHandle)->hwAttrs);

    SemaphoreP_pend(&(pnHandle->switchReady), SystemP_WAIT_FOREVER); /* wait for init*/
    SemaphoreP_post(&(pnHandle->switchReady));

    /*TODO: Review this*/
    // while(!(((ICSS_EmacObject *)(pnHandle->emacHandle)->object)->linkStatus[0] ||
    //         ((ICSS_EmacObject *)(
    //              pnHandle->emacHandle)->object)->linkStatus[1]))             /* wait for link up*/
    while(!((MDIO_phyLinkStatus(pruicssHwAttrs->miiMdioRegBase, ((ICSS_EMAC_Attrs *)((pnHandle->emacHandle)->attrs))->phyAddr[0]) == SystemP_SUCCESS) ||
          (MDIO_phyLinkStatus(pruicssHwAttrs->miiMdioRegBase, ((ICSS_EMAC_Attrs *)((pnHandle->emacHandle)->attrs))->phyAddr[1]) == SystemP_SUCCESS)))   /* wait for link up*/
    {
        /*TODO: Review this*/
        // TaskP_sleep(1);                              /* needs link down management later*/
        ClockP_usleep(ClockP_ticksToUsec(1));           /* needs link down management later*/
    }

    PN_PTCP_smaDelayMeasurement(pnHandle);      /*endless loop*/
}


void PN_PTCP_syncMonitorTask(uintptr_t arg0, uintptr_t arg1)
{
    PN_Handle pnHandle = (PN_Handle)arg0;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)((pnHandle->pruicssHandle)->hwAttrs);

    SemaphoreP_pend(&(pnHandle->switchReady), SystemP_WAIT_FOREVER); /* wait for init*/
    SemaphoreP_post(&(pnHandle->switchReady));

    /*TODO: Review this*/
    // while(!(((ICSS_EmacObject *)(pnHandle->emacHandle)->object)->linkStatus[0] ||
    //         ((ICSS_EmacObject *)(
    //              pnHandle->emacHandle)->object)->linkStatus[1]))             /* wait for link up*/
    while(!((MDIO_phyLinkStatus(pruicssHwAttrs->miiMdioRegBase, ((ICSS_EMAC_Attrs *)((pnHandle->emacHandle)->attrs))->phyAddr[0]) == SystemP_SUCCESS) ||
          (MDIO_phyLinkStatus(pruicssHwAttrs->miiMdioRegBase, ((ICSS_EMAC_Attrs *)((pnHandle->emacHandle)->attrs))->phyAddr[1]) == SystemP_SUCCESS)))   /* wait for link up*/
    {
        /*TODO: Review this*/
        // TaskP_sleep(1);                              /* needs link down management later*/
        ClockP_usleep(ClockP_ticksToUsec(1));           /* needs link down management later*/
    }

    PN_PTCP_syncTimeoutMonitor(pnHandle);       /* endless loop*/
}
#endif /*PTCP_SUPPORT*/

/*-----------------------------------------------------
 *
 *-----------------------------------------------------*/
int32_t PN_OS_txPacket(ICSS_EMAC_Handle icssEmacHandle,
                       const uint8_t *srcAddress, int32_t portNumber, int32_t queuePriority,
                       int32_t lengthOfPacket)
{
    int32_t ret=0;
    ICSS_EMAC_TxArgument txArgs;

    /*TODO: Find appropriate replacement*/
    // llEnter();
    memset(&txArgs, 0, sizeof(ICSS_EMAC_TxArgument));
    txArgs.icssEmacHandle = icssEmacHandle;
    txArgs.lengthOfPacket = lengthOfPacket;
    txArgs.portNumber = portNumber;
    txArgs.queuePriority = queuePriority;
    txArgs.srcAddress = srcAddress;

    ret= ICSS_EMAC_txPacket(&txArgs, NULL);

    /*TODO: Find appropriate replacement*/
    // llExit();

    return ret;
}

/*-----------------------------------------------------
 *
 *-----------------------------------------------------*/
#ifdef WATCHDOG_SUPPORT
void PN_tapWatchDog_task(uintptr_t arg0, uintptr_t arg1)
{
    PN_Handle pnHandle = (PN_Handle)arg0;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    SemaphoreP_pend(&(pnHandle->switchReady), SystemP_WAIT_FOREVER);   /* wait for init*/
    SemaphoreP_post(&(pnHandle->switchReady));

    while(1)                /* wait for watchdog to be enabled*/
    {
        if(pnHandle->icssWatchDogEnabled)
        {
            HW_WR_REG16(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_PD_WD_TIM_REG, pnHandle->icssWachDogTimerPeriod);
        }

        /*TODO: Review this*/
        // TaskP_sleep((pnHandle->icssWachDogTimerPeriod / 10) - (((
        //                pnHandle->icssWachDogTimerPeriod / 10) / 2)));
        ClockP_usleep(ClockP_ticksToUsec((pnHandle->icssWachDogTimerPeriod / 10) -
                      (((pnHandle->icssWachDogTimerPeriod / 10) / 2))));
    }

}
#endif
