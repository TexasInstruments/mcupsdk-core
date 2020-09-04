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
#include "iPnOs.h"
#include "iRtcDrv.h"
#include "iPtcpDrv.h"
#include "iPtcpUtils.h"
#include <drivers/hw_include/hw_types.h>
#include <string.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define PRU_IEP_CMP_CFG_INIT_VALUE 0x00000003

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */


/**
 * @brief Initializes the CPM and PPM descriptor lists.
 * All members of @ref t_descList mapped to the corresponding memory locations/or initialized to 0
 * @param[in] pnHandle Profinet handle
 */
void PN_initLists(PN_Handle pnHandle);

/* ========================================================================== */
/*                 Function Definitions                                       */
/* ========================================================================== */

/* PROFINET interrupt service processing                               */

/**
 * @brief  PPM Interrupt Service Routine. Mapped to interrupt @ref ISR_PPM_NUM_ARM  on ARM
 *
 * Called on PPM TX completed by ICSS. @ref irtPpmCall gives indication to Stack
 * \msc
 * Stack,Driver,ICSS,Net;
 * ...;
 * Stack box Stack [label="data n"];
 * Stack=>Driver [label="PN_insPpmList()", URL="\ref PN_insPpmList()"],
 * Driver=>>ICSS [label="PN_togglePpmList()", URL="\ref PN_togglePpmList()"];
 * ICSS=>ICSS [label="next sendcycle"];
 * ICSS=>ICSS [label="phase match?"];
 * Net box Net [label="TX RTC n"];
 * ICSS=>>Driver [label="PN_ppmIsrHandler()", URL="\ref PN_ppmIsrHandler()"],
 * Driver=>>Stack [label="irtPpmCall()", URL="\ref irtPpmCall()"];
 * ...;
 * Stack box Stack [label="data n+1"],
 * ...;
 * \endmsc
 */
void PN_ppmIsrHandler(void* arg)
{
    PN_Handle pnHandle = (PN_Handle)arg;
    volatile uint8_t *pBcEvent;
    uint8_t index, j;
    uint32_t lockFound = 1;
    const uint8_t maxAR = (pnHandle->currPN).cfgAR;
    t_rtcPacket *ppmArray = (pnHandle->currPN).ppmPkts;        /* array pointer*/
    PN_IntAttrs *ppmIntConfig = &((pnHandle->pnIntConfig).ppmIntConfig);
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    /* clear IRQ source early so same IRQ can queue up*/
    PN_clearPruIRQ(pruicssHwAttrs, ppmIntConfig->pruIntNum);

    do
    {
        for(index = 0; index < maxAR;
                index++)     /* go through all possible event sources*/
        {
            pBcEvent = (pnHandle->ppmList).pEvent + index;

            if((*pBcEvent) > 0)
            {
                for(j = 0; j < maxAR; j++)
                {
                    if(ppmArray[j].isActive && (index == ppmArray[j].lIndex))
                    {
                        PN_checkLastPPM(pnHandle, &(ppmArray[j]));

                        if(ppmIntConfig->callBackPtr != NULL)
                        {
                            pnCallBack_t callBackPtr = (pnCallBack_t)ppmIntConfig->callBackPtr;
                            callBackPtr(&(ppmArray[j]), j);  /* pass PPM object pointer TODO: Pass handle?*/
                        }

                        break;
                    }
                }

#ifdef RTC_DEBUG
                rtcDebug.rtcPPMbc[index]++;
#endif
                *pBcEvent = 0x0;         /* ack processed PPM*/
            }
        }

#ifdef RTC_DEBUG
        rtcDebug.rtcPpmEvent++;
#endif
        /* check again for new events arrived in meantime... */
        lockFound = 0;

        for(index = 0; index < maxAR; index++)
        {
            pBcEvent = (pnHandle->ppmList).pEvent + index;

            if(1 == (*pBcEvent))
            {
                lockFound = 1;
                break;
            }
        }
    }
    while(lockFound);                  /* more events pending?*/
}

/**
 * @brief  CPM Interrupt Service Routine. Mapped to interrupt @ref ISR_CPM_NUM_ARM on ARM
 *
 * @param arg not used
 *
 * Default ISR handler for CPM buffer complete events (new packet received)\n
 * Multiple CPMs may be received back to back or simultaneously (two ports!)\n
 * Stack call back can be used to sync with stack objects
 *
 * Normal CPM RX case:
 * \msc
 * Stack,Driver,ICSS,Net;
 * ...;
 * Net box Net [label="RX RTCx"];
 * Net->ICSS;
 * ICSS=>ICSS [label="FID match?"];
 * ICSS=>>Driver [label="PN_cpmIsrHandler()", URL="\ref PN_cpmIsrHandler()"];
 * Driver=>>Stack [label="irtCpmCall()", URL="\ref irtPpmCall()"];
 * Stack box Stack [label="Process Data"];
 * Driver<<=Stack [label=""];
 * ICSS<<=Driver [label="PN_clearPruIRQ()", URL="\ref PN_clearPruIRQ()"];
 * \endmsc
 *
 */
void PN_cpmIsrHandler(void* arg)
{
    PN_Handle pnHandle = (PN_Handle)arg;
    uint8_t j;
    uint32_t lockFound = 1;
    volatile uint8_t *pBcEvent;
    uint8_t index;
    t_rtcPacket *cpmArray = (pnHandle->currPN).cpmPkts;       /* array pointer*/
    PN_IntAttrs *cpmIntConfig = &((pnHandle->pnIntConfig).cpmIntConfig);
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    PN_clearPruIRQ(pruicssHwAttrs, cpmIntConfig->pruIntNum);

    do
    {
        for(index = 0; index < NO_CPM; index++)
        {
            pBcEvent = (pnHandle->cpmList).pEvent + index;

            if((*pBcEvent) > 0)
            {
                /*found event source*/
                for(j = 0; j < NO_CPM; j++)
                {
                    if(cpmArray[j].isActive && (index == cpmArray[j].lIndex))
                    {
#ifdef MRP_SUPPORT

                        if((pnHandle->ppmList).mrpFlag)
                        {
                            PN_setCpmPort(pnHandle, &(cpmArray[j]));
                        }

#endif /*MRP_SUPPORT*/

                        if(PN_nextCpmRdy(pnHandle, &(cpmArray[j])) >= 0)
                        {
                            if((cpmIntConfig->callBackPtr) != NULL)
                            {
                                pnCallBack_t callBackPtr = (pnCallBack_t)cpmIntConfig->callBackPtr;
                                callBackPtr(&(cpmArray[j]), j);     /* pass packet object to stack*/
                            }
                        }

                        break;
                    }
                }

                /* finally clear BC event of slot*/
                *pBcEvent = 0x0;      /* ack processed CPM*/
#ifdef RTC_DEBUG
                rtcDebug.rtcCpmEvent++;
#endif
            }
        }

        /* check again for new events arrived in meantime... */
        lockFound = 0;

        for(index = 0; index < NO_CPM; index++)
        {
            pBcEvent = (pnHandle->cpmList).pEvent + index;

            if(1 == (*pBcEvent))
            {
                lockFound = 1;
                break;
            }
        }

    }
    while(lockFound);         /*any more events set*/
}

/**
 * @brief  Status Interrupt Service Routine. Mapped to interrupt @ref ISR_DHT_NUM_ARM on ARM

 * @param arg not used
 * \callgraph
 *
 * Called by ICSS for DHT and other status events. Mapped to Interrupt @ref ISR_DHT_NUM_ARM on ARM
 * Triggers on status events
 *  - DHT Expire
 *  - List Changes
 *
 * Data Hold Timer Expired:
 * \msc
 * Stack,Driver,ICSS,Net;
 * Stack=>Driver [label="PN_setCpmDHT(3)", URL="\ref PN_setCpmDHT()"],
 * Driver->ICSS [label="DHT=3"];
 * ...;
 * ICSS=>ICSS [label="WaitCycleStart"];
 * ...;
 * ICSS=>ICSS [label="WaitCycleStart"];
 * ...;
 * ICSS=>ICSS [label="WaitCycleStart"];

 * ICSS box ICSS [label="DHT expired"];
 * ICSS=>>Driver [label="PN_dhtIsrHandler() \n RTC_NOTIFY_DHT_EXPIRE", URL="\ref PN_dhtIsrHandler()"];
 * Driver=>>Stack [label="irtStatCall()", URL="\ref irtStatCall()"];
 * Driver<<=Stack;
 * Driver=>>ICSS [label="Clear IRQ", URL="\ref PN_clearPruIRQ()"];
 * Stack box Stack [label="ALARM"];
 * \endmsc
 * PPM toggle list event:
 * \msc
 * Stack,Driver,ICSS,Net;
 * Stack=>>Driver [label="PN_insPpmList", URL="\ref PN_insPpmList()"],
 * Driver=>>ICSS [label="PN_togglePpmList", URL="\ref PN_togglePpmList()"];
 * ...;
 * ICSS=>ICSS [label="NextCycle"];
 * ICSS=>>Driver [label="PN_dhtIsrHandler() \n RTC_NOTIFY_PPM_LIST_CHANGE", URL="\ref PN_dhtIsrHandler()"];
 * Driver=>>Stack [label="irtStatCall()", URL="\ref irtStatCall()"];
 * Driver<<=Stack;
 * Driver=>>ICSS [label="Clear IRQ", URL="\ref PN_clearPruIRQ()"];
 * Stack box Stack [label="ALARM"];
 * \endmsc
 */
void PN_dhtIsrHandler(void* arg)
{
    PN_Handle pnHandle = (PN_Handle)arg;
    PN_IntAttrs *dhtIntConfig = &((pnHandle->pnIntConfig).dhtIntConfig);
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    int8_t evtStatus;
    t_rtcPacket *initiator = NULL;

#ifdef RTC_DEBUG
    rtcDebug.rtcStatEvent++;
#endif

    evtStatus = PN_getDhtStatusEvent(pnHandle, &initiator);

    if(evtStatus < 0)
    {
#ifdef RTC_DEBUG
        rtcDebug.rtcError[STATERR]++;
#endif
    }

    if(evtStatus == TRUE)
    {
        while(initiator != NULL)
        {
#ifdef RTC_DEBUG
            rtcDebug.dhtError++;           /* debug counter*/
#endif

            if((dhtIntConfig->callBackPtr) != NULL)
            {
                pnCallBack_t callBackPtr = (pnCallBack_t)dhtIntConfig->callBackPtr;
                callBackPtr(initiator, RTC_NOTIFY_DHT_EXPIRE);
            }

            if(PN_getDhtStatusEvent(pnHandle, &initiator) < 0)    /*check for more DHTs*/
            {
#ifdef RTC_DEBUG
                rtcDebug.rtcError[STATERR]++;
#endif
            }
        }
    }

    evtStatus = PN_getListToggleStatusEvent(pnHandle, &initiator);

    if(evtStatus == TRUE)   /* PPM list change flagged*/
    {
        (pnHandle->ppmList).shadow = PN_getShadowIndex(pruicssHwAttrs, PPM);
        PN_emptyList(
            &(pnHandle->ppmList));       /*clear the shadow stuff to prepare for next toggle*/

#ifdef RTC_DEBUG
        rtcDebug.ppmListChanged++;
#endif

        if((dhtIntConfig->callBackPtr != NULL) && (((pnHandle->ppmList).mrpFlag == 0)
                || ((pnHandle->ppmList).listToggleReq == 1)))
        {
            /* Clear the ppm list toggle request flag*/
            PN_setListToggleReq(pnHandle, 0);
            pnCallBack_t callBackPtr = (pnCallBack_t)dhtIntConfig->callBackPtr;
            callBackPtr(initiator, RTC_NOTIFY_PPM_LIST_CHANGE);
        }

        (pnHandle->ppmList).lInitiate = NULL;
        HW_WR_REG8(pruicssHwAttrs->pru0DramBase + RTC_NOTIFY_LIST_TOGGLE_EVENT_OFFSET, 0);
    }

    PN_clearPruIRQ(pruicssHwAttrs, dhtIntConfig->pruIntNum);
}


/***********************************************************************/
/* PROFINET packet processing API                                      */
/***********************************************************************/

int32_t PN_cpmBuffLock(PRUICSS_HwAttrs const *pruicssHwAttrs,
                       uint8_t pos, buffLocks buff)
{
    if(pos >= NO_CPM)
    {
        return -1;    /* pos outside of possible value*/
    }

    /* CPM BUFFER LOCK now one byte per entry!*/
    HW_WR_REG8(pruicssHwAttrs->pru0DramBase + RTC_CPM_BUFFER_LOCK_OFFSET
           + pos, buff);

    return 0;
}

int8_t PN_getLastCpmBuffIndex(PN_Handle pnHandle,
                              uint8_t pos)
{

    int8_t res;
    t_cpmDesc   tmp;
    t_cpmDesc   *tmpDesc = &tmp;

    if(pos >= NO_CPM)
    {
        return -1;    /* pos outside of possible value*/
    }

    res = PN_readCpmDesc(pnHandle, tmpDesc, pos);       /*read from active list*/

    if(res < 0)
    {
        return res;
    }

    return tmpDesc->FrameIndex;

}

int32_t PN_setPPMARlink(PRUICSS_HwAttrs const *pruicssHwAttrs,
                        uint8_t ARgroup, uint8_t PpmNum)
{
    uint8_t *ppmARgroup;
    uint8_t regVal;

    if(--ARgroup >= NO_PPM)     /* change AR group count to 0-7 first*/
    {
        return -1;    /* AR group out of range*/
    }

    if(PpmNum >= NO_PPM)
    {
        return -1;   /* PPM out of range*/
    }

    ppmARgroup = (uint8_t *)(pruicssHwAttrs->pru0DramBase +
                             RTC_AR_GROUP_PPM_SHADOW_OFFSET + ARgroup);
    /* set PPM AR relation*/
    /* clear the corresponding new PPM bit (inverted enable)*/
    /* write to shadow register*/

    regVal = HW_RD_REG8(ppmARgroup);
    regVal &= ~(1 << PpmNum);
    HW_WR_REG8(ppmARgroup, regVal);

    return 0;  /* success*/
}
/*
 * define AR group for CPM
 * direct access to PRU register
 */
int32_t PN_setCPMARlink(PRUICSS_HwAttrs const *pruicssHwAttrs,
                        uint8_t ARgroup, uint8_t CpmNum)
{
    if(--ARgroup >= NO_CPM)
    {
        return -1;    /* AR group out of range*/
    }

    if(CpmNum >= NO_CPM)
    {
        return -1;   /* CPM out of range*/
    }

    /* set CPM AR group relation - a CPM needs to belong to at least one group!*/
    HW_WR_REG8(pruicssHwAttrs->pru0DramBase + RTC_CPM_AR_GROUP_OFFSET +
           CpmNum, ARgroup & 0x07); /* just mask and set..*/

    return 0; /* success*/
}

int32_t PN_resetARlink(PRUICSS_HwAttrs const *pruicssHwAttrs,
                       uint8_t ARgroup, uint8_t pmNum, uint8_t dir)
{
    if(--ARgroup >= NO_CPM)
    {
        return -1;   /* AR group out of range*/
    }

    if(pmNum >= NO_CPM)
    {
        return -2;   /* PM out of range*/
    }

    if(dir >= 2)                      /* only CPM or PPM allowed*/
    {
        return -3;
    }

    if(dir == PPM)
    {
        /* reset PPM AR relation*/
        uint8_t oldPpmAR = HW_RD_REG8(pruicssHwAttrs->pru0DramBase +
                                  RTC_AR_GROUP_PPM_OFFSET + ARgroup);   /* get current group PPM relation*/
        /* set the corresponding PPM bit to remove from group (inverted enable)*/
        HW_WR_REG8(pruicssHwAttrs->pru0DramBase +
               RTC_AR_GROUP_PPM_SHADOW_OFFSET + ARgroup, oldPpmAR & (~(1 << pmNum)));
    }

    else         /* CPM processing*/
    {
        /* reset CPM AR group relation - a CPM needs to belong to at least one group!*/
        HW_WR_REG8(pruicssHwAttrs->pru0DramBase + RTC_CPM_AR_GROUP_OFFSET +
               pmNum, 0x0);   /*just set to 0 - assume CPM is not active!*/
    }

    return 0;    /*success*/
}

#ifdef MRP_SUPPORT

uint32_t PN_allCpmKnown(PN_Handle pnHandle)
{
    uint32_t i;
    t_rtcPacket *cpmArray = (pnHandle->currPN).cpmPkts;        /*array pointer*/

    for(i = 0; i < NO_CPM; i++)
    {
        if(cpmArray[i].isActive)           /* check only active CPMs...*/
        {
            if(0 == cpmArray[i].port)
            {
                return 0;
            }
        }
    }

    return 1; /* no CPM active or all CPM structs contain a 'port' value != 0*/
}

void PN_setCpmPort(PN_Handle pnHandle, t_rtcPacket *pID)
{
    uint8_t j, chgFlag = 0;
    t_cpmDesc desc;
    t_rtcPacket *ppmArray = (pnHandle->currPN).ppmPkts;       /* array pointer*/

    if(0 != PN_readCpmDesc(pnHandle, &desc,
                           pID->lIndex))      /*get valid descriptor*/
    {
        return;
    }

    pID->port = ((desc.FrameFlags2 & 0x02) >> 1) +
                1; /* retrieve RX port number of CPM*/

    for(j = 0; j < (pnHandle->currPN).cfgAR; j++)
    {
        if(ppmArray[j].crGroup == pID->crGroup)         /* matching group PPM*/
            if(ppmArray[j].isActive && (ppmArray[j].port != pID->port))
            {
                ppmArray[j].port = pID->port;           /*change PPM to new port*/
                chgFlag = 1;
            }
    }

    if(chgFlag)
    {
        if(PN_writeSortedList(pnHandle, ppmArray) < 0)
        {
            goto MRP_ERROR;
        }

        if(PN_togglePpmList(pnHandle, ppmArray, 1) < 0)
        {
            goto MRP_ERROR;    /*switch to new list finally and run!*/
        }
    }

    return;

MRP_ERROR:
    pID->port =
        ICSS_EMAC_PORT_0;                  /*reset CPM port so next RX should work...*/
    return;
}


void PN_resetCpmPorts(PN_Handle pnHandle)
{
    uint32_t i;
    t_rtcPacket *cpmArray = (pnHandle->currPN).cpmPkts;       /* array pointer*/

    for(i = 0; i < NO_CPM; i++)
    {
        cpmArray[i].port = ICSS_EMAC_PORT_0;          /*just clear all*/
    }

    (pnHandle->ppmList).mrpFlag = 1;              /* activate MRP mode*/

}
#endif /*MRP_SUPPORT*/


int32_t PN_setCpmDHT(PRUICSS_HwAttrs const *pruicssHwAttrs,
                     uint16_t dht, uint8_t pos)
{

    if(pos >= NO_CPM)
    {
        return -2;    /* descriptor position out of range*/
    }

    HW_WR_REG16(pruicssHwAttrs->pru0DramBase + RTC_DHT_TIMEOUT_OFFSET +
           pos * 2, dht);

    return 0;
}
/* only use this with DHT ISR*/
int8_t PN_getDhtStatusEvent(PN_Handle pnHandle, t_rtcPacket **pktID)
{
    uint8_t i, j, src;
    uint8_t maxAR = (pnHandle->currPN).cfgAR;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);


    /* new multi DHT processing...*/
    if(RTC_NOTIFY_DHT_EXPIRE == HW_RD_REG8(pruicssHwAttrs->pru0DramBase +
                                       RTC_NOTIFY_DHT_EVENT_OFFSET))
    {
        for(i = 0; i < maxAR; i++)
        {
            /*walk through the array*/
            src = HW_RD_REG8(pruicssHwAttrs->pru0DramBase + RTC_NOTIFY_DHT_EXPIRE_OFFSET + i);

            if(1 == src)
            {
                HW_WR_REG8(pruicssHwAttrs->pru0DramBase + RTC_NOTIFY_DHT_EXPIRE_OFFSET + i, 0xFF); /* auto clear the source!*/

                for(j = 0; j < maxAR; j++)
                {
                    if((pnHandle->currPN).cpmPkts[j].lIndex == i)
                    {
                        *pktID = &((pnHandle->currPN).cpmPkts[j]);
                        return 1;      /* always exit on first new DHT source found*/
                    }
                }

                *pktID = NULL;
                HW_WR_REG8(pruicssHwAttrs->pru0DramBase + RTC_NOTIFY_DHT_EVENT_OFFSET, 0);
                return -1;             /*no object match*/
            }
        }

        /*no more DHT sources found... so return ok with no packet*/
        *pktID = NULL;
        HW_WR_REG8(pruicssHwAttrs->pru0DramBase + RTC_NOTIFY_DHT_EVENT_OFFSET, 0);
        return 1;
    }

    else           /*all other Status ISR cases...*/
    {
        *pktID = NULL;
    }

    return 0;
}

/*only use this with DHT ISR*/
int8_t PN_getListToggleStatusEvent(PN_Handle pnHandle, t_rtcPacket **pktID)
{

    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);


    if(RTC_NOTIFY_PPM_LIST_CHANGE == HW_RD_REG8(pruicssHwAttrs->pru0DramBase +
                                            RTC_NOTIFY_LIST_TOGGLE_EVENT_OFFSET))
    {
        *pktID = (pnHandle->ppmList).lInitiate;
        return 1;
    }

    else         /* all other Status ISR cases...*/
    {
        *pktID = NULL;
    }

    return 0;
}

int32_t PN_getPmStatus(PRUICSS_HwAttrs const *pruicssHwAttrs,
                       uint8_t dir, uint8_t numPm)
{
    if(numPm >= NO_CPM)
    {
        return -1;  /* descriptor position out of range*/
    }

    if(dir >= 2)                /* only CPM or PPM allowed*/
    {
        return -2;
    }

    /* first get descriptor at pos*/
    if(dir == CPM)             /* packet buffer might be different for PPM and CPM in future*/
    {
        return HW_RD_REG8(pruicssHwAttrs->pru0DramBase +
                      RTC_CPM_STATUS_OFFSET);
    }

    else
    {
        return HW_RD_REG8(pruicssHwAttrs->pru0DramBase +
                      RTC_PPM_STATUS_OFFSET);
    }
}

int32_t PN_writeSortedList(PN_Handle pnHandle, t_rtcPacket *pPkts)
{
    uint8_t pos = 0, i;
    uint16_t fId;
    t_listIndex index;
    uint8_t maxAR = (pnHandle->currPN).cfgAR;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    for(i = 0; i < NO_CPM; i++)
    {
        HW_WR_REG8((uint8_t *)(pruicssHwAttrs->pru0DramBase +
                           RTC_AR_GROUP_PPM_SHADOW_OFFSET + i), 0xFF);
    }

   /* create list index from scratch*/
    index.end = 0;
    index.green = 0;
    index.greenP2 = 0;
    index.redP2 = 0;

    /* first search for a RED RTC3 packet*/
    /* need to adapt for MRPD later (redundant packet with FID+1)*/
    for(i = 0; i < maxAR; i++)
    {
        if(0 == pPkts[i].isActive)
        {
            continue;     /* inactive, skip*/
        }

        fId = pPkts[i].frameId;

        if(fId == 0 || fId == 0xFFFF || fId != (pnHandle->ppmList).redFID)
        {
            continue;     /* wrong frame ID, skip*/
        }

        /* found RTC3 but is it the RED*/
        if(pPkts[i].frameOffset == 0)
        {
            continue;     /* no... legacy RTC3 only*/
        }

        if(PN_writePpmDesc(pnHandle, &pPkts[i], pos) < 0)
        {
            return -1;
        }

        pPkts[i].lIndex = pos;       /* descriptor index after toggle...*/
        PN_setPpmState(pruicssHwAttrs, &pPkts[i]);
        pos++;

        if(pPkts[i].port == ICSS_EMAC_PORT_1)
        {
            index.redP2 += 1;     /* not a P2...*/
        }

        index.green += 1;            /* just increment the others...*/
        index.greenP2 += 1;
        index.end += 1;
        i = NO_PM;                   /* force exit of loop so we process max 1 RED*/
    }

    /* now process all green packets port 1*/
    for(i = 0; i < maxAR; i++)
    {
        if(0 == pPkts[i].isActive)
        {
            continue;     /* inactive, skip*/
        }

        fId = pPkts[i].frameId;

        if(fId == 0 || fId == 0xFFFF || pPkts[i].port == ICSS_EMAC_PORT_2)
        {
            continue;     /* wrong frame ID or port, skip*/
        }

        if(fId == (pnHandle->ppmList).redFID && pPkts[i].frameOffset != 0)
        {
            continue;     /* RED frame, skip*/
        }

        if(PN_writePpmDesc(pnHandle, &pPkts[i], pos) < 0)
        {
            return -1;
        }

        pPkts[i].lIndex = pos;       /* descriptor index after toggle...*/
        PN_setPpmState(pruicssHwAttrs, &pPkts[i]);
        pos++;                       /* advance to next list position*/

        index.greenP2 += 1;          /* increment port2 index*/
        index.end += 1;              /* always increment end*/
    }

    /* now do green/port 2*/
    for(i = 0; i < maxAR; i++)
    {
        if(0 == pPkts[i].isActive)
        {
            continue;    /* inactive, skip*/
        }

        fId = pPkts[i].frameId;

        if(fId == 0 || fId == 0xFFFF || pPkts[i].port == ICSS_EMAC_PORT_1)
        {
            continue;   /* wrong frame ID or port, skip*/
        }

        if(fId == (pnHandle->ppmList).redFID && pPkts[i].frameOffset != 0)
        {
            continue;    /* RED frame, skip*/
        }

        if(PN_writePpmDesc(pnHandle, &pPkts[i], pos) < 0)
        {
            return -1;
        }

        pPkts[i].lIndex = pos;      /*descriptor index after toggle...*/
        PN_setPpmState(pruicssHwAttrs, &pPkts[i]);
        pos++;                      /* advance to next list position*/

        index.end += 1;             /* always increment end*/
    }

    PN_setIndexInt(pnHandle, &index);          /* write to shadow*/

    return 0;
}

int32_t PN_writePpmDesc(PN_Handle pnHandle, t_rtcPacket *pPkt, uint8_t pos)
{
    t_ppmDesc   newDesc;
    uint32_t        *dstAddr;

    /* next only works as long as NO_PPM = NO_CPM = NO_PM*/
    if(pos >= NO_PM)            /* pos should point to next free position here*/
    {
        return -21;    /* list is full already*/
    }

    dstAddr = (uint32_t *)pPkt->pBuffer->addr[pPkt->proc];  /*PRU0 buffer address*/

    /*this address is PRU specific and therefore does not support EDMA buffer copy!*/
    newDesc.FrameReference = (uint32_t)dstAddr &
                         0xFFFF;    /*start address of current packet buffer*/
    newDesc.FrameLength = pPkt->length;
    /* store FID for identification ... little endian conversion for direct compare with packet stream*/
    newDesc.FrameId = ((pPkt->frameId & 0xFF00) >> 8) + ((pPkt->frameId & 0xFF) <<
                  8); /* aligned with CPM*/
    newDesc.FrameIndex = pPkt->proc;
    newDesc.Phase = pPkt->phase - 1;
    newDesc.RR = pPkt->reduRatio - 1;   /* PRU reduction ratio = RR-1*/

    if(pPkt->frameOffset == 0)      /*/ green?*/
    {
        newDesc.FrameFlags1 = 0;
    }

    else
    {
        newDesc.FrameFlags1 = 1;    /* red packet*/
    }

    newDesc.FrameSendOffset = pPkt->frameOffset & 0x3FFFFF;

    /*TODO: Review this*/
    uint32_t *addr = (pnHandle->ppmList).pDescs[(pnHandle->ppmList).shadow];
    addr += pos * DESC_WORD_LENGTH;                    /* advance to correct offset*/
    uint32_t *dest = (uint32_t *)&newDesc;                              /* portability?*/
    *addr++ = *dest++;                                  /* copy 16 byte decriptor as 4 words*/
    *addr++ = *dest++;
    *addr++ = *dest++;
    *addr = *dest;

    return 0;                  /* success*/
}


int32_t PN_writeCpmDesc(PN_Handle pnHandle, t_rtcPacket *pPkt, uint8_t pos)
{
    t_cpmDesc   newDesc;

    /* next only works as long as NO_PPM = NO_CPM = NO_PM*/
    if(pos >= NO_PM)                /* pos should point to next free position here*/
    {
        return -21;    /* list is full already*/
    }

    if((void *)0xFFFFFFFF == pPkt->pBuffer)
    {
        newDesc.FrameReference = 0x0;           /* special case to clear descriptor*/
    }

    else
    {
        /*new descriptor requires a pointer to three buffer addresses*/
        newDesc.FrameReference =
            pPkt->pBuffer->regbase;    /*previously assigned - slot specific*/
    }

    newDesc.FrameLength =
        pPkt->length; /* length should have space for FCS and VLAN TAG??*/
    /* store FID for identification ... little endian conversion for direct compare with packet stream*/
    newDesc.FrameId = ((pPkt->frameId & 0xFF00) >> 8) + ((pPkt->frameId & 0xFF) << 8);
    newDesc.Phase = pPkt->phase;
    newDesc.RR = pPkt->reduRatio - 1;
    newDesc.FrameDataPointer = 0;       /*clear data that will be set by PRU on RX*/
    newDesc.FrameIndex = 0;
    newDesc.FrameFlags1 = 0;
    newDesc.FrameFlags2 = 0;
    newDesc.Reserved = 0;

    uint32_t *addr =
        (pnHandle->cpmList).pDescs[0];                    /*only one active list!*/
    addr += pos * DESC_WORD_LENGTH;                    /* advance to correct offset*/
    uint32_t *dest = (uint32_t *)&newDesc;                             /*portability?*/
    *addr++ = *dest++;                                  /* copy 16 byte decriptor as 4 words*/
    *addr++ = *dest++;
    *addr++ = *dest++;
    *addr = *dest;

    return 0;                   /*success*/
}


int32_t PN_readPpmDesc(PN_Handle pnHandle, t_ppmDesc *pDesc, uint8_t pos,
                       uint8_t act)
{
    uint32_t *descAddr;
    uint8_t active;

    if(pDesc == NULL)
    {
        return -1;
    }

    active = (pnHandle->ppmList).shadow;

    if(act == ACTIVE_LIST)
    {
        active ^= 1;
    }

    descAddr = (pnHandle->ppmList).pDescs[active] + pos * DESC_WORD_LENGTH;
    memcpy(pDesc, descAddr, DESC_WORD_LENGTH *
           4);      /* copy descriptor from PRU-ICSS*/
    return 0;
}

int32_t PN_readCpmDesc(PN_Handle pnHandle, t_cpmDesc *pDesc, uint8_t pos)
{
    uint32_t *descAddr;

    if(pDesc == NULL)
    {
        return -1;
    }


    descAddr = (pnHandle->cpmList).pDescs[0] + pos * DESC_WORD_LENGTH;
    memcpy(pDesc, descAddr, DESC_WORD_LENGTH *
           4);      /* copy descriptor from PRU-ICSS*/
    return 0;
}

int32_t PN_setBaseClock(PN_Handle pnHandle, uint16_t factor)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);


    if((factor > 128) || (factor < 8)
            || (factor & (factor - 1)))       /*Checks power of 2*/
    {
        return -1;
    }

    HW_WR_REG32(pruicssHwAttrs->pru0DramBase + RTC_BASE_CLK_OFFSET, RTC_3125_CLK_CONST
            * factor);
    HW_WR_REG16(pruicssHwAttrs->pru0DramBase + RTC_SCF_OFFSET, factor);

    /* Signal to PRU that cycle time has changed*/
    HW_WR_REG8(pruicssHwAttrs->pru0DramBase + RTC_BASE_CLK_CHANGED_OFFSET, 1);

#ifdef PTCP_SUPPORT
    /*inform PTCP regarding clock change and change in CMP0 of IEP*/
    PN_PTCP_ClockChange(pnHandle, RTC_3125_CLK_CONST * factor);
#endif

    return 0;
}

void PN_initLists(PN_Handle pnHandle)
{
    uint32_t i;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    /* CPM/PPM descriptors in PRU0 DMEM*/
    (pnHandle->cpmList).pDescs[0] = (uint32_t *)(pruicssHwAttrs->pru0DramBase +
                                     RTC_CPM_IDX0_OFFSET);
    (pnHandle->cpmList).pDescs[1] =
        NULL;          /* unused! CPM doesn't have shadow list management*/
    (pnHandle->ppmList).pDescs[0] = (uint32_t *)(pruicssHwAttrs->pru0DramBase +
                                     RTC_PPM_IDX0_OFFSET);
    (pnHandle->ppmList).pDescs[1] = (uint32_t *)(pruicssHwAttrs->pru0DramBase +
                                     RTC_PPM_IDX1_OFFSET);
    /* index structure pointers*/
    (pnHandle->cpmList).rtc_index_ptr[0] = NULL;       /* not used*/
    (pnHandle->cpmList).rtc_index_ptr[1] = NULL;       /* not used*/
    (pnHandle->ppmList).rtc_index_ptr[0] = (uint8_t *)(
            pruicssHwAttrs->pru0DramBase
            + RTC_PPM_INDEX_L1_OFFSET);
    (pnHandle->ppmList).rtc_index_ptr[1] = (uint8_t *)(
            pruicssHwAttrs->pru0DramBase
            + RTC_PPM_INDEX_L2_OFFSET);

    /* desc active field*/
    (pnHandle->cpmList).pActive = (uint8_t *)(pruicssHwAttrs->pru0DramBase
                                   + RTC_CPM_ACTIVE_OFFSET);
    (pnHandle->ppmList).pActive = (uint8_t *)(pruicssHwAttrs->pru0DramBase +
                                   RTC_PPM_ACTIVE_OFFSET);
    /* desc status*/
    (pnHandle->cpmList).pStatus = (uint8_t *)(pruicssHwAttrs->pru0DramBase +
                                   RTC_CPM_STATUS_OFFSET);
    (pnHandle->ppmList).pStatus = (uint8_t *)(pruicssHwAttrs->pru0DramBase +
                                   RTC_PPM_STATUS_OFFSET);
    /* desc buffer complete events*/
    /* CPM: buffer complete evnet on RX*/
    (pnHandle->cpmList).pEvent = (uint8_t *)(pruicssHwAttrs->pru0DramBase +
                                  RTC_CPM_BC_EVENT_OFFSET);
    (pnHandle->ppmList).pEvent = (uint8_t *)(pruicssHwAttrs->pru0DramBase +
                                  RTC_PPM_SEND_STATUS_OFFSET);

    /*sizes*/
    (pnHandle->cpmList).size = RTC_CPM_LIST_SIZE;
    (pnHandle->ppmList).size = RTC_PPM_LIST_SIZE;
    /* reset last initiator*/
    (pnHandle->cpmList).lInitiate = NULL;
    (pnHandle->ppmList).lInitiate = NULL;
    /* reset MRP flag*/
    (pnHandle->cpmList).mrpFlag = 0;
    (pnHandle->ppmList).mrpFlag = 0;

    (pnHandle->cpmList).listToggleReq = 0;
    (pnHandle->cpmList).listToggleReq = 0;

    /* clear DHT event sources (8 bytes)*/
    for(i = 0; i < NO_CPM; i++)
    {
        /* walk through the array*/
        HW_WR_REG8(pruicssHwAttrs->pru0DramBase + RTC_NOTIFY_DHT_EXPIRE_OFFSET + i,
            0xFF);
    }

    HW_WR_REG16(((uint8_t *)(pruicssHwAttrs->pru0DramBase + RTC_LIST_INDEX_OFFSET)),
        0x0100);       /* PRU uses lists 1, host lists 2 initially*/
}

int32_t PN_clearList(PN_Handle pnHandle, uint8_t list)
{
    t_descList *pList;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    int i;

    if(list > PPM)
    {
        return -1;    /* wrong parameter*/
    }


    if(list == PPM)         /* store active*/
    {
        pList = &(pnHandle->ppmList);
        /* reset both (active and shadow) lists!*/
        *(pList->rtc_index_ptr[0] + RTC_GREEN_IDX)      = 0;    /* reset all values*/
        *(pList->rtc_index_ptr[0] + RTC_RED_PORT_IDX)   = 0;
        *(pList->rtc_index_ptr[0] + RTC_GREEN_PORT_IDX) = 0;
        *(pList->rtc_index_ptr[0] + RTC_GREEN_END_IDX)  = 0;
        *(pList->rtc_index_ptr[1] + RTC_GREEN_IDX)      = 0;    /* reset all values*/
        *(pList->rtc_index_ptr[1] + RTC_RED_PORT_IDX)   = 0;
        *(pList->rtc_index_ptr[1] + RTC_GREEN_PORT_IDX) = 0;
        *(pList->rtc_index_ptr[1] + RTC_GREEN_END_IDX)  = 0;

        for(i = 0; i < NO_CPM; i++)
        {
            HW_WR_REG8(pruicssHwAttrs->pru0DramBase + RTC_PPM_ACTIVE_OFFSET + i, 0);
        }

    }

    else
    {
        pList = &(pnHandle->cpmList);
        *pList->pActive = 0;    /* reset all CPM active bits*/
    }

    if(list == PPM)         /* Shadow index valid only for PPM*/
    {
        pList->shadow = PN_getShadowIndex(pruicssHwAttrs,
                                          list);    /* set shadow according to PRU state*/
    }

    if(list == CPM)
    {
        pList->pDescs[0] = (uint32_t *)(pruicssHwAttrs->pru0DramBase +
                                        RTC_CPM_IDX0_OFFSET);

        /* clear the AR relation - iterate over CPM array*/
        for(i = 0; i < NO_CPM; i++)
        {
            HW_WR_REG8(pruicssHwAttrs->pru0DramBase + RTC_CPM_AR_GROUP_OFFSET + i, 0);
        }
    }

    else
    {
        pList->pDescs[0] = (uint32_t *)(pruicssHwAttrs->pru0DramBase +
                                        RTC_PPM_IDX0_OFFSET);
        pList->pDescs[1] = (uint32_t *)(pruicssHwAttrs->pru0DramBase +
                                        RTC_PPM_IDX1_OFFSET);

        /* clear the AR relation - iterate over AR group array*/
        for(i = 0; i < NO_CPM; i++)
        {
            HW_WR_REG8(pruicssHwAttrs->pru0DramBase + RTC_AR_GROUP_PPM_OFFSET + i, 0x00);
        }
    }

    return 0;
}

int8_t PN_getShadowIndex(PRUICSS_HwAttrs const *pruicssHwAttrs, uint8_t lType)
{
    uint8_t listStat = HW_RD_REG8((uint8_t *)(
                                  pruicssHwAttrs->pru0DramBase +
                                  RTC_LIST_INDEX_OFFSET));   /* get the  PRU act field*/

    if(lType == PPM)
    {
        if(listStat != 0)
        {
            return 0;    /* PRU has List 2 active so return 1 for shadow*/
        }

        else
        {
            return 1;
        }
    }

    else
    {
        return -1;
    }
}

int32_t PN_emptyList(t_descList *pList)
{
    if((pList == NULL) || (pList->shadow > 1))
    {
        return -1;
    }

    if(pList->pDescs[pList->shadow] == NULL)
    {
        return -2;
    }

    memset(pList->pDescs[pList->shadow], 0, DESC_WORD_LENGTH * 4);
    return 0;
}

int32_t PN_togglePpmList(PN_Handle
                         pnHandle, t_rtcPacket *pktID, uint8_t ppmMrpPortShift)
{
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);


    uintptr_t key;
    key = HwiP_disable();

    if((pnHandle->ppmList).lInitiate != NULL)
    {
        return -1;    /* refuse the toggle as other toggle still in operation*/
    }

    if((pnHandle->ppmList).shadow == 1)
    {
        HW_WR_REG8((uint8_t *)(pruicssHwAttrs->pru0DramBase + RTC_LIST_INDEX_OFFSET) + 1
            , 0x0);   /* set to index 0*/
    }

    else
    {
        HW_WR_REG8((uint8_t *)(pruicssHwAttrs->pru0DramBase + RTC_LIST_INDEX_OFFSET) + 1
            , 0x01);    /* set to index 1*/
    }

    (pnHandle->ppmList).lInitiate =
        pktID;             /* save pointer reference to initiator object*/

    /* If ppm list toggle is initiated by the insert or delete ppm then set the listToggleReq*/
    if(ppmMrpPortShift == 0)
    {
        PN_setListToggleReq(pnHandle, 1);
    }

    HwiP_restore(key);

    return 0;
}

int32_t PN_initRtcDrv(PN_Handle pnHandle)
{
    uint32_t iepCmpCfg = 0;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);


    if(TRUE == pnHandle->initRtcDrvFlag)                /* do all singleton inits*/
    {
        /* setups for PROFINET RTC*/
        PN_initLists(pnHandle);         /* RTC init!*/
        PN_clearList(pnHandle, PPM);
        PN_clearList(pnHandle, CPM);
        /* always set the base clock before starting the PRUs!!!*/
        /* Initializing with the Cycle Time of 250us.*/

        HW_WR_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP0_REG0, RTC_3125_CLK_CONST * 8);
        iepCmpCfg = HW_RD_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG);

        iepCmpCfg = iepCmpCfg | PRU_IEP_CMP_CFG_INIT_VALUE;
        HW_WR_REG32(pruicssHwAttrs->iep0RegBase + CSL_ICSS_G_PR1_IEP0_SLV_CMP_CFG_REG, iepCmpCfg);

        PN_setBaseClock(pnHandle,
                        8);       /* 8*31.25us = 250us - default for IRT/RT devices without performance options*/

        if(PN_RTC_setupIsr(pnHandle) < 0)
        {
            return -1;   /* couldn't setup ISRs*/
        }

        pnHandle->initRtcDrvFlag = FALSE;
    }

    else
    {
        return -2;   /* already initialized before!*/
    }

    return 0;
}

void PN_registerPpmCall(PN_Handle pnHandle, pnCallBack_t callBack)
{
    PN_IntAttrs *ppmIntConfig = &((pnHandle->pnIntConfig).ppmIntConfig);
    ppmIntConfig->callBackPtr = (void *)callBack;
}

void PN_registerCpmCall(PN_Handle pnHandle, pnCallBack_t callBack)
{
    PN_IntAttrs *cpmIntConfig = &((pnHandle->pnIntConfig).cpmIntConfig);
    cpmIntConfig->callBackPtr = (void *)callBack;
}

void PN_registerStatCall(PN_Handle pnHandle, pnCallBack_t callBack)
{
    PN_IntAttrs *dhtIntConfig = &((pnHandle->pnIntConfig).dhtIntConfig);
    dhtIntConfig->callBackPtr = (void *)callBack;
}

/* -------------------------------------------------------------------------- */
/*                 Internal Function Definitions                              */
/* -------------------------------------------------------------------------- */

int32_t PN_setIndexInt(PN_Handle pnHandle, t_listIndex *index)
{
    uint8_t lstIndx;

    if(NULL == index)
    {
        return -88;
    }

    lstIndx = (pnHandle->ppmList).shadow;      /* get current shadow list index*/

    *((pnHandle->ppmList).rtc_index_ptr[lstIndx] + RTC_RED_PORT_IDX) = index->redP2
            * 16;
    *((pnHandle->ppmList).rtc_index_ptr[lstIndx] + RTC_GREEN_IDX) = index->green *
            16;
    *((pnHandle->ppmList).rtc_index_ptr[lstIndx] + RTC_GREEN_PORT_IDX) =
        index->greenP2 * 16;
    *((pnHandle->ppmList).rtc_index_ptr[lstIndx] + RTC_GREEN_END_IDX) = index->end
            * 16;

    return 0;
}
void PN_clearMrpFlag(PN_Handle pnHandle)
{

    (pnHandle->ppmList).mrpFlag = 0;

    return;
}

void PN_setListToggleReq(PN_Handle pnHandle, uint8_t enable)
{

    (pnHandle->cpmList).listToggleReq = enable;

    return;
}
