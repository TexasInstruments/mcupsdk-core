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
#include "pnDrvConfig.h"
#include "PN_Handle.h"
#include "PN_HandleDef.h"
#include "iPNLegacy.h"
#include "iRtcDrv.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                             Function Declaration                                  */
/* ========================================================================== */

/* ========================================================================== */
/*                             Local Variables                                */
/* ========================================================================== */
/*
 * Internal RTC buffer allocation and management
 *
 * Version 2 of the RTC drivers extend and simplify the allocation and management
 * of buffers used for PPM and CPM packet data. First step is to increase the
 * available packet memory from single 1440 bytes payload packets to a maximum
 * of two 1440 bytes packets. For CPM buffers this requires to release twice the
 * memory inside L3 SRAM. However for PPM we now need to split up the available
 * buffer area into two blocks allocated in PRU0 and PRU1 data memory. To allow
 * a simpler management of memory the two blocks are used to allocate fixed
 * triple buffer scheme. First version of RTC driver had more flexibility but
 * was unnecessarily complex in memory management. The new scheme still fulfills
 * all current requirements of Profinet ARs within the limits of available internal
 * memory.
 *
 * Depending on Profinet device configuration maxima (from GSD) it is now required
 * to configure the buffer configuration at initialization time. This will define
 * the amount of available PPM/CPM slots and their maximum data sizes.
 * Examples:
 *
 *  2 AR - each with 1440 bytes payload
 *  4 AR - each 720 bytes
 *  8 AR - each 360 bytes
 *  3 AR - 1x 1440 bytes + 2x 720 bytes
 *
 *  More combinations are possible with the restriction that a single PPM buffer
 *  (3x length) must fit into a single buffer block.
 *  PPM uses two buffer blocks of identical size. Block 0 resides in Data RAM0 and
 *  Block 1 resides in Data RAM1. Due to the fact that PRU Data RAM blocks are swapped
 *  by PRU we now need to adapt buffer start addresses depending on packet Port
 *  allocation. As PRU1 processes Port 1 TX and PRU0 processes Port 2 TX we now
 *  have the following table of start addresses:
 *
 *    TX Port   Buffer Block    Startaddress    Process Unit
 *    ------------------------------------------------------------------
 *    P1        B0              0x2E00          PRU1
 *    P1        B1              0x0000          PRU1
 *    P2        B0              0x0E00          PRU0
 *    P2        B1              0x2000          PRU0
 *
 */


/* ========================================================================== */
/*                             Local Functions                                */
/* ========================================================================== */

/**
 * @brief Initializes the CPM and PPM blocsk
 * @param[in] pnHandle Profinet HAndle
 */
static void PN_initRtcBlocks(PN_Handle pnHandle);

/**
 * @internal
 * @brief Helper function to setup triple buffers
 * @param pBuffDesc     Buffer pointer
 * @param offset        Offset
 * @param size          size to clear
 *
 */
void PN_initRtcBuffs(t_rtcMemBuff *pBuffDesc, uint8_t *offset, uint16_t size);

/**
 * @internal
 * @brief Internal function to detect next free block in buffer arrays
 * @param pnHandle Profinet Handle
 * @param[in] pmType        @ref PPM or @ref CPM
 * @param[out] pBlock   block number
 *
 * @retval 0 on success
 * @retval <0 if failure
 */
int8_t PN_findFreeSlot(PN_Handle pnHandle, uint8_t pmType, uint8_t *pBlock);

/* ========================================================================== */
/*                             Function Definitions                           */
/* ========================================================================== */

int32_t PN_cfgRtcMem(PN_Handle pnHandle, uint8_t ar, uint16_t size)
{
    uint8_t loops, i;
    uint16_t cpmSize, ppmSize;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    if(pnHandle->initRtcMemFlag)
    {
        return -1;    /* singleton, we are already configured */
    }

    PN_initRtcBlocks(pnHandle); /* here we are sure it will happen exactly once*/

    /* static param checks*/
    if(ar > NO_PPM || ar == 0)
    {
        return -2;    /* out of bounds*/
    }

    if(size > PPM_IO_DATA_SIZE)     /* 1440...*/
    {
        return -3;    /* maximum payload size exceeded, out of spec*/
    }

    /* hard coded support cases*/
    switch(ar)
    {
        case 1:     /* special case without meaning for IRT - treat as ar = 2*/
            loops = 2;
            break;

        case 2:     /* 2 full AR possible*/
            loops = ar;
            break;

        case 4:
            if(size > PPM_IO_DATA_SIZE / 2)
            {
                return -3;    /* available payload size exceeded*/
            }

            loops = ar;
            break;

        case 8:
            if(size > PPM_IO_DATA_SIZE / 4)
            {
                return -3;    /* available payload size exceeded*/
            }

            loops = ar;
            break;

        default:
            return -2;      /* again out of bounds*/
    }

    /* due to limits in RTC descriptor all buffer addresses need to be word aligned*/
    /* this is forced by aligning size to multiple of 4*/
    if(size % 4 > 0)
    {
        size = (size + 3) & 0xFFFC;
    }

    /*add RTC trailer/header sizes to get full buffer size*/
    ppmSize = size + PPM_ETH_HEADER + PPM_TRAILER;
    cpmSize = size + CPM_ETH_HEADER + CPM_TRAILER;

    /* now assign triple buffer start addresses*/
    for(i = 0; i < loops / 2; i++) /* only works for loop>1!*/
    {
        PN_initRtcBuffs(&((pnHandle->ppmBlock[0]).slots[i]),
                        ((pnHandle->ppmBlock[0]).blockStart) + ppmSize * 3 * i, ppmSize);
        PN_initRtcBuffs(&((pnHandle->ppmBlock[1]).slots[i]),
                        ((pnHandle->ppmBlock[1]).blockStart) + ppmSize * 3 * i, ppmSize);
    }

    for(i = 0; i < loops; i++)
    {
        PN_initRtcBuffs(&((pnHandle->cpmBlock).slots[i]),
                        ((pnHandle->cpmBlock).blockStart) + cpmSize * 3 * i, cpmSize);
        /* store the CPM buffer address array base (lower 16 bit only - for direct re-use in descriptor)*/
        (pnHandle->cpmBlock).slots[i].regbase = ((uint32_t)
                pruicssHwAttrs->pru0DramBase + RTC_CPM_BUFFER_ADDRESSES_OFFSET + i * 2 * 3) &
                0xFFFF;
        /* Copy the buffer addresses for PRU Firmware usage*/
        HW_WR_REG16((pruicssHwAttrs->pru0DramBase + RTC_CPM_BUFFER_ADDRESSES_OFFSET + i * 2
               * 3), (pnHandle->cpmBlock).slots[i].addr[0] - (uint8_t *)
                      pnHandle->emacHandle->attrs->l3OcmcBaseAddr);
        HW_WR_REG16((pruicssHwAttrs->pru0DramBase + RTC_CPM_BUFFER_ADDRESSES_OFFSET + i * 2
               * 3 + 2), (pnHandle->cpmBlock).slots[i].addr[1] - (uint8_t *)
                          pnHandle->emacHandle->attrs->l3OcmcBaseAddr);
        HW_WR_REG16((pruicssHwAttrs->pru0DramBase + RTC_CPM_BUFFER_ADDRESSES_OFFSET + i * 2
               * 3 + 4), (pnHandle->cpmBlock).slots[i].addr[2] - (uint8_t *)
                          pnHandle->emacHandle->attrs->l3OcmcBaseAddr);
    }

    /* initialize the main data structure for the driver*/
    (pnHandle->currPN).maxPpmSize = ppmSize;
    (pnHandle->currPN).maxCpmSize = cpmSize;
    (pnHandle->currPN).cfgAR         = loops;      /* fixed ar*/

    for(i = 0; i < NO_CPM; i++)
    {
        PN_initPacket(&((pnHandle->currPN).cpmPkts[i]), CPM);
        PN_initPacket(&((pnHandle->currPN).ppmPkts[i]), PPM);
    }

    pnHandle->initRtcMemFlag = 1;

    return 0;           /* success*/
}

int32_t PN_deCfgRtcMem(PN_Handle pnHandle)
{
    /* Clear flag to signify that configuration has not been done */
    /* This will enable re-configuration */
    pnHandle->initRtcMemFlag = 0;
    return 0;
}


void PN_initRtcBlocks(PN_Handle pnHandle)
{
    uint8_t i;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);

    /* PRU configured block start addresses*/
    (pnHandle->ppmBlock[0]).blockStart = (uint8_t *)pruicssHwAttrs->pru0DramBase +
                                         PPM_BUFFER_OFFSET0;
    (pnHandle->ppmBlock[1]).blockStart = (uint8_t *)pruicssHwAttrs->pru1DramBase +
                                         PPM_BUFFER_OFFSET1;
    (pnHandle->cpmBlock).blockStart  = (uint8_t *)pnHandle->emacHandle->attrs->l3OcmcBaseAddr +
                                        CPM_BUFFER_OFFSET;

    for(i = 0; i < HALFSIZE; i++)
    {
        (pnHandle->ppmBlock[0]).slots[i].slotUsed = 0;      /* slot data is don't care*/
        (pnHandle->ppmBlock[1]).slots[i].slotUsed = 0;
    }

    for(i = 0; i < NO_CPM; i++)
    {
        (pnHandle->cpmBlock).slots[i].slotUsed = 0;
    }
}


void PN_initRtcBuffs(t_rtcMemBuff *pBuffDesc, uint8_t *offset, uint16_t size)
{
    uint8_t i;

    for(i = 0; i < 3; i++)
    {
        pBuffDesc->addr[i] = offset + size * i;
    }

    pBuffDesc->slotUsed = 0;
}

int32_t PN_delPmList(PN_Handle pnHandle, t_rtcPacket *pmPkt)
{
    t_descList *pList;
    int32_t index;
    uint8_t red = 0;

#ifdef RTC_DEBUG
   /*  newLogPkt(DELPM);*/
#endif

    if(pmPkt->frameId == 0 || pmPkt->frameId == 0xFFFF)
    {
        return -1;    /*invalid packet*/
    }

    if(pmPkt->pBuffer == NULL)
    {
        return -2;
    };                        /* already deleted?*/

    if(PPM == pmPkt->type)
    {
        pList = &(pnHandle->ppmList);
    }

    else if(CPM == pmPkt->type)
    {
        pList = &(pnHandle->cpmList);
    }

    else
    {
        return -7;
    }

    if(pmPkt->frameId == pList->redFID)
        if(pmPkt->frameOffset !=
                0)         /* this should care about the legacy startup green packet with RTC3 FID!*/
        {
            red = 1;    ;                        /*red packet removal*/
        }

    pmPkt->isActive = 0;

    if(pmPkt->type == PPM)
    {
        index = pmPkt->lIndex;
    }

    else
    {
        index = PN_setCpmState(pnHandle, pmPkt, 0);           /* disable RX first*/
    }

    if((index < 0) || (index >= (pnHandle->currPN).cfgAR))     /* already deleted?*/
    {
        return -1;
    }

    if(red)
    {
        pList->redFID = 0;                 /* ready to insert new red*/
#ifdef IRT_LEGACY_STARTUP_SUPPORT
        pnHandle->irtLegStateCall(pnHandle, (void *)READY);
        pnHandle->irtLegPktCall(pnHandle, NULL);
#endif

    }

    pmPkt->lIndex = 0xFF;                   /* invalidate index*/

    if(pmPkt->type == PPM)
    {
        if(PN_writeSortedList(pnHandle, (pnHandle->currPN).ppmPkts) < 0)
        {
            return -1;
        }

        PN_togglePpmList(pnHandle,
                         pmPkt, 0);               /* switch to new list and run!*/
    }

    else
    {
        t_rtcPacket nullPkt;
        nullPkt.frameId = 0;
        nullPkt.frameOffset = 0;
        nullPkt.length = 0;
        nullPkt.pBuffer = (t_rtcMemBuff *)0xFFFFFFFF;

        /* just clear the descriptor used previously*/
        if(PN_writeCpmDesc(pnHandle, &nullPkt, index) < 0)
        {
            return -1;
        }
    }

    return 0;
}

int32_t PN_insPpmList(PN_Handle pnHandle, t_rtcPacket *ppmPkt, uint8_t legMode)
{
    t_descList *pList;

#ifdef RTC_DEBUG
    /*  newLogPkt(INSPPM);*/
#endif

    /* basic param checks*/
    if((ppmPkt->frameId == 0) || (ppmPkt->frameId == 0xFFFF)) /* invalid frame ID*/
    {
        return -4;
    }

    if(ppmPkt->reduRatio ==
            0)              /* invalid - higher boundary masked out later*/
    {
        return -4;
    }

    if(ppmPkt->phase ==
            0)                  /*invalid - higher boundary masked out later*/
    {
        return -4;
    }

    if(ppmPkt->phase >  ppmPkt->reduRatio)  /* phase can't exceed RR*/
    {
        return -4;
    }

    if(ppmPkt->length >
            (pnHandle->currPN).maxPpmSize) /* max packet size as configured*/
    {
        return -4;
    }

    if((ppmPkt->port != ICSS_EMAC_PORT_1)
            && (ppmPkt->port != ICSS_EMAC_PORT_2))    /* driver needs port definition!*/
    {
        return -4;
    }

    pList = &(pnHandle->ppmList);

    if(ppmPkt->frameOffset == 0xFFFFFFFF)   /* FW special case for green*/
    {
        ppmPkt->frameOffset = 0;    /* driver requires 0 for green*/
    }

    if(ppmPkt->frameOffset > 0)
    {
        if(pList->redFID !=
                0)              /* always insert GREEN legacy startup packet with RTC3 FID first!*/
        {
            return -8;    /* single RED frame slot already used...*/
        }

        ppmPkt->frameOffset &= 0x3FFFFF;    /* RTC3 frame FSO, 22 bits*/
        pList->redFID = ppmPkt->frameId;    /* store the red packet FID*/
#ifdef IRT_LEGACY_STARTUP_SUPPORT

        if(legMode == 1)
        {
            if(pnHandle->irtLegPktCall != NULL)
            {
                pnHandle->irtLegPktCall(pnHandle,
                                        (void *)ppmPkt);   /* packet buffer required to be initialized!*/
            }

            else
            {
                return -9;    /*missing init for legacy mode*/
            }

            if(pnHandle->irtLegStateCall != NULL)
            {
                pnHandle->irtLegStateCall(pnHandle, (void *)SENDPPM);    /* will send asap...*/
            }

            else
            {
                return -9;   /* missing init for legacy mode*/
            }
        }

#endif
    }

    ppmPkt->isActive =
        1;           /* new activation needs to be set before sorting!*/

    if(PN_writeSortedList(pnHandle, (pnHandle->currPN).ppmPkts) < 0)
    {
        goto PPM_ERROR;
    }

    if(PN_togglePpmList(pnHandle,
                        ppmPkt, 0) < 0)   /* switch to new list finally and run!*/

    {
        goto PPM_ERROR;
    }

    return 0;

PPM_ERROR:
    ppmPkt->isActive = 0;   /*clear this again as action failed*/
    return -1;              /* inform stack..*/
}

int32_t PN_insCpmList(PN_Handle pnHandle, t_rtcPacket *cpmPkt)
{
    t_descList *pList;
    t_cpmDesc   tmp;
    t_cpmDesc   *tmpDesc = &tmp;
    uint32_t        i;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);


#ifdef RTC_DEBUG
    /*  newLogPkt(INSCPM);*/
#endif

    pList = &(pnHandle->cpmList);

    /* basic param checks*/
    if(cpmPkt->reduRatio ==
            0)                 /* invalid - higher boundary masked out later*/
    {
        return -4;
    }

    cpmPkt->length += 4;                       /* add space for CRC (VLAN?)*/

    if((cpmPkt->length) >
            (pnHandle->currPN).maxCpmSize)   /* max packet size as configured*/
    {
        return -5;
    }

    if(cpmPkt->frameOffset == 0xFFFFFFFF)       /* FW special case for green*/
    {
        cpmPkt->frameOffset = 0;    /* driver requires 0 for green*/
    }

    if(cpmPkt->frameOffset > 0)
    {
        if(pList->redFID != 0)
        {
            return -8;    /*single RED frame slot already used...*/
        }

        pList->redFID = cpmPkt->frameId;        /* store the red packet FID*/
    }

    for(i = 0; i < (pnHandle->currPN).cfgAR; i++)
    {
        int res = PN_readCpmDesc(pnHandle, tmpDesc, i);     /* read from active list*/

        if(res < 0)
        {
            return res;
        }

        if(0 == tmpDesc->FrameId)
        {
            if((res = PN_writeCpmDesc(pnHandle, cpmPkt, i)) < 0)        /* ACTIVE list now!*/
            {
                return res;
            }

            cpmPkt->lIndex = i;                 /*save the index*/
            PN_cpmBuffLock(pruicssHwAttrs, i, (buffLocks)cpmPkt->proc);
            res = PN_setCpmState(pnHandle, cpmPkt, 1);      /* auto activate!*/
            return res;
        }
    }

    /*ran out of CPM descriptor list entries!*/
    return -1;
}

int32_t PN_setPpmState(PRUICSS_HwAttrs const *pruicssHwAttrs,
                       t_rtcPacket *pPkt)
{
    uint8_t listIndex;
    uint8_t *pShadow = (uint8_t *)(pruicssHwAttrs->pru0DramBase +
                                   RTC_PPM_ACTIVE_SHADOW_OFFSET);

    listIndex = pPkt->lIndex;
    pShadow = pShadow + listIndex;

    if(0 == pPkt->isActive)
    {
        *pShadow = 0;

        if((pPkt->crGroup > 0) && (pPkt->crGroup < 9))
        {
            if(PN_resetARlink(pruicssHwAttrs, pPkt->crGroup, listIndex,
                              pPkt->type) < 0)
            {
                return -1;
            }
        }
    }

    else
    {
        if((pPkt->crGroup > 0) && (pPkt->crGroup < 9))
        {
            if(PN_setPPMARlink(pruicssHwAttrs, pPkt->crGroup, listIndex) < 0)
            {
                return -2;    /*failed to set AR group*/
            }
        }

        *pShadow = 1;
    }

    return listIndex;               /* ok, done and give back index*/
}


int32_t PN_setCpmState(PN_Handle pnHandle, t_rtcPacket *pPkt, uint8_t val)
{
    t_descList *pList;
    uint8_t listIndex;
    uint8_t mask;
    PRUICSS_HwAttrs const *pruicssHwAttrs = (PRUICSS_HwAttrs const *)(pnHandle->pruicssHandle->hwAttrs);


    if(PPM == pPkt->type)
    {
        pList = &(pnHandle->ppmList);
    }

    else if(CPM == pPkt->type)
    {
        pList = &(pnHandle->cpmList);
    }

    else
    {
        return -2;
    }

    listIndex = pPkt->lIndex;

    if(listIndex > (pnHandle->currPN).cfgAR)
    {
        return -3;    /*wrong index...*/
    }

    mask = 1 << listIndex;

    if(0 == val)
    {
        *pList->pActive &= ~mask;   /* clear active bit to disable PM*/

        /*Clear the Previous Cycle Counter value when CPM connection drops*/
        HW_WR_REG16((pruicssHwAttrs->pru0DramBase +
               CPM_PREV_CYCLE_COUNTER_OFFSET + ((mask - 1) << 1)), 0);

        if(pPkt->type == CPM)
        {
            *pList->pStatus &= ~mask;   /* MM Based on V1.1.0.4 sdk patch...*/
        }

        pPkt->isActive = val;       /* update internal state*/

        if((pPkt->crGroup > 0) && (pPkt->crGroup < 9))
        {
            if(PN_resetARlink(pruicssHwAttrs, pPkt->crGroup, listIndex, pPkt->type) < 0)
            {
                return -4;
            }
        }
    }

    else
    {
        if((pPkt->crGroup > 0) && (pPkt->crGroup < 9))
        {
            if(PN_setCPMARlink(pruicssHwAttrs, pPkt->crGroup, listIndex) < 0)
            {
                return -4;    /* failed to set AR group*/
            }
        }

        *pList->pActive |= mask;    /*set active bit to enable PM*/
        pPkt->isActive = val;       /* update internal state*/
    }

    return listIndex;           /* ok, done and give back index*/
}

/* TODO not used ? */
int32_t PN_getPkt(PN_Handle pnHandle, t_rtcPacket **pPkt, uint8_t pos,
                  uint8_t type)
{
    t_rtcPacket *temp;

    *pPkt = NULL;

    if(type > PPM)
    {
        return -1;    /*wrong type spec*/
    }

    if(pos > (pnHandle->currPN).cfgAR)
    {
        return -2;    /*index out of bounds*/
    }

    if(type == CPM)
    {
        temp = &((pnHandle->currPN).cpmPkts[pos]);
    }

    else
    {
        temp = &((pnHandle->currPN).ppmPkts[pos]);
    }

    if(temp->frameId == 0)
    {
        return -3;    /*not allocated...*/
    }

    *pPkt = temp;
    return 0;
}


int32_t PN_allocPkt(PN_Handle pnHandle, t_rtcPacket **pPkt, uint8_t type)
{
    int8_t nextSlot;
    uint8_t i;
    t_rtcPacket *packets;      /* pointer to array of available packet objects*/

    if(type > PPM)
    {
        return -1;    /*wrong type spec*/
    }

    if(type == CPM)
    {
        packets = (pnHandle->currPN).cpmPkts;
    }

    else
    {
        packets = (pnHandle->currPN).ppmPkts;
    }

    for(i = 0; i < (pnHandle->currPN).cfgAR;
            i++)      /* always configure at least 2 AR for legacy startup support*/
    {
        if(packets[i].frameId == 0)         /* find a free packet first*/
        {
            if(PN_initPacket(&packets[i], type) < 0)
            {
                return -3;                 /* init failure*/
            }

            else                            /* buffer assignments from pool*/
            {
                *pPkt = &packets[i];
                (*pPkt)->frameId = 0xFFFF; /* temporary FID to mark packet as allocated*/
                /* now assign the buffers*/
                nextSlot = PN_findFreeSlot(pnHandle, type, &packets[i].block);

                if(nextSlot < 0)
                {
                    return -5;    /*slot allocation issue*/
                }

                if(type == CPM)
                {
                    t_cpmBlock *pBlock;
                    pBlock = &(pnHandle->cpmBlock);
                    /*assign buffer mem to packet*/
                    packets[i].pBuffer = &pBlock->slots[nextSlot];
                    pBlock->slots[nextSlot].slotUsed = 1;
                }

                else
                {
                    t_ppmBlock *pBlock;

                    if(packets[i].block == 0)
                    {
                        pBlock = &(pnHandle->ppmBlock[0]);
                    }

                    else
                    {
                        pBlock = &(pnHandle->ppmBlock[1]);
                    }

                    /*assign buffer mem to packet*/
                    packets[i].pBuffer = &pBlock->slots[nextSlot];
                    pBlock->slots[nextSlot].slotUsed = 1;
                }
            }

            return i;       /* free packet found, return object and slot*/
        }
    }

    return -2;              /* no free packets available*/
}

int32_t PN_freePkt(t_rtcPacket *pPkt)
{
    if(pPkt == NULL)
    {
        return -2;
    }

    if(1 == pPkt->isActive)
    {
        return -3;    /* packet object still active*/
    }

    pPkt->frameId = 0;                  /* clear frame IDv*/
    pPkt->pBuffer->slotUsed = 0;/* mark the buffer slot as free*/
    pPkt->pBuffer = NULL;               /* packet has no more buffer*/

    return 0;
}

int32_t PN_initPacket(t_rtcPacket *pPkt, uint8_t type)
{
    if(type > PPM)
    {
        return -1;
    }

    pPkt->frameId = 0;
    pPkt->frameOffset = 0;
    pPkt->length = 0;
    pPkt->pBuffer = NULL;
    pPkt->phase = 1;            /*safe value...*/
    pPkt->port = ICSS_EMAC_PORT_0;         /* not a physical port*/
    pPkt->reduRatio = 0;
    pPkt->type = type;
    pPkt->crGroup = 0;
    pPkt->lIndex = 0xFF;
    pPkt->next = 0;
    pPkt->last = 1;
    pPkt->proc = 2;
    return 0;
}

int8_t PN_findFreeSlot(PN_Handle pnHandle, uint8_t pmTyp, uint8_t *pBlock)
{
    uint8_t i;

    if(pmTyp == PPM)
    {
        for(i = 0; i < ((pnHandle->currPN).cfgAR / 2); i++)
        {
            if((pnHandle->ppmBlock[1]).slots[i].slotUsed ==
                    0)      /* this always returns free slot in block1 first!*/
            {
                *pBlock = 1;
                return i;
            }

            else if((pnHandle->ppmBlock[0]).slots[i].slotUsed == FALSE)
            {
                *pBlock = 0;
                return i;
            }
        }

        return -2;              /* no free slot*/
    }

    else if(pmTyp == CPM)
    {
        for(i = 0; i < (pnHandle->currPN).cfgAR; i++)
        {
            if((pnHandle->cpmBlock).slots[i].slotUsed == 0)
            {
                *pBlock = 0;
                return i;
            }
        }

        return -2;              /* no free slot*/
    }

    return -1;              /* bad param pmTyp*/
}

int32_t PN_chgPpmBuffer(PN_Handle pnHandle, t_rtcPacket *ppmPkt)
{
    uint32_t *dest;
    int32_t ret;
    t_ppmDesc desc;
    uint32_t dstAddr;
    t_descList *pList;
    uint8_t buffIndex = ppmPkt->proc;
    uint32_t *addr;

    pList = &(pnHandle->ppmList);


    ret = PN_readPpmDesc(pnHandle, &desc, ppmPkt->lIndex,
                         ACTIVE_LIST);  /* active list read*/

    if(ret != 0)
    {
        return ret;    /* read failed, pass on error*/
    }

    dstAddr = (uint32_t)ppmPkt->pBuffer->addr[buffIndex];

    desc.FrameIndex = buffIndex;
    desc.FrameReference = (uint16_t)(dstAddr &
                                     0xFFFF);   /* new buffer address in PRU mem map!*/

    addr= pList->pDescs[pList->shadow ^ 1];  /*active list start address*/
    addr += ppmPkt->lIndex * DESC_WORD_LENGTH;          /*advance to correct offset*/
    dest = (uint32_t *)&desc;                             /* portability?*/
    *addr++ = *dest++;                                  /* copy 16 byte decriptor as 4 words*/
    *addr++ = *dest++;
    *addr++ = *dest++;
    *addr = *dest;
    return 0;
}

/*TODO: Review this function */
void PN_clearPruIRQ(PRUICSS_HwAttrs const *pruicssHwAttrs,
                    uint8_t irq_num)      /* event clearing*/
{
    HW_WR_REG32((pruicssHwAttrs->intcRegBase + CSL_ICSS_G_PR1_ICSS_INTC_INTC_SLV_ENA_STATUS_REG0), 1 << irq_num);
}

#ifdef IRT_LEGACY_STARTUP_SUPPORT
void PN_registerSetState(PN_Handle pnHandle, pnLegCallBack_t callBack)
{
    pnHandle->irtLegStateCall = callBack;
}

void PN_registerSetPkt(PN_Handle pnHandle, pnLegCallBack_t callBack)
{
    pnHandle->irtLegPktCall = callBack;
}
#endif /*IRT_LEGACY_STARTUP_SUPPORT*/
